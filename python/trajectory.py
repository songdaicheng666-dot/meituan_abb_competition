#!/usr/bin/env python3
"""Record ABB joint trajectories through read-only RWS and replay over TCP."""

from __future__ import annotations

import argparse
import base64
import getpass
import hashlib
import http.client
import json
import math
import os
import re
import ssl
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from http.cookies import SimpleCookie
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from xml.etree import ElementTree

try:
    from .abb_client import AbbTcpClient, BridgeConfig, BridgeError
except ImportError:  # Allows: python python\trajectory.py ...
    from abb_client import AbbTcpClient, BridgeConfig, BridgeError


DEFAULT_HOST = "192.168.125.1"
DEFAULT_RWS_PORT = 443
DEFAULT_SOCKET_PORT = 55000
DEFAULT_MECHUNIT = "ROB_1"
DEFAULT_SAMPLE_HZ = 20.0
SCHEMA_VERSION = 1

HOME_TOLERANCE_DEG = 0.2
STATIONARY_DELTA_DEG = 0.02
SIMPLIFY_TOLERANCE_DEG = 0.05
MAX_REPLAY_STEP_DEG = 1.0
MAX_SAMPLE_GAP_SECONDS = 0.250
MIN_MEDIAN_SAMPLE_HZ = 10.0

_FLOAT_PATTERN = re.compile(
    r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
)


class TrajectoryError(RuntimeError):
    """Base error for recording, validation, and replay."""


class RwsError(TrajectoryError):
    """Robot Web Services request or response failed."""


class RwsHttpError(RwsError):
    def __init__(self, status: int, reason: str, body: str) -> None:
        self.status = status
        self.reason = reason
        self.body = body
        super().__init__(f"RWS HTTP {status} {reason}: {body[:200]}")


@dataclass(frozen=True)
class RwsConfig:
    host: str
    port: int
    username: str
    password: str
    ca_cert: Optional[str] = None
    insecure: bool = False
    timeout: float = 3.0


@dataclass(frozen=True)
class JointSample:
    elapsed: float
    joints: Tuple[float, float, float, float, float, float]


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _span_values(xml_body: bytes) -> Dict[str, str]:
    try:
        root = ElementTree.fromstring(xml_body)
    except ElementTree.ParseError as exc:
        raise RwsError("RWS returned invalid XML") from exc

    values: Dict[str, str] = {}
    for element in root.iter():
        if _local_name(element.tag) != "span":
            continue
        class_name = element.attrib.get("class")
        if class_name and class_name not in values:
            values[class_name] = (element.text or "").strip()
    return values


def parse_jointtarget_xml(xml_body: bytes) -> Tuple[float, float, float, float, float, float]:
    values = _span_values(xml_body)
    names = [f"rax_{index}" for index in range(1, 7)]
    if not all(name in values for name in names):
        names = [f"j{index}" for index in range(1, 7)]
    if not all(name in values for name in names):
        raise RwsError("RWS jointtarget response does not contain six robot axes")

    try:
        joints = tuple(float(values[name]) for name in names)
    except ValueError as exc:
        raise RwsError("RWS jointtarget contains a non-numeric axis") from exc
    if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
        raise RwsError("RWS jointtarget contains invalid axis values")
    return joints  # type: ignore[return-value]


def parse_symbol_xml(xml_body: bytes) -> str:
    values = _span_values(xml_body)
    if "value" not in values:
        raise RwsError("RWS RAPID symbol response has no value")
    return values["value"]


def parse_jointtarget_value(value: str) -> List[float]:
    numbers = [float(match.group(0)) for match in _FLOAT_PATTERN.finditer(value)]
    if len(numbers) < 6 or not all(math.isfinite(number) for number in numbers[:6]):
        raise RwsError(f"invalid RAPID jointtarget value: {value!r}")
    return numbers[:6]


def parse_rapid_bool(value: str) -> bool:
    normalized = value.strip().upper()
    if normalized == "TRUE":
        return True
    if normalized == "FALSE":
        return False
    raise RwsError(f"invalid RAPID bool value: {value!r}")


class RwsClient:
    """Small read-only RobotWare 7 RWS 2.0 HTTPS client."""

    def __init__(self, config: RwsConfig) -> None:
        self.config = config
        self._connection: Optional[http.client.HTTPSConnection] = None
        self._cookies: Dict[str, str] = {}

    def _ssl_context(self) -> ssl.SSLContext:
        if self.config.insecure:
            return ssl._create_unverified_context()  # noqa: SLF001 - explicit CLI opt-in
        return ssl.create_default_context(cafile=self.config.ca_cert)

    def _new_connection(self) -> http.client.HTTPSConnection:
        return http.client.HTTPSConnection(
            self.config.host,
            self.config.port,
            timeout=self.config.timeout,
            context=self._ssl_context(),
        )

    def close(self) -> None:
        if self._connection is not None:
            self._connection.close()
            self._connection = None

    def __enter__(self) -> "RwsClient":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _headers(self) -> Dict[str, str]:
        credentials = f"{self.config.username}:{self.config.password}".encode("utf-8")
        headers = {
            "Accept": "application/xhtml+xml;v=2.0",
            "Authorization": "Basic " + base64.b64encode(credentials).decode("ascii"),
            "Connection": "keep-alive",
        }
        if self._cookies:
            headers["Cookie"] = "; ".join(
                f"{name}={value}" for name, value in self._cookies.items()
            )
        return headers

    def get(self, path: str) -> bytes:
        if not path.startswith("/"):
            raise ValueError("RWS path must be absolute")

        for attempt in range(2):
            if self._connection is None:
                self._connection = self._new_connection()
            try:
                self._connection.request("GET", path, headers=self._headers())
                response = self._connection.getresponse()
                body = response.read()
            except ssl.SSLCertVerificationError as exc:
                self.close()
                raise RwsError(
                    "RWS TLS certificate verification failed; use --ca-cert with "
                    "the trusted controller certificate, or explicitly use --insecure "
                    "only on an isolated lab network"
                ) from exc
            except (OSError, http.client.HTTPException, ssl.SSLError) as exc:
                self.close()
                if attempt == 0:
                    continue
                raise RwsError(f"cannot reach RWS at {self.config.host}:{self.config.port}") from exc

            cookies = SimpleCookie()
            for header, value in response.getheaders():
                if header.lower() == "set-cookie":
                    cookies.load(value)
            for name, morsel in cookies.items():
                self._cookies[name] = morsel.value

            if response.status < 200 or response.status >= 300:
                raise RwsHttpError(
                    response.status,
                    response.reason,
                    body.decode("utf-8", errors="replace"),
                )
            return body

        raise AssertionError("unreachable")

    def get_joint_target(self, mechunit: str = DEFAULT_MECHUNIT) -> Tuple[float, ...]:
        body = self.get(
            f"/rw/motionsystem/mechunits/{mechunit}/jointtarget?ignore=1"
        )
        return parse_jointtarget_xml(body)

    def get_rapid_symbol(self, module: str, symbol: str) -> str:
        candidates = [
            f"/rw/rapid/symbol/data/RAPID/T_ROB1/{module}/{symbol}",
            f"/rw/rapid/symbol/RAPID/T_ROB1/{module}/{symbol}/data",
        ]
        last_error: Optional[RwsHttpError] = None
        for path in candidates:
            try:
                return parse_symbol_xml(self.get(path))
            except RwsHttpError as exc:
                if exc.status not in {400, 404}:
                    raise
                last_error = exc
        raise RwsError(
            f"cannot read RAPID symbol {module}/{symbol}; verify that the module is loaded"
        ) from last_error

    def get_system_info(self) -> Dict[str, str]:
        values = _span_values(self.get("/rw/system"))
        return {
            "name": values.get("name", "unknown"),
            "system_id": values.get("sysid", "unknown"),
            "robotware": values.get("rwversion", "unknown"),
        }


def max_joint_delta(first: Sequence[float], second: Sequence[float]) -> float:
    if len(first) != 6 or len(second) != 6:
        raise ValueError("joint vectors must contain exactly 6 values")
    return max(abs(float(a) - float(b)) for a, b in zip(first, second))


def _remove_stationary_points(points: Sequence[Sequence[float]]) -> List[List[float]]:
    if not points:
        return []
    result = [list(map(float, points[0]))]
    for point in points[1:-1]:
        if max_joint_delta(result[-1], point) >= STATIONARY_DELTA_DEG:
            result.append(list(map(float, point)))
    if len(points) > 1:
        result.append(list(map(float, points[-1])))
    return result


def _point_segment_error(
    point: Sequence[float], start: Sequence[float], end: Sequence[float]
) -> float:
    segment = [b - a for a, b in zip(start, end)]
    denominator = sum(value * value for value in segment)
    if denominator == 0:
        return max_joint_delta(point, start)
    projection = sum((p - a) * d for p, a, d in zip(point, start, segment))
    fraction = min(1.0, max(0.0, projection / denominator))
    projected = [a + fraction * d for a, d in zip(start, segment)]
    return max_joint_delta(point, projected)


def _simplify_points(points: Sequence[Sequence[float]]) -> List[List[float]]:
    if len(points) <= 2:
        return [list(map(float, point)) for point in points]

    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        start_index, end_index = stack.pop()
        greatest_error = -1.0
        greatest_index = -1
        for index in range(start_index + 1, end_index):
            error = _point_segment_error(
                points[index], points[start_index], points[end_index]
            )
            if error > greatest_error:
                greatest_error = error
                greatest_index = index
        if greatest_error > SIMPLIFY_TOLERANCE_DEG:
            keep.add(greatest_index)
            stack.append((start_index, greatest_index))
            stack.append((greatest_index, end_index))
    return [list(map(float, points[index])) for index in sorted(keep)]


def _round_point(point: Sequence[float]) -> List[float]:
    return [round(float(value), 2) for value in point]


def prepare_playback_points(raw_points: Sequence[Sequence[float]]) -> List[List[float]]:
    if len(raw_points) < 2:
        raise TrajectoryError("recording must contain at least 2 samples")

    simplified = _simplify_points(_remove_stationary_points(raw_points))
    result = [_round_point(simplified[0])]
    step_limit_with_margin = 0.99
    for start, end in zip(simplified, simplified[1:]):
        subdivisions = max(
            1,
            math.ceil(max_joint_delta(start, end) / step_limit_with_margin),
        )
        for step in range(1, subdivisions + 1):
            fraction = step / subdivisions
            interpolated = [
                a + (b - a) * fraction for a, b in zip(start, end)
            ]
            rounded = _round_point(interpolated)
            if rounded != result[-1] or step == subdivisions:
                result.append(rounded)

    if len(result) < 2:
        result.append(list(result[0]))
    for first, second in zip(result, result[1:]):
        if max_joint_delta(first, second) > MAX_REPLAY_STEP_DEG + 1e-9:
            raise TrajectoryError("trajectory preprocessing produced an oversized joint step")
    return result


def trajectory_digest(points: Sequence[Sequence[float]]) -> str:
    canonical = json.dumps(points, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode("ascii")).hexdigest()


def _sampling_stats(samples: Sequence[JointSample]) -> Dict[str, float]:
    intervals = [
        current.elapsed - previous.elapsed
        for previous, current in zip(samples, samples[1:])
    ]
    if not intervals:
        return {"median_hz": 0.0, "max_gap_seconds": math.inf}
    median_interval = statistics.median(intervals)
    return {
        "median_hz": 1.0 / median_interval if median_interval > 0 else 0.0,
        "max_gap_seconds": max(intervals),
    }


def _coerce_samples(raw_samples: object) -> List[JointSample]:
    if not isinstance(raw_samples, list):
        raise TrajectoryError("raw_samples must be a list")
    result: List[JointSample] = []
    for entry in raw_samples:
        if not isinstance(entry, dict) or "t" not in entry or "j" not in entry:
            raise TrajectoryError("invalid raw sample")
        elapsed = float(entry["t"])
        joints = tuple(float(value) for value in entry["j"])
        if (
            not math.isfinite(elapsed)
            or len(joints) != 6
            or not all(math.isfinite(value) for value in joints)
        ):
            raise TrajectoryError("raw sample contains invalid values")
        result.append(JointSample(elapsed, joints))  # type: ignore[arg-type]
    return result


def validate_document(document: object) -> Tuple[List[str], Dict[str, float]]:
    errors: List[str] = []
    stats: Dict[str, float] = {}
    if not isinstance(document, dict):
        return ["trajectory root must be an object"], stats
    if document.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"unsupported schema_version: {document.get('schema_version')!r}")

    controller = document.get("controller")
    if not isinstance(controller, dict):
        errors.append("controller metadata is missing")
    else:
        if not isinstance(controller.get("host"), str) or not controller.get("host"):
            errors.append("controller host metadata is missing")
        if not isinstance(controller.get("mechunit"), str) or not controller.get("mechunit"):
            errors.append("controller mechunit metadata is missing")

    try:
        samples = _coerce_samples(document.get("raw_samples"))
    except (TrajectoryError, TypeError, ValueError) as exc:
        errors.append(str(exc))
        samples = []
    if len(samples) < 2:
        errors.append("trajectory must contain at least 2 raw samples")
    else:
        if any(
            current.elapsed <= previous.elapsed
            for previous, current in zip(samples, samples[1:])
        ):
            errors.append("raw sample timestamps must be strictly increasing")
        sampling = _sampling_stats(samples)
        stats.update(sampling)
        if sampling["median_hz"] < MIN_MEDIAN_SAMPLE_HZ:
            errors.append(
                f"median sample rate {sampling['median_hz']:.2f} Hz is below "
                f"{MIN_MEDIAN_SAMPLE_HZ:.2f} Hz"
            )
        if sampling["max_gap_seconds"] > MAX_SAMPLE_GAP_SECONDS:
            errors.append(
                f"sample gap {sampling['max_gap_seconds']:.3f}s exceeds "
                f"{MAX_SAMPLE_GAP_SECONDS:.3f}s"
            )

    reference = document.get("reference")
    py_home: Optional[List[float]] = None
    if not isinstance(reference, dict):
        errors.append("reference metadata is missing")
    else:
        try:
            py_home = [float(value) for value in reference.get("py_home", [])]
            if len(py_home) != 6 or not all(math.isfinite(value) for value in py_home):
                raise ValueError
        except (TypeError, ValueError):
            errors.append("reference py_home must contain 6 finite values")
            py_home = None
        if reference.get("py_home_captured") is not True:
            errors.append("recorded pyHomeCaptured is not TRUE")
        if reference.get("tool_state") != 2:
            errors.append("recorded tool_state is not 2")

    if samples and py_home is not None:
        start_error = max_joint_delta(samples[0].joints, py_home)
        end_error = max_joint_delta(samples[-1].joints, py_home)
        stats["start_home_error_deg"] = start_error
        stats["end_home_error_deg"] = end_error
        if start_error > HOME_TOLERANCE_DEG:
            errors.append(f"start is {start_error:.3f} deg away from pyHome")

    playback = document.get("playback_points")
    valid_playback: List[List[float]] = []
    if not isinstance(playback, list) or len(playback) < 2:
        errors.append("playback_points must contain at least 2 points")
    else:
        try:
            for point in playback:
                converted = [float(value) for value in point]
                if len(converted) != 6 or not all(
                    math.isfinite(value) for value in converted
                ):
                    raise ValueError
                valid_playback.append(converted)
        except (TypeError, ValueError):
            errors.append("playback_points contain invalid joint values")
            valid_playback = []

    if valid_playback:
        largest_step = max(
            max_joint_delta(first, second)
            for first, second in zip(valid_playback, valid_playback[1:])
        )
        stats["max_replay_step_deg"] = largest_step
        if largest_step > MAX_REPLAY_STEP_DEG + 1e-9:
            errors.append(
                f"playback joint step {largest_step:.3f} deg exceeds "
                f"{MAX_REPLAY_STEP_DEG:.3f} deg"
            )
        expected_digest = trajectory_digest(valid_playback)
        if document.get("playback_sha256") != expected_digest:
            errors.append("playback point checksum does not match")
        if py_home is not None:
            if max_joint_delta(valid_playback[0], py_home) > HOME_TOLERANCE_DEG:
                errors.append("first playback point is not pyHome")

    return errors, stats


def _load_document(path: Path) -> Dict[str, object]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise TrajectoryError(f"cannot read trajectory file {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise TrajectoryError("trajectory file root must be an object")
    return document


def _print_validation(errors: Sequence[str], stats: Dict[str, float]) -> None:
    if "median_hz" in stats:
        print(f"Median sample rate: {stats['median_hz']:.2f} Hz")
    if "max_gap_seconds" in stats:
        print(f"Maximum sample gap: {stats['max_gap_seconds']:.3f} s")
    if "start_home_error_deg" in stats:
        print(f"Start pyHome error: {stats['start_home_error_deg']:.3f} deg")
    if "end_home_error_deg" in stats:
        print(f"End pyHome error: {stats['end_home_error_deg']:.3f} deg")
    if "max_replay_step_deg" in stats:
        print(f"Maximum replay step: {stats['max_replay_step_deg']:.3f} deg")
    if errors:
        print("INVALID trajectory:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
    else:
        print("VALID trajectory")


def _read_rws_password(args: argparse.Namespace) -> str:
    password = os.environ.get("ABB_RWS_PASSWORD")
    if password is not None:
        return password
    return getpass.getpass(f"RWS password for {args.rws_user}: ")


def _rws_config(args: argparse.Namespace) -> RwsConfig:
    if args.insecure:
        print(
            "WARNING: TLS certificate verification is disabled for this RWS session.",
            file=sys.stderr,
        )
    return RwsConfig(
        host=args.host,
        port=args.rws_port,
        username=args.rws_user,
        password=_read_rws_password(args),
        ca_cert=args.ca_cert,
        insecure=args.insecure,
        timeout=args.rws_timeout,
    )


def _collect_for_duration(
    client: RwsClient, mechunit: str, seconds: float, hz: float
) -> List[JointSample]:
    samples: List[JointSample] = []
    start = time.monotonic()
    deadline = start
    period = 1.0 / hz
    while True:
        now = time.monotonic()
        if now - start >= seconds:
            break
        if now < deadline:
            time.sleep(deadline - now)
        sample_time = time.monotonic()
        joints = client.get_joint_target(mechunit)
        samples.append(JointSample(sample_time - start, joints))  # type: ignore[arg-type]
        deadline += period
        if deadline < time.monotonic() - period:
            deadline = time.monotonic()
    return samples


def run_rws_check(args: argparse.Namespace) -> int:
    config = _rws_config(args)
    with RwsClient(config) as client:
        system = client.get_system_info()
        py_home = parse_jointtarget_value(
            client.get_rapid_symbol("PythonBridge", "pyHome")
        )
        py_home_captured = parse_rapid_bool(
            client.get_rapid_symbol("PythonBridge", "pyHomeCaptured")
        )
        tool_state = float(client.get_rapid_symbol("Modul1", "toolState"))
        current = client.get_joint_target(args.mechunit)
        print(
            f"RWS connected: system={system['name']}, "
            f"RobotWare={system['robotware']}, mechunit={args.mechunit}"
        )
        print("Current joints: " + ", ".join(f"{value:.2f}" for value in current))
        print(f"toolState={tool_state:g}, pyHome error={max_joint_delta(current, py_home):.3f} deg")
        print(f"Sampling for {args.seconds:g} seconds; keep the robot stationary...")
        samples = _collect_for_duration(
            client, args.mechunit, args.seconds, args.hz
        )

    stats = _sampling_stats(samples)
    axis_ranges = [
        max(sample.joints[axis] for sample in samples)
        - min(sample.joints[axis] for sample in samples)
        for axis in range(6)
    ] if samples else [math.inf] * 6
    print(f"Samples: {len(samples)}")
    print(f"Median sample rate: {stats['median_hz']:.2f} Hz")
    print(f"Maximum sample gap: {stats['max_gap_seconds']:.3f} s")
    print("Stationary axis ranges: " + ", ".join(f"{value:.3f}" for value in axis_ranges))

    if not py_home_captured:
        print("ERROR: pyHomeCaptured is FALSE", file=sys.stderr)
        return 1
    if tool_state != 2:
        print("ERROR: toolState is not 2", file=sys.stderr)
        return 1
    if stats["median_hz"] < MIN_MEDIAN_SAMPLE_HZ:
        print("ERROR: RWS sampling rate is too low", file=sys.stderr)
        return 1
    if stats["max_gap_seconds"] > MAX_SAMPLE_GAP_SECONDS:
        print("ERROR: RWS sampling gap is too large", file=sys.stderr)
        return 1
    if max(axis_ranges) > 0.1:
        print("WARNING: robot did not appear stationary during rws-check", file=sys.stderr)
    print("RWS CHECK PASSED")
    return 0


def _record_worker(
    config: RwsConfig,
    mechunit: str,
    hz: float,
    stop_event: threading.Event,
    part_path: Path,
    result: Dict[str, object],
) -> None:
    period = 1.0 / hz
    start = time.monotonic()
    deadline = start
    count = 0
    try:
        with RwsClient(config) as client, part_path.open(
            "w", encoding="utf-8", newline="\n"
        ) as stream:
            while not stop_event.is_set():
                now = time.monotonic()
                if now < deadline:
                    stop_event.wait(deadline - now)
                    if stop_event.is_set():
                        break
                sample_time = time.monotonic()
                joints = client.get_joint_target(mechunit)
                entry = {"t": sample_time - start, "j": list(joints)}
                stream.write(json.dumps(entry, separators=(",", ":")) + "\n")
                stream.flush()
                count += 1
                deadline += period
                if deadline < time.monotonic() - period:
                    deadline = time.monotonic()
    except Exception as exc:  # Propagated to the main CLI thread.
        result["error"] = exc
        stop_event.set()
    finally:
        result["count"] = count


def _read_part_samples(path: Path) -> List[JointSample]:
    samples: List[JointSample] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                entry = json.loads(line)
                elapsed = float(entry["t"])
                joints = tuple(float(value) for value in entry["j"])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise TrajectoryError(
                    f"invalid temporary sample at line {line_number}"
                ) from exc
            if len(joints) != 6:
                raise TrajectoryError(
                    f"temporary sample at line {line_number} does not have 6 axes"
                )
            samples.append(JointSample(elapsed, joints))  # type: ignore[arg-type]
    return samples


def run_record(args: argparse.Namespace) -> int:
    output = Path(args.file)
    part_path = output.with_suffix(output.suffix + ".part")
    temp_path = output.with_suffix(output.suffix + ".tmp")
    if output.exists() and not args.overwrite:
        raise TrajectoryError(f"{output} already exists; use --overwrite to replace it")
    output.parent.mkdir(parents=True, exist_ok=True)
    if args.overwrite:
        for stale in (part_path, temp_path):
            if stale.exists():
                stale.unlink()

    config = _rws_config(args)
    with RwsClient(config) as client:
        system = client.get_system_info()
        py_home = parse_jointtarget_value(
            client.get_rapid_symbol("PythonBridge", "pyHome")
        )
        py_home_captured = parse_rapid_bool(
            client.get_rapid_symbol("PythonBridge", "pyHomeCaptured")
        )
        tool_state = float(client.get_rapid_symbol("Modul1", "toolState"))
        current = client.get_joint_target(args.mechunit)

    if not py_home_captured:
        raise TrajectoryError("pyHomeCaptured is FALSE; recapture pyHome before recording")
    if tool_state != 2:
        raise TrajectoryError(f"toolState must be 2, received {tool_state:g}")
    home_error = max_joint_delta(current, py_home)
    if home_error > HOME_TOLERANCE_DEG:
        raise TrajectoryError(
            f"robot is not at pyHome (maximum joint error {home_error:.3f} deg)"
        )

    input("Robot is at pyHome. Press ENTER to arm recording...")
    for remaining in range(args.countdown, 0, -1):
        print(f"Recording starts in {remaining}...")
        time.sleep(1)

    stop_event = threading.Event()
    result: Dict[str, object] = {}
    worker = threading.Thread(
        target=_record_worker,
        args=(config, args.mechunit, args.hz, stop_event, part_path, result),
        daemon=True,
    )
    worker.start()
    try:
        input(
            "RECORDING. Stop at the desired safe endpoint, then press ENTER "
            "to finish.\n"
        )
    finally:
        stop_event.set()
        worker.join(timeout=max(5.0, config.timeout + 2.0))

    if worker.is_alive():
        raise TrajectoryError("RWS recorder did not stop cleanly; temporary file was retained")
    if "error" in result:
        raise TrajectoryError(f"recording failed: {result['error']}") from result["error"]  # type: ignore[arg-type]

    samples = _read_part_samples(part_path)
    playback_points = prepare_playback_points([sample.joints for sample in samples])
    sampling = _sampling_stats(samples)
    document: Dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "controller": {
            "host": args.host,
            "system_name": system["name"],
            "system_id": system["system_id"],
            "robotware": system["robotware"],
            "mechunit": args.mechunit,
        },
        "recording": {
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "requested_hz": args.hz,
            "median_hz": sampling["median_hz"],
            "max_gap_seconds": sampling["max_gap_seconds"],
            "duration_seconds": samples[-1].elapsed if samples else 0.0,
        },
        "reference": {
            "tool_state": int(tool_state),
            "py_home_captured": py_home_captured,
            "py_home": py_home,
        },
        "constraints": {
            "home_tolerance_deg": HOME_TOLERANCE_DEG,
            "max_replay_step_deg": MAX_REPLAY_STEP_DEG,
            "max_sample_gap_seconds": MAX_SAMPLE_GAP_SECONDS,
        },
        "raw_samples": [
            {"t": sample.elapsed, "j": list(sample.joints)} for sample in samples
        ],
        "playback_points": playback_points,
        "playback_sha256": trajectory_digest(playback_points),
    }
    errors, stats = validate_document(document)
    document["validation"] = {
        "valid": not errors,
        "errors": errors,
        "checked_utc": datetime.now(timezone.utc).isoformat(),
    }

    with temp_path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    os.replace(temp_path, output)
    part_path.unlink()

    print(
        f"Saved {len(samples)} raw samples and {len(playback_points)} playback "
        f"points to {output}"
    )
    _print_validation(errors, stats)
    return 1 if errors else 0


def run_validate(args: argparse.Namespace) -> int:
    document = _load_document(Path(args.file))
    errors, stats = validate_document(document)
    _print_validation(errors, stats)
    return 1 if errors else 0


def run_play(args: argparse.Namespace) -> int:
    document = _load_document(Path(args.file))
    errors, stats = validate_document(document)
    _print_validation(errors, stats)
    if errors:
        raise TrajectoryError("refusing to play an invalid trajectory")

    controller = document["controller"]
    assert isinstance(controller, dict)
    if controller.get("host") != args.host:
        raise TrajectoryError(
            f"trajectory was recorded from {controller.get('host')}, not {args.host}"
        )
    if controller.get("mechunit") != args.mechunit:
        raise TrajectoryError(
            f"trajectory was recorded from {controller.get('mechunit')}, "
            f"not {args.mechunit}"
        )

    playback = document["playback_points"]
    assert isinstance(playback, list)
    answer = input(
        f"Robot will replay {len(playback)} points and must remain supervised. "
        "Type PLAY to confirm: "
    )
    if answer.strip().upper() != "PLAY":
        raise TrajectoryError("trajectory replay cancelled")

    config = BridgeConfig(
        host=args.host,
        port=args.socket_port,
        connect_timeout=args.connect_timeout,
        response_timeout=args.response_timeout,
        motion_timeout=args.motion_timeout,
    )
    with AbbTcpClient(config) as client:
        client.play_trajectory(playback)
    print("OK DONE TRAJECTORY")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--mechunit", default=DEFAULT_MECHUNIT)
    parser.add_argument("--rws-port", type=int, default=DEFAULT_RWS_PORT)
    parser.add_argument("--rws-user", default=os.environ.get("ABB_RWS_USER", "Default User"))
    parser.add_argument("--rws-timeout", type=float, default=3.0)
    parser.add_argument("--ca-cert")
    parser.add_argument("--insecure", action="store_true")
    parser.add_argument("--socket-port", type=int, default=DEFAULT_SOCKET_PORT)
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--response-timeout", type=float, default=5.0)
    parser.add_argument("--motion-timeout", type=float, default=120.0)

    subparsers = parser.add_subparsers(dest="action", required=True)
    check = subparsers.add_parser("rws-check")
    check.add_argument("--seconds", type=float, default=10.0)
    check.add_argument("--hz", type=float, default=DEFAULT_SAMPLE_HZ)

    record = subparsers.add_parser("record")
    record.add_argument("--file", default="trajectory.json")
    record.add_argument("--hz", type=float, default=DEFAULT_SAMPLE_HZ)
    record.add_argument("--countdown", type=int, default=5)
    record.add_argument("--overwrite", action="store_true")

    validate = subparsers.add_parser("validate")
    validate.add_argument("--file", default="trajectory.json")

    play = subparsers.add_parser("play")
    play.add_argument("--file", default="trajectory.json")
    return parser


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if hasattr(args, "hz") and args.hz <= 0:
        print("ERROR: --hz must be greater than zero", file=sys.stderr)
        return 1
    if getattr(args, "seconds", 1.0) <= 0:
        print("ERROR: --seconds must be greater than zero", file=sys.stderr)
        return 1
    if getattr(args, "countdown", 0) < 0:
        print("ERROR: --countdown cannot be negative", file=sys.stderr)
        return 1

    try:
        if args.action == "rws-check":
            return run_rws_check(args)
        if args.action == "record":
            return run_record(args)
        if args.action == "validate":
            return run_validate(args)
        if args.action == "play":
            return run_play(args)
    except (TrajectoryError, BridgeError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    raise AssertionError("unreachable")


if __name__ == "__main__":
    raise SystemExit(run())
