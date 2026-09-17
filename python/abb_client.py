#!/usr/bin/env python3
"""Supervised TCP client for fixed targets and validated ABB trajectories."""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence


DEFAULT_HOST = "192.168.125.1"
DEFAULT_PORT = 55000
REQUEST_SIZE = 64
RESPONSE_TERMINATOR = b"#"
MAX_RESPONSE_SIZE = 80
TRAJECTORY_PROGRESS_INTERVAL = 25


class BridgeError(RuntimeError):
    """Base exception for bridge failures."""


class ProtocolError(BridgeError):
    """The peer sent data that does not conform to the bridge protocol."""


class ControllerError(BridgeError):
    """The RAPID bridge returned an ERR response."""


def encode_request(command: str) -> bytes:
    """Encode one command as an exactly 64-byte, space-padded ASCII frame."""
    normalized = " ".join(command.strip().upper().split())
    if not normalized:
        raise ValueError("command cannot be empty")

    try:
        payload = normalized.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("commands must contain ASCII characters only") from exc

    if len(payload) >= REQUEST_SIZE:
        raise ValueError(f"command must be shorter than {REQUEST_SIZE} bytes")

    return payload.ljust(REQUEST_SIZE, b" ")


class ResponseBuffer:
    """Collect a TCP byte stream and return #-terminated response frames."""

    def __init__(self) -> None:
        self._data = bytearray()

    def feed(self, chunk: bytes) -> None:
        self._data.extend(chunk)
        if RESPONSE_TERMINATOR not in self._data and len(self._data) > MAX_RESPONSE_SIZE:
            raise ProtocolError("controller response exceeded 80 bytes without terminator")

    def pop(self) -> Optional[str]:
        try:
            end = self._data.index(RESPONSE_TERMINATOR)
        except ValueError:
            if len(self._data) > MAX_RESPONSE_SIZE:
                raise ProtocolError("controller response exceeded 80 bytes without terminator")
            return None

        frame_length = end + 1
        if frame_length > MAX_RESPONSE_SIZE:
            raise ProtocolError("controller response exceeded 80 bytes")

        raw = bytes(self._data[:frame_length])
        del self._data[:frame_length]
        try:
            return raw.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ProtocolError("controller response was not ASCII") from exc


def parse_joints(response: str) -> List[float]:
    prefix = "OK JOINTS "
    if not response.startswith(prefix) or not response.endswith("#"):
        raise ProtocolError(f"unexpected joint response: {response!r}")

    fields = response[len(prefix) : -1].split(",")
    if len(fields) != 6:
        raise ProtocolError(f"expected 6 joint values, received {len(fields)}")

    try:
        return [float(field) for field in fields]
    except ValueError as exc:
        raise ProtocolError(f"invalid joint value in response: {response!r}") from exc


def encode_trajectory_point(joints: Sequence[float]) -> bytes:
    """Encode one six-axis trajectory point as a fixed-size request frame."""
    if len(joints) != 6:
        raise ValueError("trajectory points must contain exactly 6 joint values")

    values = [float(value) for value in joints]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("trajectory joint values must be finite")

    return encode_request("P " + " ".join(f"{value:.2f}" for value in values))


@dataclass(frozen=True)
class BridgeConfig:
    host: str = DEFAULT_HOST
    port: int = DEFAULT_PORT
    connect_timeout: float = 3.0
    response_timeout: float = 5.0
    motion_timeout: float = 120.0


class AbbTcpClient:
    """Synchronous request/response client. Motion commands are never retried."""

    def __init__(self, config: BridgeConfig) -> None:
        self.config = config
        self._socket: Optional[socket.socket] = None
        self._responses = ResponseBuffer()

    def connect(self) -> None:
        if self._socket is not None:
            return
        self._responses = ResponseBuffer()
        self._socket = socket.create_connection(
            (self.config.host, self.config.port),
            timeout=self.config.connect_timeout,
        )
        self._socket.settimeout(self.config.response_timeout)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
            self._socket = None

    def __enter__(self) -> "AbbTcpClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise BridgeError("not connected")
        return self._socket

    def _send(self, command: str) -> None:
        self._require_socket().sendall(encode_request(command))

    def _receive(self, timeout: float) -> str:
        sock = self._require_socket()
        deadline = time.monotonic() + timeout

        while True:
            response = self._responses.pop()
            if response is not None:
                return response

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out waiting for a complete controller response")

            sock.settimeout(remaining)
            try:
                chunk = sock.recv(4096)
            except socket.timeout as exc:
                raise TimeoutError("timed out waiting for the controller") from exc

            if not chunk:
                raise ConnectionError("controller closed the TCP connection")
            self._responses.feed(chunk)

    @staticmethod
    def _raise_if_error(response: str) -> None:
        if response.startswith("ERR "):
            raise ControllerError(response[:-1])

    def _request_one(self, command: str) -> str:
        self._send(command)
        response = self._receive(self.config.response_timeout)
        self._raise_if_error(response)
        return response

    def ping(self) -> None:
        response = self._request_one("PING")
        if response != "OK PONG#":
            raise ProtocolError(f"unexpected PING response: {response!r}")

    def get_state(self) -> str:
        response = self._request_one("GET_STATE")
        prefix = "OK STATE "
        if not response.startswith(prefix) or not response.endswith("#"):
            raise ProtocolError(f"unexpected state response: {response!r}")
        return response[len(prefix) : -1]

    def get_joints(self) -> List[float]:
        return parse_joints(self._request_one("GET_JOINTS"))

    def move(self, target: str) -> None:
        normalized_target = target.strip().upper()
        if normalized_target not in {"HOME", "TEST"}:
            raise ValueError("target must be HOME or TEST")

        self._send(f"MOVE {normalized_target}")
        accepted = self._receive(self.config.response_timeout)
        self._raise_if_error(accepted)
        expected_accepted = f"OK ACCEPTED {normalized_target}#"
        if accepted != expected_accepted:
            raise ProtocolError(f"unexpected motion acceptance: {accepted!r}")

        try:
            completed = self._receive(self.config.motion_timeout)
        except (TimeoutError, ConnectionError) as exc:
            raise BridgeError(
                "motion result is unknown; inspect the robot before issuing any "
                "new MOVE command, and never retry automatically"
            ) from exc

        self._raise_if_error(completed)
        expected_completed = f"OK DONE {normalized_target}#"
        if completed != expected_completed:
            raise ProtocolError(f"unexpected motion completion: {completed!r}")

    def play_trajectory(self, points: Sequence[Sequence[float]]) -> None:
        """Stream one validated trajectory. An accepted replay is never retried."""
        if len(points) < 2:
            raise ValueError("trajectory must contain at least 2 points")

        encoded_points = [encode_trajectory_point(point) for point in points]
        self._send(f"PLAY BEGIN {len(encoded_points)}")
        ready = self._receive(self.config.response_timeout)
        self._raise_if_error(ready)
        if ready != "OK READY FIRST#":
            raise ProtocolError(f"unexpected trajectory readiness response: {ready!r}")

        self._require_socket().sendall(encoded_points[0])
        accepted = self._receive(self.config.response_timeout)
        self._raise_if_error(accepted)
        if accepted != "OK ACCEPTED TRAJECTORY#":
            raise ProtocolError(f"unexpected trajectory acceptance: {accepted!r}")

        try:
            next_index = 1
            while next_index < len(encoded_points):
                batch_end = min(
                    (next_index // TRAJECTORY_PROGRESS_INTERVAL + 1)
                    * TRAJECTORY_PROGRESS_INTERVAL,
                    len(encoded_points),
                )
                self._require_socket().sendall(
                    b"".join(encoded_points[next_index:batch_end])
                )
                progress = self._receive(self.config.motion_timeout)
                self._raise_if_error(progress)
                expected_progress = f"OK RECEIVED {batch_end}#"
                if progress != expected_progress:
                    raise ProtocolError(
                        f"unexpected trajectory progress: {progress!r}"
                    )
                next_index = batch_end

            completed = self._receive(self.config.motion_timeout)
            self._raise_if_error(completed)
            if completed != "OK DONE TRAJECTORY#":
                raise ProtocolError(
                    f"unexpected trajectory completion response: {completed!r}"
                )
        except (ControllerError, ProtocolError, TimeoutError, ConnectionError, OSError) as exc:
            raise BridgeError(
                "trajectory result is unknown; inspect the robot and controller "
                "before issuing any new motion command, and never retry automatically. "
                f"Detail: {exc}"
            ) from exc

    def quit(self) -> None:
        if self._socket is None:
            return
        response = self._request_one("QUIT")
        if response != "OK BYE#":
            raise ProtocolError(f"unexpected QUIT response: {response!r}")
        self.close()


def _confirm_motion(target: str, assume_yes: bool) -> None:
    if assume_yes:
        return
    answer = input(
        f"Robot will move to {target}. Type {target} to confirm (anything else cancels): "
    )
    if answer.strip().upper() != target:
        raise BridgeError("motion cancelled by operator")


def _run_interactive(client: AbbTcpClient) -> None:
    print("Commands: ping, state, joints, home, test, quit")
    while True:
        command = input("abb> ").strip().lower()
        if command == "ping":
            client.ping()
            print("OK PONG")
        elif command == "state":
            print(client.get_state())
        elif command == "joints":
            print(", ".join(f"J{i}={value:.2f} deg" for i, value in enumerate(client.get_joints(), 1)))
        elif command in {"home", "test"}:
            target = command.upper()
            _confirm_motion(target, assume_yes=False)
            client.move(target)
            print(f"OK DONE {target}")
        elif command in {"quit", "exit"}:
            client.quit()
            print("OK BYE")
            return
        elif not command:
            continue
        else:
            print("Unknown command. Use: ping, state, joints, home, test, quit")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--response-timeout", type=float, default=5.0)
    parser.add_argument("--motion-timeout", type=float, default=120.0)

    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("ping")
    ping_loop = subparsers.add_parser("ping-loop")
    ping_loop.add_argument("--count", type=int, default=20)
    subparsers.add_parser("state")
    subparsers.add_parser("joints")

    for action in ("home", "test"):
        motion = subparsers.add_parser(action)
        motion.add_argument(
            "--yes",
            action="store_true",
            help="skip the typed target confirmation prompt",
        )

    subparsers.add_parser("interactive")
    return parser


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    config = BridgeConfig(
        host=args.host,
        port=args.port,
        connect_timeout=args.connect_timeout,
        response_timeout=args.response_timeout,
        motion_timeout=args.motion_timeout,
    )

    try:
        with AbbTcpClient(config) as client:
            if args.action == "ping":
                client.ping()
                print("OK PONG")
            elif args.action == "ping-loop":
                if args.count < 1:
                    raise ValueError("--count must be at least 1")
                for index in range(1, args.count + 1):
                    client.ping()
                    print(f"{index}/{args.count} OK PONG")
            elif args.action == "state":
                print(client.get_state())
            elif args.action == "joints":
                joints = client.get_joints()
                print(", ".join(f"J{i}={value:.2f} deg" for i, value in enumerate(joints, 1)))
            elif args.action in {"home", "test"}:
                target = args.action.upper()
                _confirm_motion(target, assume_yes=args.yes)
                client.move(target)
                print(f"OK DONE {target}")
            elif args.action == "interactive":
                _run_interactive(client)
    except (BridgeError, ConnectionError, OSError, TimeoutError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(run())
