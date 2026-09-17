import unittest

from python.trajectory import (
    HOME_TOLERANCE_DEG,
    MAX_REPLAY_STEP_DEG,
    max_joint_delta,
    parse_jointtarget_value,
    parse_jointtarget_xml,
    prepare_playback_points,
    trajectory_digest,
    validate_document,
)


class RwsParsingTests(unittest.TestCase):
    def test_parses_mechanical_unit_jointtarget(self):
        body = b"""<?xml version='1.0'?>
        <html xmlns='http://www.w3.org/1999/xhtml'><body><div><ul>
          <li class='ms-jointtarget'>
            <span class='rax_1'>1.25</span><span class='rax_2'>-2</span>
            <span class='rax_3'>3</span><span class='rax_4'>4</span>
            <span class='rax_5'>5</span><span class='rax_6'>6.5</span>
          </li>
        </ul></div></body></html>"""
        self.assertEqual(parse_jointtarget_xml(body), (1.25, -2, 3, 4, 5, 6.5))

    def test_parses_rapid_jointtarget_value(self):
        value = "[[1,-2,3.5,4,5,6],[9E9,9E9,9E9,9E9,9E9,9E9]]"
        self.assertEqual(parse_jointtarget_value(value), [1, -2, 3.5, 4, 5, 6])


class ProcessingTests(unittest.TestCase):
    def test_simplifies_and_subdivides_open_path(self):
        raw = [
            [0, 0, 0, 0, 0, 0],
            [0.001, 0, 0, 0, 0, 0],
            [2.5, 0, 0, 0, 0, 0],
        ]
        playback = prepare_playback_points(raw)
        self.assertEqual(playback[0], raw[0])
        self.assertEqual(playback[-1], raw[-1])
        self.assertTrue(
            all(
                max_joint_delta(first, second) <= MAX_REPLAY_STEP_DEG
                for first, second in zip(playback, playback[1:])
            )
        )

    def test_validates_complete_document_and_detects_tampering(self):
        home = [0, 0, 0, 0, 0, 0]
        raw_points = [[index * 0.05, 0, 0, 0, 0, 0] for index in range(21)]
        playback = prepare_playback_points(raw_points)
        document = {
            "schema_version": 1,
            "controller": {"host": "192.168.125.1", "mechunit": "ROB_1"},
            "reference": {
                "tool_state": 2,
                "py_home_captured": True,
                "py_home": home,
            },
            "raw_samples": [
                {"t": index * 0.05, "j": point}
                for index, point in enumerate(raw_points)
            ],
            "playback_points": playback,
            "playback_sha256": trajectory_digest(playback),
        }
        errors, stats = validate_document(document)
        self.assertEqual(errors, [])
        self.assertLessEqual(stats["start_home_error_deg"], HOME_TOLERANCE_DEG)
        self.assertGreater(stats["end_home_error_deg"], HOME_TOLERANCE_DEG)

        document["playback_points"][1][0] += 0.01
        errors, _ = validate_document(document)
        self.assertIn("playback point checksum does not match", errors)

    def test_rejects_trajectory_that_does_not_start_at_home(self):
        home = [0, 0, 0, 0, 0, 0]
        raw_points = [[1 + index * 0.05, 0, 0, 0, 0, 0] for index in range(21)]
        playback = prepare_playback_points(raw_points)
        document = {
            "schema_version": 1,
            "controller": {"host": "192.168.125.1", "mechunit": "ROB_1"},
            "reference": {
                "tool_state": 2,
                "py_home_captured": True,
                "py_home": home,
            },
            "raw_samples": [
                {"t": index * 0.05, "j": point}
                for index, point in enumerate(raw_points)
            ],
            "playback_points": playback,
            "playback_sha256": trajectory_digest(playback),
        }
        errors, _ = validate_document(document)
        self.assertTrue(any("start is" in error for error in errors))
        self.assertIn("first playback point is not pyHome", errors)

    def test_rejects_large_sampling_gap(self):
        home = [0, 0, 0, 0, 0, 0]
        playback = [home, home]
        document = {
            "schema_version": 1,
            "controller": {"host": "192.168.125.1", "mechunit": "ROB_1"},
            "reference": {
                "tool_state": 2,
                "py_home_captured": True,
                "py_home": home,
            },
            "raw_samples": [
                {"t": 0.0, "j": home},
                {"t": 0.5, "j": home},
            ],
            "playback_points": playback,
            "playback_sha256": trajectory_digest(playback),
        }
        errors, _ = validate_document(document)
        self.assertTrue(any("sample gap" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
