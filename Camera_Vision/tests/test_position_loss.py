import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from test_run_handshake import make_vision


class PositionLossTest(unittest.TestCase):
    def setUp(self):
        self.vision, self.transport = make_vision()
        self.vision.config["transport"] = {"lost_heartbeat_hz": 5}
        self.vision.pid_running = True

    def send_at(self, now, position, reason="NO_DETECTION", outside=None):
        with patch("ball_vision.time.monotonic", return_value=now):
            self.vision._send_position(position, reason, outside)

    def test_one_missing_frame_recovers_without_old_position_or_stop(self):
        self.send_at(10.0, (15.0, 51.5))
        self.send_at(10.033, None)
        self.assertEqual(len(self.transport.sent), 1)
        self.send_at(10.066, (19.7, 53.4))
        self.assertEqual(len(self.transport.sent), 2)
        self.assertTrue(self.transport.sent[-1].startswith("POS,19.70,53.40,"))
        self.assertIsNone(self.vision.missing_ball_since)

    def test_sustained_missing_detection_sends_lost_and_reports_cause(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.send_at(10.0, None)
            self.send_at(10.07, None)
            self.assertEqual(self.transport.sent, [])
            self.send_at(10.081, None)
            self.send_at(10.10, None)
        self.assertEqual(self.transport.sent, ["LOST"])
        self.assertIn("reason=NO_DETECTION", output.getvalue())

    def test_outside_and_invalid_geometry_stop_immediately(self):
        for reason, position in [("OUTSIDE", (101.0, 53.4)), ("INVALID_GEOMETRY", None)]:
            with self.subTest(reason=reason):
                self.vision.last_lost_at = None
                self.transport.sent.clear()
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.send_at(10.0, None, reason, position)
                self.assertEqual(self.transport.sent, ["LOST"])
                self.assertIn("reason=" + reason, output.getvalue())
                if position:
                    self.assertIn("(101.00,53.40)%", output.getvalue())

    def test_outside_during_missing_grace_stops_immediately(self):
        self.send_at(10.0, None)
        with contextlib.redirect_stdout(io.StringIO()):
            self.send_at(10.01, None, "OUTSIDE", (101.0, 0.0))
        self.assertEqual(self.transport.sent, ["LOST"])

    def test_ready_does_not_send_or_restart(self):
        self.vision.pid_running = False
        self.send_at(10.0, None, "OUTSIDE", (101.0, 0.0))
        self.send_at(10.1, (0.0, 0.0))
        self.assertEqual(self.transport.sent, [])

    def test_outside_frame_contains_markers_and_is_saved_locally(self):
        with tempfile.TemporaryDirectory() as directory:
            self.vision.config_path = Path(directory) / "camera_config.json"
            self.vision.config["detection"] = {"contact_offset_radius": 1.0}
            self.vision.corners = [(20.0, 20.0), (180.0, 20.0), (180.0, 160.0), (20.0, 160.0)]
            self.vision.mode = "normal"
            self.transport.is_connected = True
            frame = np.zeros((200, 200, 3), dtype=np.uint8)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.vision._save_outside_frame(frame, ((100.0, 50.0), 10.0), (100.0, 60.0), (-188.51, 135.94))
            saved = cv2.imread(str(Path(directory) / "vision_last_outside.png"))
            self.assertIsNotNone(saved)
            self.assertEqual(tuple(saved[60, 100]), (0, 165, 255))
            self.assertIn("center_px=(100.0,50.0)", output.getvalue())


if __name__ == "__main__":
    unittest.main()
