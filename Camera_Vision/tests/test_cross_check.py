import unittest
from unittest.mock import patch

import cv2
import numpy as np

from ball_vision import BallVision


class CrossCheckTest(unittest.TestCase):
    def make_vision(self, corners=None):
        vision = BallVision.__new__(BallVision)
        vision.corners = corners or [(25., 25.), (375., 25.), (375., 375.), (25., 375.)]
        vision.config = {"platform": {}}
        vision.zero_reference = np.array([0., 0.], dtype=np.float32)
        vision.corner_order_message = None
        vision._rebuild_homography()
        return vision

    def make_frame(self, vertical_x=200, horizontal_y=200, second_line=True):
        frame = np.full((401, 401, 3), 235, dtype=np.uint8)
        cv2.line(frame, (vertical_x, 25), (vertical_x, 375), (0, 0, 0), 4)
        if second_line:
            cv2.line(frame, (25, horizontal_y), (375, horizontal_y), (0, 0, 0), 4)
        return frame

    def test_centred_black_cross(self):
        result = self.make_vision()._check_platform_cross(self.make_frame(), (200, 100))
        self.assertIsNotNone(result)
        point, dx, dy, _, _ = result
        self.assertAlmostEqual(point[0], 200, delta=5)
        self.assertAlmostEqual(point[1], 200, delta=5)
        self.assertAlmostEqual(dx, 0, delta=1.5)
        self.assertAlmostEqual(dy, 0, delta=1.5)

    def test_off_centre_reports_fraction_of_full_width_and_height(self):
        result = self.make_vision()._check_platform_cross(self.make_frame(235, 182), (235, 100))
        self.assertIsNotNone(result)
        _, dx, dy, _, _ = result
        self.assertAlmostEqual(dx, 10, delta=2)
        self.assertAlmostEqual(dy, 5.14, delta=2)

    def test_skewed_line_reports_angle_after_rectification(self):
        frame = self.make_frame(second_line=False)
        cv2.line(frame, (25, 160), (375, 240), (0, 0, 0), 4)
        result = self.make_vision()._check_platform_cross(frame, (200, 100))
        self.assertIsNotNone(result)
        _, _, _, horizontal_angle, _ = result
        self.assertGreater(abs(horizontal_angle), 8)

    def test_cross_check_does_not_change_corners_or_control_state(self):
        vision = self.make_vision()
        before = list(vision.corners)
        before_homography = vision.homography.copy()
        vision.mode = "cross_sample"
        vision.latest_frame = self.make_frame()
        vision.cross_check_marker = None
        vision._mouse_callback(cv2.EVENT_LBUTTONDOWN, 200, 100, 0, None)
        self.assertEqual(vision.mode, "normal")
        self.assertIsNotNone(vision.cross_check_marker)
        self.assertEqual(vision.corners, before)
        np.testing.assert_array_equal(vision.homography, before_homography)

    def test_no_cross_or_only_one_stroke_cannot_validate(self):
        vision = self.make_vision()
        self.assertIsNone(vision._check_platform_cross(self.make_frame(second_line=False), (200, 100)))
        self.assertIsNone(vision._check_platform_cross(np.full((401, 401, 3), 235, dtype=np.uint8), (200, 100)))

    def test_uniform_dark_scene_is_rejected(self):
        frame = np.zeros((401, 401, 3), dtype=np.uint8)
        self.assertIsNone(self.make_vision()._check_platform_cross(frame, (200, 100)))

    def test_perspective_warp_recovers_cross(self):
        board = self.make_frame()
        source = np.float32([(25, 25), (375, 25), (375, 375), (25, 375)])
        dest = np.float32([(110, 50), (300, 55), (380, 300), (30, 310)])
        transform = cv2.getPerspectiveTransform(source, dest)
        frame = cv2.warpPerspective(board, transform, (401, 401), borderValue=(235, 235, 235))
        sample = cv2.perspectiveTransform(np.float32([[[200, 100]]]), transform)[0, 0]
        result = self.make_vision([tuple(p) for p in dest])._check_platform_cross(frame, tuple(sample))
        self.assertIsNotNone(result)
        _, dx, dy, _, _ = result
        self.assertAlmostEqual(dx, 0, delta=2)
        self.assertAlmostEqual(dy, 0, delta=2)

    def test_overlay_has_only_single_status_line(self):
        vision = self.make_vision()
        vision.mode = "normal"
        vision.pending_corners = []
        vision.pid_running = False
        vision.run_start_phase = None
        vision.transport = type("Transport", (), {"is_connected": True, "joystick_target": None})()
        vision.config["ball_hsv"] = dict(hue=0, hue_tolerance=10, saturation_min=20, value_min=20)
        vision.config["detection"] = {"contact_offset_radius": .85}
        with patch("ball_vision.cv2.putText", wraps=cv2.putText) as put_text:
            vision._draw_overlay(self.make_frame(), ((200, 180), 20), (0, 0))
        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertEqual(set(labels), {"STATE: READY | NANO: OPEN"})


if __name__ == "__main__":
    unittest.main()
