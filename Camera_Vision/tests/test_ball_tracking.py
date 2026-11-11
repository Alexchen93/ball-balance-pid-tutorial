import unittest

import cv2
import numpy as np

from ball_vision import BallVision


class BallTrackingTest(unittest.TestCase):
    def setUp(self):
        self.vision = BallVision.__new__(BallVision)
        self.vision.config = {
            "ball_hsv": {"hue": 0, "hue_tolerance": 12, "saturation_min": 180, "value_min": 172},
            "detection": {"min_radius_px": 4, "max_radius_px": 80, "morph_kernel_px": 5},
        }
        self.vision.pid_running = True
        self.vision.last_valid_ball_center = (120.0, 120.0)

    def test_run_prefers_ball_near_last_position_over_larger_distractor(self):
        frame = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.circle(frame, (125, 120), 20, (0, 0, 255), -1)
        cv2.circle(frame, (300, 60), 50, (0, 0, 255), -1)
        center, _ = self.vision._detect_ball(frame)
        self.assertAlmostEqual(center[0], 125.0, delta=1.0)
        self.assertAlmostEqual(center[1], 120.0, delta=1.0)

    def test_only_candidate_is_detected_even_if_far_from_last_position(self):
        # Do not hide a real ball that has moved out of the platform.
        frame = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.circle(frame, (300, 60), 20, (0, 0, 255), -1)
        center, _ = self.vision._detect_ball(frame)
        self.assertAlmostEqual(center[0], 300.0, delta=1.0)

    def test_dark_seam_is_joined_before_nearest_candidate_selection(self):
        frame = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.circle(frame, (125, 120), 40, (0, 0, 255), -1)
        cv2.line(frame, (85, 120), (165, 120), (0, 0, 0), 4)
        self.vision.last_valid_ball_center = (125.0, 100.0)
        center, radius = self.vision._detect_ball(frame)
        self.assertAlmostEqual(center[0], 125.0, delta=1.0)
        self.assertAlmostEqual(center[1], 120.0, delta=1.0)
        self.assertAlmostEqual(radius, 40.0, delta=1.0)

    def test_ready_keeps_original_largest_candidate_selection(self):
        self.vision.pid_running = False
        frame = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.circle(frame, (125, 120), 20, (0, 0, 255), -1)
        cv2.circle(frame, (300, 60), 50, (0, 0, 255), -1)
        center, _ = self.vision._detect_ball(frame)
        self.assertAlmostEqual(center[0], 300.0, delta=1.0)


if __name__ == "__main__":
    unittest.main()
