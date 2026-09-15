import unittest

import numpy as np

from ball_vision import BallVision, normalize_platform_corners


class PlatformCornerGeometryTest(unittest.TestCase):
    def assert_normalized(self, raw, expected):
        normalized, problem = normalize_platform_corners(raw)
        self.assertIsNone(problem)
        self.assertEqual(normalized, [(float(x), float(y)) for x, y in expected])

    def assert_rejected(self, raw, expected_reason_part):
        normalized, problem = normalize_platform_corners(raw)
        self.assertIsNone(normalized)
        self.assertIsNotNone(problem)
        self.assertIn(expected_reason_part, problem)

    def test_sample_tl_tr_br_bl_is_accepted(self):
        self.assert_normalized(
            [(194, 82), (484, 92), (602, 266), (76, 251)],
            [(194, 82), (484, 92), (602, 266), (76, 251)],
        )

    def test_sample_tl_bl_br_tr_is_rejected_with_swapped_points(self):
        self.assert_rejected(
            [(194, 82), (76, 251), (602, 266), (484, 92)],
            "point 2 is BL but should be TR; point 4 is TR but should be BL",
        )

    def test_already_correct_order_is_kept(self):
        self.assert_normalized(
            [(100, 100), (500, 100), (520, 300), (80, 280)],
            [(100, 100), (500, 100), (520, 300), (80, 280)],
        )

    def test_duplicate_click_is_rejected(self):
        self.assert_rejected(
            [(100, 100), (100, 100), (500, 300), (80, 280)],
            "duplicate",
        )

    def test_degenerate_collinear_points_are_rejected(self):
        self.assert_rejected(
            [(100, 100), (200, 100), (300, 100), (400, 100)],
            "degenerate",
        )

    def test_concave_or_interior_point_is_rejected(self):
        self.assert_rejected(
            [(100, 100), (500, 100), (300, 160), (100, 300)],
            "concave",
        )

    def test_cross_like_uncertain_diamond_is_rejected(self):
        self.assert_rejected(
            [(300, 80), (520, 240), (300, 400), (80, 240)],
            "not geometrically clear",
        )


class CalibrationClickPointTest(unittest.TestCase):
    def make_vision(self):
        vision = BallVision.__new__(BallVision)
        vision.latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        return vision

    def test_click_inside_frame_is_kept_as_frame_pixel(self):
        self.assertEqual(self.make_vision()._calibration_click_point(639, 479), (639.0, 479.0))

    def test_click_outside_frame_is_rejected(self):
        self.assertIsNone(self.make_vision()._calibration_click_point(640, 480))


if __name__ == "__main__":
    unittest.main()
