import json
import tempfile
import unittest

import cv2
import numpy as np

from pathlib import Path

from ball_vision import BallVision, load_config, normalize_platform_corners


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


class PlatformGeometryTest(unittest.TestCase):
    def test_zero_reference_creates_independent_asymmetric_percent_edge_distances(self):
        vision = BallVision.__new__(BallVision)
        vision.config = {"platform": {}}
        vision.zero_reference = np.array([30.0, -20.0], dtype=np.float32)

        geometry = vision._platform_geometry()

        self.assertIsNotNone(geometry)
        self.assertEqual(geometry.as_command(), "GEOM,-130.00,70.00,-80.00,120.00")

    def test_contact_point_moves_toward_platform_near_edge_by_configured_radius_fraction(self):
        vision = BallVision.__new__(BallVision)
        vision.corners = [(100.0, 100.0), (300.0, 100.0), (320.0, 300.0), (80.0, 300.0)]
        vision.config = {"detection": {"contact_offset_radius": 0.85}}

        contact = vision._ball_contact_point((200.0, 180.0), 20.0)

        self.assertAlmostEqual(contact[0], 200.0)
        self.assertAlmostEqual(contact[1], 197.0)

    def test_position_outside_calibrated_platform_is_rejected_before_serial(self):
        vision = BallVision.__new__(BallVision)

        self.assertTrue(vision._position_is_inside_platform((100.0, -100.0)))
        self.assertFalse(vision._position_is_inside_platform((100.1, 0.0)))
        self.assertFalse(vision._position_is_inside_platform((0.0, -100.1)))

    def test_calculated_platform_center_creates_symmetric_geometry(self):
        vision = BallVision.__new__(BallVision)
        vision.config = {"platform": {}}
        vision.zero_reference = np.array([0.0, 0.0], dtype=np.float32)

        geometry = vision._platform_geometry()

        self.assertIsNotNone(geometry)
        self.assertEqual(geometry.as_command(), "GEOM,-100.00,100.00,-100.00,100.00")

    def test_homography_maps_calibrated_board_edges_to_percent_protocol(self):
        vision = BallVision.__new__(BallVision)
        vision.config = {"platform": {"corners_px": []}, "control": {}}
        vision.config_path = None
        vision.corners = [(10.0, 20.0), (210.0, 20.0), (210.0, 220.0), (10.0, 220.0)]
        vision.zero_reference = None
        vision.corner_order_message = None
        vision.corner_success_message = None

        vision._rebuild_homography()

        self.assertEqual(vision._platform_position((10.0, 20.0)), (-100.0, 100.0))
        self.assertEqual(vision._platform_position((210.0, 220.0)), (100.0, -100.0))

    def test_legacy_mm_zero_reference_is_ignored(self):
        with tempfile.NamedTemporaryFile("w+", encoding="utf-8", suffix=".json") as config_file:
            json.dump({"platform": {}, "control": {"zero_reference_mm": [10.0, 5.0]}}, config_file)
            config_file.flush()

            config = load_config(Path(config_file.name))

        self.assertNotIn("zero_reference_mm", config["control"])
        self.assertNotIn("zero_reference_pct", config["control"])


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
