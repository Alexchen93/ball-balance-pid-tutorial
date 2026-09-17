import unittest
from pathlib import Path


FIRMWARE = Path(__file__).resolve().parents[2] / "Arduino_Nano_Ball_Balance" / "Arduino_Nano_Ball_Balance.ino"


class FirmwareProtocolStaticTest(unittest.TestCase):
    def setUp(self):
        self.source = FIRMWARE.read_text(encoding="utf-8")

    def test_percent_protocol_and_bounded_geometry_are_documented(self):
        self.assertIn("GEOM,<x_min_pct>,<x_max_pct>,<y_min_pct>,<y_max_pct>", self.source)
        self.assertIn("POS,<x_pct>,<y_pct>,<camera_timestamp_ms>", self.source)
        self.assertIn("(xMax - xMin) > 250.0f", self.source)
        self.assertNotIn("(xMax - xMin) > 1000.0f", self.source)

    def test_pos_range_has_small_numeric_tolerance_and_clamps_accepted_edges(self):
        self.assertIn("POSITION_RANGE_EPSILON_PCT = 0.50f", self.source)
        self.assertIn("x >= positionMinX - POSITION_RANGE_EPSILON_PCT", self.source)
        self.assertIn("ballX = constrain(x, positionMinX, positionMaxX);", self.source)
        self.assertIn("Serial.println(F(\"ERROR,POSITION_OUT_OF_RANGE\"));", self.source)

    def test_each_valid_pos_can_drive_pid_in_run(self):
        self.assertIn("newPositionAvailable = true;", self.source)
        self.assertIn("if (!newPositionAvailable || controllerState != RUN)", self.source)
        self.assertIn("outputNormX = pidX.update(errorNormX, dtSeconds);", self.source)
        self.assertIn("updateServos();", self.source)


if __name__ == "__main__":
    unittest.main()
