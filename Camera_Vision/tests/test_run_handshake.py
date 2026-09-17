import contextlib
import io
import time
import unittest

import numpy as np

from ball_vision import BallVision


class FakeTransport:
    def __init__(self):
        self.sent = []
        self.events = []
        self.discard_count = 0

    def send(self, line):
        self.sent.append(line)
        return True

    def discard_pending_input(self):
        self.discard_count += 1

    def poll(self):
        events = self.events
        self.events = []
        return events


def make_vision():
    transport = FakeTransport()
    vision = BallVision.__new__(BallVision)
    vision.transport = transport
    vision.pid_running = False
    vision.run_start_phase = None
    vision.run_start_deadline = 0.0
    vision.run_start_ack_position = None
    vision.run_start_pos_sent_at = 0.0
    vision.run_start_events_seen = 0
    vision.run_start_last_event = None
    vision.latest_position = (12.34, -5.67)
    vision.config = {"platform": {}}
    vision.zero_reference = np.array([30.0, -20.0], dtype=np.float32)
    vision.filtered_position = None
    vision.last_sent_at = 0.0
    vision.last_lost_at = 0.0
    return vision, transport


class RunHandshakeTest(unittest.TestCase):
    def test_state_ready_compatibility_still_waits_for_state_run(self):
        vision, transport = make_vision()

        vision._request_run((12.34, -5.67))
        self.assertEqual(transport.discard_count, 1)
        self.assertEqual(transport.sent, ["GEOM,-130.00,70.00,-80.00,120.00"])
        self.assertEqual(vision.run_start_phase, "wait_geom_ack")

        transport.events = ["GEOM,OK"]
        vision._handle_nano_events()
        self.assertEqual(vision.run_start_phase, "wait_pos_ack")
        self.assertEqual(len(transport.sent), 2)
        self.assertTrue(transport.sent[1].startswith("POS,12.34,-5.67,"))

        transport.events = ["STATE,READY"]
        vision._handle_nano_events()
        self.assertEqual(vision.run_start_phase, "wait_run_ack")
        self.assertFalse(vision.pid_running)
        self.assertEqual(len(transport.sent), 4)
        self.assertTrue(transport.sent[2].startswith("POS,12.34,-5.67,"))
        self.assertEqual(transport.sent[3], "RUN")

        transport.events = ["STATE,RUN"]
        vision._handle_nano_events()
        self.assertIsNone(vision.run_start_phase)
        self.assertTrue(vision.pid_running)

    def test_geom_error_cancels_run_without_legacy_fallback(self):
        vision, transport = make_vision()

        vision._request_run((12.34, -5.67))
        transport.events = ["ERROR,UNKNOWN_COMMAND"]
        vision._handle_nano_events()

        self.assertEqual(transport.sent, ["GEOM,-130.00,70.00,-80.00,120.00", "READY"])
        self.assertIsNone(vision.run_start_phase)
        self.assertFalse(vision.pid_running)

    def test_stale_state_ready_does_not_acknowledge_pos(self):
        vision, transport = make_vision()
        vision.run_start_phase = "wait_pos_ack"
        vision.run_start_ack_position = (12.34, -5.67)
        vision.run_start_pos_sent_at = time.monotonic() - 1.0
        vision.run_start_deadline = time.monotonic() + 10.0

        transport.events = ["STATE,READY"]
        vision._handle_nano_events()

        self.assertEqual(transport.sent, [])
        self.assertEqual(vision.run_start_phase, "wait_pos_ack")

    def test_mismatched_ready_telemetry_does_not_fall_through_to_state_ack(self):
        vision, transport = make_vision()
        vision.run_start_phase = "wait_pos_ack"
        vision.run_start_ack_position = (12.34, -5.67)
        vision.run_start_pos_sent_at = time.monotonic()
        vision.run_start_deadline = time.monotonic() + 10.0

        transport.events = [
            "TEL,READY,OK,99.9,88.8,0,0,0,0,90,90,10",
            "TEL_STATE,READY",
        ]
        vision._handle_nano_events()

        self.assertEqual(transport.sent, [])
        self.assertEqual(vision.run_start_phase, "wait_pos_ack")

    def test_pos_ack_timeout_reports_no_nano_response(self):
        vision, transport = make_vision()
        vision.run_start_phase = "wait_pos_ack"
        vision.run_start_ack_position = (12.34, -5.67)
        vision.run_start_pos_sent_at = time.monotonic() - 1.0
        vision.run_start_deadline = time.monotonic() - 0.1

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            vision._handle_nano_events()

        self.assertIn("No Nano response was parsed after Python sent POS", output.getvalue())
        self.assertEqual(transport.sent, ["READY"])
        self.assertIsNone(vision.run_start_phase)

    def test_pos_ack_timeout_reports_last_non_ack_event(self):
        vision, transport = make_vision()
        vision.run_start_phase = "wait_pos_ack"
        vision.run_start_ack_position = (12.34, -5.67)
        vision.run_start_pos_sent_at = time.monotonic()
        vision.run_start_deadline = time.monotonic() - 0.1
        transport.events = ["PONG"]

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            vision._handle_nano_events()

        self.assertIn("Python parsed 1 Nano event(s); last event was 'PONG'", output.getvalue())
        self.assertEqual(transport.sent, ["READY"])
        self.assertIsNone(vision.run_start_phase)


if __name__ == "__main__":
    unittest.main()
