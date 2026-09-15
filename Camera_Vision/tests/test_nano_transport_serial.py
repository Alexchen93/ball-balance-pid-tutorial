import io
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import serial

from ball_vision import NanoTransport


class ExistingConnection:
    def __init__(self):
        self.closed = False
        self.writes = []

    def write(self, data):
        self.writes.append(data)

    def close(self):
        self.closed = True


class NewConnection:
    def __init__(self):
        self.closed = False
        self.writes = []

    def write(self, data):
        self.writes.append(data)

    def close(self):
        self.closed = True


class NanoTransportSerialTest(unittest.TestCase):
    def make_transport(self):
        transport = NanoTransport.__new__(NanoTransport)
        transport.connection = None
        transport.baudrate = 115200
        transport.last_telemetry = ""
        transport.last_terminal_controller_state = None
        return transport

    def test_connect_closes_existing_connection_before_opening_new_port(self):
        transport = self.make_transport()
        old_connection = ExistingConnection()
        new_connection = NewConnection()
        transport.connection = old_connection

        with patch("ball_vision.os.access", return_value=True), patch(
            "ball_vision.serial.Serial", return_value=new_connection
        ) as serial_factory, patch("ball_vision.time.sleep"):
            self.assertTrue(transport._connect("/tmp/mock-ttyUSB0"))

        self.assertTrue(old_connection.closed)
        self.assertEqual(old_connection.writes, [])
        self.assertIs(transport.connection, new_connection)
        serial_factory.assert_called_once_with(
            "/tmp/mock-ttyUSB0", baudrate=115200, timeout=0, write_timeout=0.2
        )
        self.assertEqual(new_connection.writes, [b"READY\n", b"PING\n"])

    def test_poll_returns_geometry_ack_event(self):
        class BufferedConnection:
            def __init__(self):
                self.lines = [b"GEOM,OK\n"]

            @property
            def in_waiting(self):
                return len(self.lines)

            def readline(self):
                return self.lines.pop(0)

        transport = self.make_transport()
        transport.connection = BufferedConnection()

        with redirect_stdout(io.StringIO()):
            self.assertEqual(transport.poll(), ["GEOM,OK"])

    def test_busy_connect_error_reports_holder_details(self):
        transport = self.make_transport()
        error = serial.SerialException("[Errno 16] Device or resource busy: '/dev/ttyUSB0'")

        output = io.StringIO()
        with patch("ball_vision.os.access", return_value=True), patch(
            "ball_vision.serial.Serial", side_effect=error
        ), patch(
            "ball_vision.serial_port_holder_details",
            return_value=["PID 123 fd 7: serial-monitor /dev/ttyUSB0 | parent 1: systemd"],
        ), redirect_stdout(output):
            self.assertFalse(transport._connect("/dev/ttyUSB0"))

        text = output.getvalue()
        self.assertIn("NANO CONNECT ERROR (/dev/ttyUSB0):", text)
        self.assertIn("NANO BUSY DIAGNOSTIC: /dev/ttyUSB0 is already open", text)
        self.assertIn("PID 123 fd 7: serial-monitor /dev/ttyUSB0", text)
        self.assertIsNone(transport.connection)


if __name__ == "__main__":
    unittest.main()
