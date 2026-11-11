import io
import os
import pty
import time
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
        transport._rx_buffer = bytearray()
        transport._rx_discarding_line = False
        return transport

    def test_pong_confirms_nano_response(self):
        transport = self.make_transport()
        connection = unittest.mock.MagicMock()
        connection.in_waiting = 5
        connection.read.return_value = b"PONG\n"
        transport.connection = connection
        output = io.StringIO()
        with redirect_stdout(output):
            transport.poll()
        self.assertIn("Nano replied to PING (PONG)", output.getvalue())

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
                return sum(map(len, self.lines))

            def read(self, size):
                data = b"".join(self.lines)
                self.lines = [data[size:]] if len(data) > size else []
                return data[:size]

        transport = self.make_transport()
        transport.connection = BufferedConnection()

        with redirect_stdout(io.StringIO()):
            self.assertEqual(transport.poll(), ["GEOM,OK"])

    def test_maze_target_is_parsed_without_affecting_run_events(self):
        class BufferedConnection:
            def __init__(self):
                self.lines = [b"TGT,12.5,-8.0,1\n", b"TGT,nan,9,1\n", b"TGT,1,2,invalid\n", b"GEOM,OK\n"]

            @property
            def in_waiting(self):
                return sum(map(len, self.lines))

            def read(self, size):
                data = b"".join(self.lines)
                self.lines = [data[size:]] if len(data) > size else []
                return data[:size]

        transport = self.make_transport()
        transport.connection = BufferedConnection()
        with redirect_stdout(io.StringIO()):
            events = transport.poll()
        self.assertEqual(events, ["GEOM,OK"])
        self.assertEqual(transport.joystick_target, (12.5, -8.0, True))

    def test_partial_serial_line_is_not_dispatched_until_newline(self):
        class ChunkConnection:
            def __init__(self):
                self.chunks = [b"STATE,REA"]

            @property
            def in_waiting(self):
                return len(self.chunks[0]) if self.chunks else 0

            def readline(self):
                return self.chunks.pop(0)

            def read(self, size):
                return self.chunks.pop(0)

        transport = self.make_transport()
        connection = ChunkConnection()
        transport.connection = connection
        with redirect_stdout(io.StringIO()):
            self.assertEqual(transport.poll(), [])
            connection.chunks.append(b"DY\nPOS,OK\n")
            self.assertEqual(transport.poll(), ["STATE,READY", "POS,OK"])

    def test_oversized_line_is_discarded_until_newline_and_reports_error(self):
        class ChunkConnection:
            def __init__(self):
                self.data = b"X" * 513

            @property
            def in_waiting(self):
                return len(self.data)

            def read(self, size):
                data, self.data = self.data[:size], self.data[size:]
                return data

        transport = self.make_transport()
        connection = ChunkConnection()
        transport.connection = connection
        with redirect_stdout(io.StringIO()):
            self.assertEqual(transport.poll(), ["ERROR,SERIAL_LINE_TOO_LONG"])
            connection.data = b"STATE,READY\nGEOM,OK\n"
            self.assertEqual(transport.poll(), ["GEOM,OK"])

    def test_discard_pending_input_also_discards_partial_line(self):
        class ChunkConnection:
            def __init__(self):
                self.data = b"STATE,REA"
            @property
            def in_waiting(self):
                return len(self.data)
            def read(self, size):
                data, self.data = self.data[:size], self.data[size:]
                return data
            def reset_input_buffer(self):
                self.data = b""

        transport = self.make_transport()
        connection = ChunkConnection()
        transport.connection = connection
        with redirect_stdout(io.StringIO()):
            self.assertEqual(transport.poll(), [])
            transport.discard_pending_input()
            connection.data = b"DY\nGEOM,OK\n"
            self.assertEqual(transport.poll(), ["GEOM,OK"])

    def test_pos_ack_is_forwarded_without_flooding_terminal(self):
        class ChunkConnection:
            in_waiting = 7
            def read(self, size):
                self.in_waiting = 0
                return b"POS,OK\n"
        transport = self.make_transport()
        transport.connection = ChunkConnection()
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(transport.poll(), ["POS,OK"])
        self.assertEqual(out.getvalue(), "")

    def test_nonblocking_pyserial_waits_for_complete_line_through_pty(self):
        master, slave = pty.openpty()
        transport = self.make_transport()
        try:
            transport.connection = serial.Serial(os.ttyname(slave), baudrate=115200, timeout=0)
            def send_and_wait(data):
                os.write(master, data)
                deadline = time.monotonic() + 1.0
                while not transport.connection.in_waiting and time.monotonic() < deadline:
                    time.sleep(0.001)
                self.assertGreater(transport.connection.in_waiting, 0)

            with redirect_stdout(io.StringIO()):
                send_and_wait(b"STATE,REA")
                self.assertEqual(transport.poll(), [])
                send_and_wait(b"DY\r\n")
                self.assertEqual(transport.poll(), ["STATE,READY"])
        finally:
            if transport.connection is not None:
                transport.close(send_ready=False)
            os.close(slave)
            os.close(master)

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
