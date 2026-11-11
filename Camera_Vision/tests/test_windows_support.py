import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import cv2

from ball_vision import BallVision, NanoTransport


class WindowsSupportTest(unittest.TestCase):
    def test_windows_launcher_and_program_use_ascii_text(self):
        folder = Path(__file__).resolve().parents[1]
        for name in ("start_camera_vision.bat", "ball_vision.py"):
            with self.subTest(name=name):
                (folder / name).read_bytes().decode("ascii")

    def test_search_lists_com_ports_without_filesystem_permission_check(self):
        ports = [
            SimpleNamespace(device="COM3", description="USB-SERIAL CH340", manufacturer=""),
            SimpleNamespace(device="COM4", description="Other", manufacturer=""),
        ]
        with patch("ball_vision.sys.platform", "win32"), patch(
            "ball_vision.list_ports.comports", return_value=ports
        ), patch("ball_vision.os.access", side_effect=AssertionError("COM is not a filesystem path")):
            self.assertEqual(NanoTransport._candidate_ports(), (["COM3", "COM4"], ["COM3"], []))

    def test_connect_opens_com_port_and_sends_ready(self):
        transport = NanoTransport.__new__(NanoTransport)
        transport.connection = None
        transport.baudrate = 115200
        transport._rx_buffer = bytearray()
        transport._rx_discarding_line = False
        connection = MagicMock()
        with patch("ball_vision.sys.platform", "win32"), patch(
            "ball_vision.serial.Serial", return_value=connection
        ) as open_port, patch("ball_vision.time.sleep"), patch(
            "ball_vision.os.access", side_effect=AssertionError("COM is not a filesystem path")
        ):
            self.assertTrue(transport._connect("COM3"))
        open_port.assert_called_once_with("COM3", baudrate=115200, timeout=0, write_timeout=0.2)
        self.assertEqual([call.args[0] for call in connection.write.call_args_list], [b"READY\n", b"PING\n"])

    def test_windows_camera_uses_directshow(self):
        capture = MagicMock()
        capture.isOpened.return_value = False
        with patch("ball_vision.sys.platform", "win32"), patch(
            "ball_vision.cv2.VideoCapture", return_value=capture
        ) as open_camera:
            with self.assertRaises(RuntimeError):
                BallVision.__new__(BallVision).run(1, 640, 480, 30, 1)
        open_camera.assert_called_once_with(1, cv2.CAP_DSHOW)


if __name__ == "__main__":
    unittest.main()
