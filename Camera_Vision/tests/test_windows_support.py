import unittest
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from ball_vision import BallVision, NanoTransport


class WindowsSupportTest(unittest.TestCase):
    def test_windows_launcher_and_program_use_ascii_text(self):
        folder = Path(__file__).resolve().parents[1]
        for name in ("start_camera_vision.bat", "install_python_windows.ps1", "windows_launcher.py", "ball_vision.py"):
            with self.subTest(name=name):
                (folder / name).read_bytes().decode("ascii")

    def test_python_runtime_is_checked_before_extraction(self):
        folder = Path(__file__).resolve().parents[1]
        installer = (folder / "install_python_windows.ps1").read_text(encoding="ascii")
        expected = "b26ca4b4ee51e157836049934d2b8eabd55939be296f3130254251b6df96ec2f"
        self.assertIn("https://www.python.org/ftp/python/3.13.17/python-3.13.17-amd64.zip", installer)
        self.assertIn(expected, installer)
        self.assertLess(installer.index("Get-FileHash"), installer.index("Expand-Archive"))
        self.assertIn("$actualSha256 -ne $expectedSha256", installer)

    def test_runtime_checks_avoid_nested_windows_command_quoting(self):
        installer = (Path(__file__).resolve().parents[1] / "install_python_windows.ps1").read_text(encoding="ascii")
        self.assertIn("& $python -m ensurepip --version", installer)
        self.assertIn("& $python -m venv --help", installer)
        self.assertNotIn("& $python -c", installer)
        for args in (("-m", "ensurepip", "--version"), ("-m", "venv", "--help")):
            with self.subTest(args=args):
                result = subprocess.run([sys.executable, *args], capture_output=True, text=True, check=False)
                self.assertEqual(result.returncode, 0, result.stderr)

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

    def test_ready_loop_reads_nano_responses(self):
        vision = BallVision.__new__(BallVision)
        vision.headless = True
        vision.last_frame_at = 0.0
        vision.fps = 0.0
        vision.pid_running = False
        vision.run_start_phase = None
        vision.zero_reference = None
        vision.transport = MagicMock()
        vision.transport.poll.return_value = []
        vision._detect_ball = MagicMock(return_value=None)
        capture = MagicMock()
        capture.isOpened.return_value = True
        capture.read.return_value = (True, np.zeros((8, 8, 3), dtype=np.uint8))
        with patch("ball_vision.cv2.VideoCapture", return_value=capture):
            vision.run(0, 8, 8, 30, 1)
        vision.transport.poll.assert_called_once()
        capture.release.assert_called_once()


if __name__ == "__main__":
    unittest.main()
