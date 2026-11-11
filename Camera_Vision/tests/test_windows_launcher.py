import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import numpy as np

import windows_launcher as launcher


class WindowsLauncherTest(unittest.TestCase):
    def test_probe_releases_every_camera_and_lists_readable_indexes(self):
        captures = []

        def open_camera(index, backend):
            capture = MagicMock()
            capture.isOpened.return_value = index in (0, 2)
            capture.read.return_value = (True, np.zeros((2, 2, 3), dtype=np.uint8))
            captures.append(capture)
            return capture

        with patch.object(launcher, "CAMERA_SCAN_LIMIT", 4), patch.object(
            launcher.cv2, "VideoCapture", side_effect=open_camera
        ):
            self.assertEqual(launcher.find_cameras(), [0, 2])
        self.assertEqual(len(captures), 4)
        for capture in captures:
            capture.release.assert_called_once()

    def test_selects_correct_index_and_requires_preview_confirmation(self):
        with patch.object(launcher, "find_cameras", return_value=[0, 2]), patch.object(
            launcher, "preview_camera", side_effect=[False, True]
        ) as preview, patch("builtins.input", side_effect=["1", "2"]), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(launcher.choose_camera(), 2)
        self.assertEqual(preview.call_args_list, [call(0), call(2)])

    def test_preview_accepts_y_and_releases_camera(self):
        capture = MagicMock()
        capture.isOpened.return_value = True
        capture.read.return_value = (True, np.zeros((40, 100, 3), dtype=np.uint8))
        with patch.object(launcher.cv2, "VideoCapture", return_value=capture), patch.object(
            launcher.cv2, "namedWindow"
        ), patch.object(launcher.cv2, "imshow"), patch.object(launcher.cv2, "destroyWindow"), patch.object(
            launcher.cv2, "waitKey", return_value=ord("y")
        ), contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(launcher.preview_camera(1))
        capture.release.assert_called_once()

    def test_search_retries_and_returns_selected_port(self):
        ports = [SimpleNamespace(device="COM7", description="USB-SERIAL CH340", manufacturer="WCH")]
        with patch.object(launcher.list_ports, "comports", side_effect=[[], ports]), patch(
            "builtins.input", side_effect=["1", "r", "1"]
        ), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(launcher.choose_port(), (True, "COM7"))

    def test_ready_without_nano_does_not_scan_serial_ports(self):
        with patch.object(launcher.list_ports, "comports") as scan, patch(
            "builtins.input", return_value="2"
        ), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(launcher.choose_port(), (True, None))
        scan.assert_not_called()

    def test_launcher_passes_selected_camera_and_port_to_program(self):
        with patch.object(launcher, "choose_camera", return_value=2), patch.object(
            launcher, "choose_port", return_value=(True, "COM7")
        ), patch.object(launcher.subprocess, "run", return_value=SimpleNamespace(returncode=0)) as run:
            self.assertEqual(launcher.main(), 0)
        command = run.call_args.args[0]
        self.assertEqual(command[-4:], ["--camera-index", "2", "--serial-port", "COM7"])


if __name__ == "__main__":
    unittest.main()
