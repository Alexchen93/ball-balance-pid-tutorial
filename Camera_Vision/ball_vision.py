#!/usr/bin/env python3
"""Interactive camera calibration, ball detection, and Nano serial transport."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import serial
from serial.tools import list_ports


WINDOW_NAME = "Ball vision: c=platform b=ball colour r=PID q=quit"


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as config_file:
        return json.load(config_file)


def save_config(path: Path, config: dict[str, Any]) -> None:
    path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


class NanoTransport:
    def __init__(self, port: str | None, baudrate: int, disabled: bool) -> None:
        self.connection: serial.Serial | None = None
        self.last_telemetry = "No serial"
        if disabled:
            return
        selected_port = port or self._find_arduino_port()
        if not selected_port:
            raise RuntimeError(
                "找不到 Arduino serial 埠。請接上 Nano 後指定 --serial-port /dev/ttyUSB0，"
                "或以 --no-serial 只測試鏡頭。"
            )
        self.connection = serial.Serial(selected_port, baudrate=baudrate, timeout=0, write_timeout=0.2)
        time.sleep(2.0)  # A classic Nano normally resets when USB serial opens.
        self.send("READY")
        self.send("PING")
        print(f"Nano connected: {selected_port} @ {baudrate}")

    @staticmethod
    def _find_arduino_port() -> str | None:
        likely_ports = []
        for port in list_ports.comports():
            description = f"{port.description} {port.manufacturer or ''}".lower()
            if any(marker in description for marker in ("arduino", "ch340", "cp210", "usb serial", "ftdi")):
                likely_ports.append(port.device)
        return likely_ports[0] if len(likely_ports) == 1 else None

    def send(self, line: str) -> None:
        if self.connection is None:
            return
        self.connection.write((line + "\n").encode("ascii"))

    def poll(self) -> None:
        if self.connection is None:
            return
        while self.connection.in_waiting:
            line = self.connection.readline().decode("ascii", errors="replace").strip()
            if line:
                self.last_telemetry = line

    def close(self) -> None:
        if self.connection is None:
            return
        try:
            self.send("READY")
        finally:
            self.connection.close()
            self.connection = None


class BallVision:
    def __init__(self, config_path: Path, config: dict[str, Any], transport: NanoTransport, headless: bool) -> None:
        self.config_path = config_path
        self.config = config
        self.transport = transport
        self.headless = headless
        self.mode = "normal"
        self.corners: list[tuple[float, float]] = [tuple(point) for point in config["platform"]["corners_px"]]
        self.homography: np.ndarray | None = None
        self.filtered_position: np.ndarray | None = None
        self.latest_frame: np.ndarray | None = None
        self.latest_hsv: np.ndarray | None = None
        self.pid_running = False
        self.last_sent_at = 0.0
        self.last_lost_at = 0.0
        self.last_frame_at = time.monotonic()
        self.fps = 0.0
        self._rebuild_homography()

    def _rebuild_homography(self) -> None:
        if len(self.corners) != 4:
            self.homography = None
            return
        width = float(self.config["platform"]["width_mm"])
        height = float(self.config["platform"]["height_mm"])
        source = np.float32(self.corners)
        destination = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
        self.homography = cv2.getPerspectiveTransform(source, destination)

    def _platform_position(self, point: tuple[float, float]) -> tuple[float, float] | None:
        if self.homography is None:
            return None
        mapped = cv2.perspectiveTransform(np.float32([[point]]), self.homography)[0][0]
        width = float(self.config["platform"]["width_mm"])
        height = float(self.config["platform"]["height_mm"])
        return float(mapped[0] - width / 2.0), float(height / 2.0 - mapped[1])

    def _ball_mask(self, hsv: np.ndarray) -> np.ndarray:
        settings = self.config["ball_hsv"]
        hue = int(settings["hue"])
        tolerance = int(settings["hue_tolerance"])
        saturation_min = int(settings["saturation_min"])
        value_min = int(settings["value_min"])
        lower_hue = hue - tolerance
        upper_hue = hue + tolerance
        if lower_hue >= 0 and upper_hue <= 179:
            mask = cv2.inRange(hsv, (lower_hue, saturation_min, value_min), (upper_hue, 255, 255))
        elif lower_hue < 0:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, (0, saturation_min, value_min), (upper_hue, 255, 255)),
                cv2.inRange(hsv, (180 + lower_hue, saturation_min, value_min), (179, 255, 255)),
            )
        else:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, (lower_hue, saturation_min, value_min), (179, 255, 255)),
                cv2.inRange(hsv, (0, saturation_min, value_min), (upper_hue - 180, 255, 255)),
            )
        kernel_size = max(1, int(self.config["detection"]["morph_kernel_px"]))
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)

    def _detect_ball(self, frame: np.ndarray) -> tuple[tuple[float, float], float] | None:
        self.latest_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._ball_mask(self.latest_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_radius = float(self.config["detection"]["min_radius_px"])
        max_radius = float(self.config["detection"]["max_radius_px"])
        candidates: list[tuple[float, tuple[float, float], float]] = []
        for contour in contours:
            (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
            if min_radius <= radius <= max_radius:
                candidates.append((cv2.contourArea(contour), (center_x, center_y), radius))
        if not candidates:
            return None
        _, center, radius = max(candidates, key=lambda item: item[0])
        return center, radius

    def _send_position(self, position: tuple[float, float] | None) -> None:
        now = time.monotonic()
        transport_settings = self.config["transport"]
        if position is None:
            interval = 1.0 / float(transport_settings["lost_heartbeat_hz"])
            if now - self.last_lost_at >= interval:
                self.transport.send("LOST")
                self.last_lost_at = now
            return
        interval = 1.0 / float(transport_settings["update_hz"])
        if now - self.last_sent_at >= interval:
            timestamp_ms = int(time.monotonic() * 1000)
            self.transport.send(f"POS,{position[0]:.2f},{position[1]:.2f},{timestamp_ms}")
            self.last_sent_at = now

    def _draw_overlay(self, frame: np.ndarray, ball: tuple[tuple[float, float], float] | None,
                      position: tuple[float, float] | None) -> np.ndarray:
        overlay = frame.copy()
        if self.corners:
            points = np.int32(self.corners)
            cv2.polylines(overlay, [points], len(self.corners) == 4, (255, 255, 0), 2)
            for index, point in enumerate(points):
                cv2.putText(overlay, str(index + 1), tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        if ball:
            center, radius = ball
            cv2.circle(overlay, (round(center[0]), round(center[1])), round(radius), (0, 255, 0), 2)
            cv2.circle(overlay, (round(center[0]), round(center[1])), 3, (0, 0, 255), -1)
        status = "RUN" if self.pid_running else "SAFE / READY"
        calibration = "CALIBRATED" if self.homography is not None else "PRESS c, CLICK 4 CORNERS"
        position_text = "BALL: LOST" if position is None else f"BALL: X={position[0]:+.1f} mm  Y={position[1]:+.1f} mm"
        lines = [f"PID: {status}", calibration, position_text, f"FPS: {self.fps:.1f}", self.transport.last_telemetry[:75]]
        for index, line in enumerate(lines):
            cv2.putText(overlay, line, (10, 25 + index * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(overlay, line, (10, 25 + index * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1)
        if self.mode != "normal":
            cv2.putText(overlay, f"MODE: {self.mode}", (10, overlay.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return overlay

    def _mouse_callback(self, event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.latest_frame is None:
            return
        if self.mode == "corners":
            self.corners.append((float(x), float(y)))
            if len(self.corners) == 4:
                self.config["platform"]["corners_px"] = self.corners
                self._rebuild_homography()
                save_config(self.config_path, self.config)
                self.mode = "normal"
                print("Platform calibration saved.")
            return
        if self.mode == "ball_colour" and self.latest_hsv is not None:
            patch = self.latest_hsv[max(0, y - 5): y + 6, max(0, x - 5): x + 6]
            if patch.size:
                hue, saturation, value = np.median(patch.reshape(-1, 3), axis=0).astype(int)
                settings = self.config["ball_hsv"]
                settings["hue"] = int(hue)
                settings["saturation_min"] = max(20, int(saturation) - 70)
                settings["value_min"] = max(20, int(value) - 70)
                save_config(self.config_path, self.config)
                self.mode = "normal"
                print(f"Ball colour saved: H={hue}, S>={settings['saturation_min']}, V>={settings['value_min']}")
            return
        position = self._platform_position((float(x), float(y)))
        if position is not None:
            self.transport.send(f"TARGET,{position[0]:.2f},{position[1]:.2f}")
            print(f"Target set to X={position[0]:+.1f}, Y={position[1]:+.1f} mm")

    def handle_key(self, key: int) -> bool:
        if key in (ord("q"), 27):
            return False
        if key == ord("c"):
            self.pid_running = False
            self.transport.send("READY")
            self.corners = []
            self.homography = None
            self.filtered_position = None
            self.mode = "corners"
            print("Click platform corners in order: top-left, top-right, bottom-right, bottom-left.")
        elif key == ord("b"):
            self.mode = "ball_colour"
            print("Click the ball to sample its HSV colour.")
        elif key == ord("r"):
            if self.homography is None:
                print("Cannot RUN: calibrate the four platform corners first.")
            else:
                self.pid_running = not self.pid_running
                self.transport.send("RUN" if self.pid_running else "READY")
                print("PID RUN" if self.pid_running else "PID READY")
        elif key == ord("0"):
            self.transport.send("TARGET,0.00,0.00")
            print("Target reset to platform centre.")
        return True

    def run(self, camera_index: int, width: int, height: int, requested_fps: int, max_frames: int) -> None:
        capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"無法開啟攝影機 index {camera_index}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, requested_fps)
        if not self.headless:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(WINDOW_NAME, self._mouse_callback)

        frame_count = 0
        try:
            while max_frames <= 0 or frame_count < max_frames:
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError("攝影機讀取失敗")
                now = time.monotonic()
                elapsed = max(now - self.last_frame_at, 0.001)
                self.fps = 0.9 * self.fps + 0.1 / elapsed
                self.last_frame_at = now
                self.latest_frame = frame
                ball = self._detect_ball(frame)
                position = self._platform_position(ball[0]) if ball else None
                if position is not None:
                    sample = np.array(position, dtype=np.float32)
                    alpha = float(self.config["detection"]["filter_alpha"])
                    self.filtered_position = sample if self.filtered_position is None else alpha * sample + (1 - alpha) * self.filtered_position
                    position = (float(self.filtered_position[0]), float(self.filtered_position[1]))
                else:
                    self.filtered_position = None
                self._send_position(position)
                self.transport.poll()
                if not self.headless:
                    cv2.imshow(WINDOW_NAME, self._draw_overlay(frame, ball, position))
                    if not self.handle_key(cv2.waitKey(1) & 0xFF):
                        break
                frame_count += 1
        finally:
            self.pid_running = False
            self.transport.send("READY")
            capture.release()
            if not self.headless:
                cv2.destroyAllWindows()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OpenCV ball detection for Arduino Nano ball balancing")
    default_config = Path(__file__).with_name("camera_config.json")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--camera-index", type=int)
    parser.add_argument("--serial-port", help="e.g. /dev/ttyUSB0 or /dev/ttyACM0")
    parser.add_argument("--no-serial", action="store_true", help="run vision only; never control Nano")
    parser.add_argument("--headless", action="store_true", help="no preview window; useful for camera diagnostics")
    parser.add_argument("--max-frames", type=int, default=0, help="stop after this many frames (0 = run until q)")
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    config = load_config(arguments.config)
    camera_settings = config["camera"]
    transport = NanoTransport(arguments.serial_port, int(config["transport"]["baudrate"]), arguments.no_serial)
    vision = BallVision(arguments.config, config, transport, arguments.headless)
    try:
        vision.run(
            arguments.camera_index if arguments.camera_index is not None else int(camera_settings["index"]),
            int(camera_settings["width"]),
            int(camera_settings["height"]),
            int(camera_settings["requested_fps"]),
            arguments.max_frames,
        )
        return 0
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    finally:
        transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
