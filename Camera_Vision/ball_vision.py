#!/usr/bin/env python3
"""Interactive camera calibration, ball detection, and Nano serial transport."""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import serial
from serial.tools import list_ports


WINDOW_NAME = "Ball vision: READY checklist | b=HSV c=platform d=zero r=RUN p=READY q=quit"


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as config_file:
        return json.load(config_file)


def save_config(path: Path, config: dict[str, Any]) -> None:
    path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


class NanoTransport:
    def __init__(self, port: str | None, baudrate: int, disabled: bool) -> None:
        self.connection: serial.Serial | None = None
        self.baudrate = baudrate
        self.last_telemetry = "No serial selected; READY setup only" if disabled else "Nano disconnected; press n to search."
        self.last_terminal_controller_state: str | None = None
        if disabled:
            print("No Nano selected. READY setup is active; press n to search for Nano later.")
        elif port:
            self._connect(port)
        else:
            self.search_and_connect()

    @property
    def is_connected(self) -> bool:
        return self.connection is not None

    @staticmethod
    def _candidate_ports() -> tuple[list[str], list[str], list[str]]:
        markers = ("arduino", "ch340", "cp210", "usb serial", "ftdi")
        metadata = {
            port.device: f"{port.description or ''} {port.manufacturer or ''}".lower()
            for port in list_ports.comports()
        }
        visible = set(metadata)
        visible.update(glob.glob("/dev/ttyUSB*"))
        visible.update(glob.glob("/dev/ttyACM*"))
        all_ports = sorted(
            device for device in visible if device.startswith(("/dev/ttyUSB", "/dev/ttyACM"))
        )
        blocked_ports = [device for device in all_ports if not os.access(device, os.R_OK | os.W_OK)]
        visible_ports = [device for device in all_ports if os.access(device, os.R_OK | os.W_OK)]
        likely_ports = [
            device for device in visible_ports if any(marker in metadata.get(device, "") for marker in markers)
        ]
        return visible_ports, likely_ports, blocked_ports

    def _connect(self, port: str) -> bool:
        if not os.access(port, os.R_OK | os.W_OK):
            self.connection = None
            self.last_telemetry = f"Nano port permission blocked: {port}"
            print(
                f"NANO CONNECT ERROR ({port}): current user cannot read/write this device. "
                "Check serial group membership or udev rules, then replug Nano."
            )
            return False
        try:
            connection = serial.Serial(port, baudrate=self.baudrate, timeout=0, write_timeout=0.2)
            self.connection = connection
            time.sleep(2.0)  # A classic Nano normally resets when USB serial opens.
            self.send("READY")
            self.send("PING")
            if self.connection is None:
                return False
            self.last_telemetry = f"Nano connected: {port}"
            print(f"Nano connected: {port} @ {self.baudrate}")
            return True
        except (OSError, serial.SerialException) as error:
            self.connection = None
            self.last_telemetry = f"Nano connect failed: {error}"
            print(f"NANO CONNECT ERROR ({port}): {error}")
            return False

    def search_and_connect(self) -> bool:
        if self.connection is not None:
            print("Nano is already connected.")
            return True
        visible_ports, likely_ports, blocked_ports = self._candidate_ports()
        if len(likely_ports) == 1:
            print(f"Searching Nano: using unique USB-serial candidate {likely_ports[0]}")
            return self._connect(likely_ports[0])
        if len(visible_ports) == 1:
            print(f"Searching Nano: using only usable USB-serial device {visible_ports[0]}")
            return self._connect(visible_ports[0])
        if not visible_ports:
            if blocked_ports:
                self.last_telemetry = "USB serial exists, but permission is blocked; press n after fixing it."
                print(
                    "NANO SEARCH: USB serial device exists but is not readable/writable by this user: "
                    + ", ".join(blocked_ports)
                )
                print("Fix serial group membership or udev rules, replug Nano, then press n to retry.")
            else:
                self.last_telemetry = "No USB serial; press n to retry."
                print("NANO SEARCH: no /dev/ttyUSB* or /dev/ttyACM* found. Plug Nano in, then press n to retry.")
        else:
            self.last_telemetry = "Multiple USB serial devices; press n after selecting one."
            print(f"NANO SEARCH: multiple usable candidates: {', '.join(visible_ports)}")
            if blocked_ports:
                print(f"Ignored permission-blocked devices: {', '.join(blocked_ports)}")
            print("Disconnect other USB serial devices, then press n to retry; use --serial-port only for manual override.")
        return False

    def send(self, line: str) -> bool:
        if self.connection is None:
            return False
        try:
            self.connection.write((line + "\n").encode("ascii"))
            return True
        except (OSError, serial.SerialException, serial.SerialTimeoutException) as error:
            self.last_telemetry = f"Nano link lost: {error}"
            print(f"NANO WRITE ERROR: {error}. Servo control is safely stopped; press n to reconnect.")
            self.close(send_ready=False)
            return False

    def discard_pending_input(self) -> None:
        if self.connection is None:
            return
        try:
            self.connection.reset_input_buffer()
        except (OSError, serial.SerialException) as error:
            self.last_telemetry = f"Nano link lost: {error}"
            print(f"NANO READ ERROR: {error}. Press n to reconnect.")
            self.close(send_ready=False)

    def poll(self) -> list[str]:
        events: list[str] = []
        if self.connection is None:
            return events
        try:
            while self.connection.in_waiting:
                line = self.connection.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                # TEL,state,link,ballX,ballY,errorX,errorY,outputX,outputY,servoX,servoY,ageMs
                parts = line.split(",")
                if len(parts) == 12 and parts[0] == "TEL":
                    self.last_telemetry = line
                    events.append(line)
                    # Keep setup quiet: print one status when READY changes, then
                    # stream live servo feedback only after the user presses r.
                    controller_state = parts[1]
                    if controller_state != "RUN":
                        events.append(f"STATE,{controller_state}")
                    if controller_state == "RUN" or controller_state != self.last_terminal_controller_state:
                        print(
                            "SERVO"
                            f" state={controller_state} link={parts[2]}"
                            f" ball=({parts[3]},{parts[4]})"
                            f" error=({parts[5]},{parts[6]})"
                            f" output=({parts[7]},{parts[8]})deg"
                            f" angle=X:{parts[9]}deg Y:{parts[10]}deg"
                            f" position_age={parts[11]}ms"
                        )
                    self.last_terminal_controller_state = controller_state
                else:
                    self.last_telemetry = line
                    if line.startswith(("ERROR,", "STATE,", "POS,OK")):
                        events.append(line)
                    print(f"NANO {line}")
        except (OSError, serial.SerialException) as error:
            self.last_telemetry = f"Nano link lost: {error}"
            print(f"NANO READ ERROR: {error}. Press n to reconnect.")
            self.close(send_ready=False)
        return events

    def close(self, send_ready: bool = True) -> None:
        if self.connection is None:
            return
        try:
            if send_ready:
                try:
                    self.connection.write(b"READY\n")
                except (OSError, serial.SerialException, serial.SerialTimeoutException):
                    pass
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
        self.corner_order_message: str | None = None
        self.filtered_position: np.ndarray | None = None
        stored_reference = config.get("control", {}).get("zero_reference_mm")
        self.zero_reference: np.ndarray | None = (
            np.array(stored_reference, dtype=np.float32)
            if isinstance(stored_reference, list) and len(stored_reference) == 2
            else None
        )
        self.latest_frame: np.ndarray | None = None
        self.latest_hsv: np.ndarray | None = None
        self.corner_labels = ("top-left", "top-right", "bottom-right", "bottom-left")
        self.pid_running = False
        self.run_start_phase: str | None = None
        self.run_start_deadline = 0.0
        self.run_start_ack_position: tuple[float, float] | None = None
        self.last_sent_at = 0.0
        self.last_lost_at = 0.0
        self.latest_position: tuple[float, float] | None = None
        self.last_frame_at = time.monotonic()
        self.fps = 0.0
        self._rebuild_homography()

    def _corner_order_problem(self) -> str | None:
        if len(self.corners) != 4:
            return None
        points = [(float(x), float(y)) for x, y in self.corners]
        if not all(math.isfinite(value) for point in points for value in point):
            return "Platform corner config has non-finite values. Press C to recalibrate TL, TR, BR, BL."
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        if span_x <= 1.0 or span_y <= 1.0:
            return "Platform corner config is degenerate. Press C to recalibrate TL, TR, BR, BL."
        center_x = (min(xs) + max(xs)) / 2.0
        center_y = (min(ys) + max(ys)) / 2.0
        tolerance_x = max(5.0, span_x * 0.08)
        tolerance_y = max(5.0, span_y * 0.08)

        top_left, top_right, bottom_right, bottom_left = points
        expected_order = (
            top_left[0] < center_x - tolerance_x
            and top_left[1] < center_y - tolerance_y
            and top_right[0] > center_x + tolerance_x
            and top_right[1] < center_y - tolerance_y
            and bottom_right[0] > center_x + tolerance_x
            and bottom_right[1] > center_y + tolerance_y
            and bottom_left[0] < center_x - tolerance_x
            and bottom_left[1] > center_y + tolerance_y
        )
        if expected_order:
            return None

        looks_tl_bl_br_tr = (
            top_left[0] < center_x - tolerance_x
            and top_left[1] < center_y - tolerance_y
            and top_right[0] < center_x - tolerance_x
            and top_right[1] > center_y + tolerance_y
            and bottom_right[0] > center_x + tolerance_x
            and bottom_right[1] > center_y + tolerance_y
            and bottom_left[0] > center_x + tolerance_x
            and bottom_left[1] < center_y - tolerance_y
        )
        if looks_tl_bl_br_tr:
            return (
                "Stored platform corners look like TL, BL, BR, TR; RUN is blocked. "
                "Press C to recalibrate TL, TR, BR, BL, or fix config with old indices [0,3,2,1]."
            )
        return "Stored platform corner order is ambiguous; RUN is blocked. Press C to recalibrate TL, TR, BR, BL."

    def _rebuild_homography(self) -> None:
        self.corner_order_message = None
        if len(self.corners) != 4:
            self.homography = None
            return
        self.corner_order_message = self._corner_order_problem()
        if self.corner_order_message is not None:
            self.homography = None
            print(self.corner_order_message)
            return
        width = float(self.config["platform"]["width_mm"])
        height = float(self.config["platform"]["height_mm"])
        source = np.float32(self.corners)
        destination = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
        self.homography = cv2.getPerspectiveTransform(source, destination)

    def _has_ball_hsv(self) -> bool:
        settings = self.config.get("ball_hsv", {})
        return all(key in settings for key in ("hue", "hue_tolerance", "saturation_min", "value_min"))

    def _has_platform(self) -> bool:
        return len(self.corners) == 4 and self.homography is not None and self.corner_order_message is None

    def _has_zero(self) -> bool:
        return self.zero_reference is not None

    def _ready_items(self) -> dict[str, bool]:
        return {
            "B ball HSV": self._has_ball_hsv(),
            "C platform border": self._has_platform(),
            "D zero point": self._has_zero(),
        }

    def _ready_complete(self) -> bool:
        return all(self._ready_items().values())

    def _ready_prompt(self) -> str:
        if self.mode == "ball_colour":
            return "B: click the ball centre to sample HSV only."
        if self.mode == "corners":
            next_index = min(len(self.corners), 3)
            return f"C: click platform {self.corner_labels[next_index]} corner."
        if self.mode == "zero":
            return "D: click the place where the ball should rest at X=0 Y=0."
        if self.corner_order_message is not None:
            return self.corner_order_message
        if self._ready_complete():
            return "READY complete. Press R to start RUN; P stays READY; B/C/D recalibrate."
        if not self._has_ball_hsv():
            return "Next: press B, then click the ball centre for HSV."
        if not self._has_platform():
            return "Next: press C, then click TL, TR, BR, BL platform corners."
        return "Next: press D, then click the desired balance zero point."

    def _print_ready_checklist(self) -> None:
        states = " | ".join(f"{'[x]' if done else '[ ]'} {name}" for name, done in self._ready_items().items())
        print(f"READY checklist: {states}")
        if self.corner_order_message is not None:
            print(self.corner_order_message)
        print(self._ready_prompt())

    def _save_and_report_ready(self) -> None:
        save_config(self.config_path, self.config)
        self._print_ready_checklist()

    def _apply_saved_config(self) -> None:
        self.pid_running = False
        self.run_start_phase = None
        self.transport.send("READY")
        self.config = load_config(self.config_path)
        self.corners = [tuple(point) for point in self.config["platform"].get("corners_px", [])]
        stored_reference = self.config.get("control", {}).get("zero_reference_mm")
        self.zero_reference = (
            np.array(stored_reference, dtype=np.float32)
            if isinstance(stored_reference, list) and len(stored_reference) == 2
            else None
        )
        self.filtered_position = None
        self.mode = "normal"
        self._rebuild_homography()
        print("Saved camera_config.json applied.")
        self._print_ready_checklist()

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

    def _send_position_command(self, position: tuple[float, float]) -> bool:
        timestamp_ms = int(time.monotonic() * 1000)
        sent = self.transport.send(f"POS,{position[0]:.2f},{position[1]:.2f},{timestamp_ms}")
        if sent:
            self.last_sent_at = time.monotonic()
        return sent

    def _send_position(self, position: tuple[float, float] | None) -> None:
        # Calibration may run while connected, but no live position commands are
        # sent until r explicitly starts PID control.
        if not self.pid_running:
            return
        now = time.monotonic()
        transport_settings = self.config["transport"]
        if position is None:
            interval = 1.0 / float(transport_settings["lost_heartbeat_hz"])
            if now - self.last_lost_at >= interval:
                self.transport.send("LOST")
                self.last_lost_at = now
            return
        self._send_position_command(position)

    def _safe_ready(self, message: str) -> None:
        was_active = self.pid_running or self.run_start_phase is not None
        self.pid_running = False
        self.run_start_phase = None
        self.run_start_ack_position = None
        self.filtered_position = None
        self.last_sent_at = 0.0
        self.last_lost_at = 0.0
        self.transport.send("READY")
        if was_active:
            print(message)
        else:
            print("Already READY. P keeps Nano in READY; no POS/LOST/servo commands are being sent.")

    def _request_run(self, position: tuple[float, float]) -> None:
        self.transport.discard_pending_input()
        if not self._send_position_command(position):
            print("Cannot RUN: failed to send fresh POS to Nano. Press n to reconnect, then R again.")
            return
        self.run_start_phase = "wait_pos_ack"
        self.run_start_ack_position = position
        self.run_start_deadline = time.monotonic() + 0.6
        print("RUN requested: sent fresh POS for this frame; POS,OK or matching TEL,READY,OK will trigger a final POS immediately followed by RUN.")

    def _is_implicit_pos_ack(self, nano_event: str) -> bool:
        if self.run_start_ack_position is None:
            return False
        parts = nano_event.split(",")
        if len(parts) != 12 or parts[:3] != ["TEL", "READY", "OK"]:
            return False
        try:
            telemetry_x = float(parts[3])
            telemetry_y = float(parts[4])
            age_ms = float(parts[11])
        except ValueError:
            return False
        if not all(math.isfinite(value) for value in (telemetry_x, telemetry_y, age_ms)):
            return False
        sent_x, sent_y = self.run_start_ack_position
        return age_ms <= 500.0 and abs(telemetry_x - sent_x) <= 2.0 and abs(telemetry_y - sent_y) <= 2.0

    def _send_final_pos_and_run(self, ack_label: str) -> None:
        position = self.latest_position
        if position is None:
            self._safe_ready("RUN cancelled: ball was lost before Nano ACK. Keep it visible, then press R again.")
            return
        if not self._send_position_command(position):
            self._safe_ready("RUN cancelled: failed to send final fresh POS immediately before RUN. Press n to reconnect, then R again.")
            return
        self.transport.send("RUN")
        self.run_start_phase = "wait_run_ack"
        self.run_start_ack_position = None
        self.run_start_deadline = time.monotonic() + 0.6
        print(f"Nano accepted initial POS via {ack_label}; resent latest fresh POS immediately followed by RUN. Waiting for STATE,RUN.")

    def _handle_nano_events(self) -> None:
        for nano_event in self.transport.poll():
            if self.run_start_phase == "wait_pos_ack":
                if nano_event.startswith("POS,OK"):
                    self._send_final_pos_and_run("POS,OK")
                    continue
                if self._is_implicit_pos_ack(nano_event):
                    self._send_final_pos_and_run("TEL,READY,OK compatibility fallback")
                    continue
                if nano_event.startswith("ERROR,"):
                    self._safe_ready(f"RUN cancelled by Nano ({nano_event}). Back to READY.")
                    continue
            if self.run_start_phase == "wait_run_ack":
                if nano_event == "STATE,RUN":
                    self.pid_running = True
                    self.run_start_phase = None
                    self.last_sent_at = 0.0
                    self.last_lost_at = 0.0
                    print("RUN confirmed by Nano. Python is now sending fresh POS every camera frame.")
                    continue
                if nano_event.startswith("ERROR,"):
                    self._safe_ready(f"RUN rejected by Nano ({nano_event}). Back to READY.")
                    continue
            if self.pid_running and (nano_event.startswith("ERROR,") or nano_event == "STATE,READY"):
                self._safe_ready(f"RUN stopped by Nano ({nano_event}). Back to READY; fix the reported condition before pressing R again.")
                break
        if self.run_start_phase is not None and time.monotonic() > self.run_start_deadline:
            phase = self.run_start_phase
            self.run_start_ack_position = None
            self._safe_ready(f"RUN handshake timed out while waiting for Nano {phase}; Back to READY. Check serial RX/TX and Nano firmware protocol before pressing R again.")

    def _draw_overlay(self, frame: np.ndarray, ball: tuple[tuple[float, float], float] | None,
                      position: tuple[float, float] | None) -> np.ndarray:
        overlay = frame.copy()
        if self.corners:
            points = np.int32(self.corners)
            cv2.polylines(overlay, [points], len(self.corners) == 4, (255, 255, 0), 2)
            for index, point in enumerate(points):
                label = str(index + 1)
                cv2.putText(overlay, label, tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        if ball:
            center, radius = ball
            cv2.circle(overlay, (round(center[0]), round(center[1])), round(radius), (0, 255, 0), 2)
            cv2.circle(overlay, (round(center[0]), round(center[1])), 3, (0, 0, 255), -1)

        if self.pid_running:
            run_state = "RUN"
        elif self.run_start_phase is not None:
            run_state = "RUN START: waiting for Nano ACK"
        else:
            run_state = "READY COMPLETE" if self._ready_complete() else "READY SETUP"
        nano_status = "NANO: CONNECTED" if self.transport.is_connected else "NANO: DISCONNECTED (press n to search)"
        items = self._ready_items()
        checklist = "  ".join(f"{'[x]' if done else '[ ]'} {name}" for name, done in items.items())
        if self.pid_running:
            position_text = "BALL: LOST" if position is None else f"BALL: X={position[0]:+.1f} mm  Y={position[1]:+.1f} mm"
        else:
            position_text = "READY: P=hold READY; no POS/LOST/servo commands are sent"
        lines = [
            f"STATE: {run_state}",
            nano_status,
            checklist,
            self._ready_prompt(),
            position_text,
            f"FPS: {self.fps:.1f}",
        ]
        if self.corner_order_message is not None:
            lines.insert(4, self.corner_order_message[:95])
        if self.pid_running:
            lines.append(self.transport.last_telemetry[:75])
        for index, line in enumerate(lines):
            y = 25 + index * 25
            cv2.putText(overlay, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(overlay, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1)
        if self.mode != "normal":
            cv2.putText(overlay, f"ACTIVE STEP: {self.mode}", (10, overlay.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return overlay

    def _mouse_callback(self, event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.latest_frame is None:
            return
        if self.mode == "ball_colour" and self.latest_hsv is not None:
            if 0 <= y < self.latest_hsv.shape[0] and 0 <= x < self.latest_hsv.shape[1]:
                hue, saturation, value = (int(component) for component in self.latest_hsv[y, x])
                settings = self.config["ball_hsv"]
                settings["hue"] = hue
                settings["saturation_min"] = max(20, saturation - 70)
                settings["value_min"] = max(20, value - 70)
                self.mode = "normal"
                print(
                    f"B saved centre-pixel HSV only: H={hue}, "
                    f"S>={settings['saturation_min']}, V>={settings['value_min']}."
                )
                self._save_and_report_ready()
            return

        if self.mode == "corners":
            self.corners.append((float(x), float(y)))
            clicked = self.corner_labels[len(self.corners) - 1]
            print(f"C captured {clicked} corner ({x}, {y}).")
            if len(self.corners) == 4:
                self.config["platform"]["corners_px"] = self.corners
                self._rebuild_homography()
                self.filtered_position = None
                self.zero_reference = None
                self.config.setdefault("control", {}).pop("zero_reference_mm", None)
                self.mode = "normal"
                print("C saved platform border. D zero point must be selected for this border.")
                self._save_and_report_ready()
            else:
                print(f"C next: click {self.corner_labels[len(self.corners)]} corner.")
            return

        if self.mode == "zero":
            position = self._platform_position((float(x), float(y)))
            if position is None:
                print("D needs platform border first: press C and click all four corners.")
                return
            self.zero_reference = np.array(position, dtype=np.float32)
            self.config.setdefault("control", {})["zero_reference_mm"] = [
                round(float(position[0]), 2),
                round(float(position[1]), 2),
            ]
            self.filtered_position = None
            self.mode = "normal"
            print(f"D saved zero point: raw platform X={position[0]:+.1f}, Y={position[1]:+.1f} mm becomes X=0, Y=0.")
            self._save_and_report_ready()
            return

        print("READY: press B for HSV, C for platform border, D for zero point, or R to RUN when complete.")

    def handle_key(self, key: int) -> bool:
        if key in (ord("q"), 27):
            return False
        if key == ord("n"):
            self._safe_ready("Stopped for Nano search. Back to READY; no live position or servo commands are being sent.")
            self.mode = "normal"
            self.transport.search_and_connect()
        elif key == ord("a"):
            self._apply_saved_config()
        elif key == ord("b"):
            self._safe_ready("B selected. Back to READY; no live position or servo commands are being sent.")
            self.mode = "ball_colour"
            print("B: click the centre of the ball. Only HSV colour will be saved.")
        elif key == ord("c"):
            self._safe_ready("C selected. Back to READY; no live position or servo commands are being sent.")
            self.corners = []
            self.homography = None
            self.filtered_position = None
            self.zero_reference = None
            self.config.setdefault("control", {}).pop("zero_reference_mm", None)
            self.mode = "corners"
            print("C: click platform corners in order: top-left, top-right, bottom-right, bottom-left.")
        elif key == ord("d"):
            self._safe_ready("D selected. Back to READY; no live position or servo commands are being sent.")
            if self.homography is None:
                print("D needs platform border first: press C and click all four corners.")
            else:
                self.mode = "zero"
                print("D: click the place where the ball should rest. That point becomes X=0, Y=0.")
        elif key in (ord("p"), ord("P")):
            self._safe_ready("P pressed. Back to READY; sent READY and stopped live POS/LOST/servo control.")
        elif key in (ord("r"), ord("R")):
            if self.pid_running:
                self._safe_ready("R pressed during RUN. Back to READY; no live position or servo commands are being sent.")
            elif self.run_start_phase is not None:
                print("RUN is already waiting for Nano ACK. Press P to cancel safely back to READY.")
            elif self.mode != "normal":
                print("Cannot RUN: finish the active READY step first, or press A to apply saved config.")
            elif not self.transport.is_connected:
                print("Cannot RUN: Nano is not connected. Press n to search.")
            elif not self._ready_complete():
                print("Cannot RUN: READY checklist is incomplete.")
                self._print_ready_checklist()
            elif self.latest_position is None:
                print("Cannot RUN: ball is not detected yet. Keep it visible, then press R again.")
            else:
                self._request_run(self.latest_position)
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
            print("GUIDE: READY checklist uses A=apply saved config, B=HSV, C=platform TL/TR/BR/BL, D=zero point, R=RUN, P=READY/pause, Q=quit.")
            self._print_ready_checklist()

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
                if position is not None and self.zero_reference is not None:
                    position = (
                        position[0] - float(self.zero_reference[0]),
                        position[1] - float(self.zero_reference[1]),
                    )
                if position is not None:
                    sample = np.array(position, dtype=np.float32)
                    alpha = float(self.config["detection"]["filter_alpha"])
                    self.filtered_position = sample if self.filtered_position is None else alpha * sample + (1 - alpha) * self.filtered_position
                    position = (float(self.filtered_position[0]), float(self.filtered_position[1]))
                else:
                    self.filtered_position = None
                self.latest_position = position
                self._send_position(position)
                if self.pid_running or self.run_start_phase is not None:
                    self._handle_nano_events()
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
    parser.add_argument("--no-serial", action="store_true", help="open READY setup without a Nano connection")
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
