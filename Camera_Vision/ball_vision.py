#!/usr/bin/env python3
"""Interactive camera calibration, ball detection, and Nano serial transport."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import errno
from functools import lru_cache
import glob
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import serial
from serial.tools import list_ports


PLATFORM_HALF_SPAN_PCT = 100.0
PLATFORM_FULL_SPAN_PCT = PLATFORM_HALF_SPAN_PCT * 2.0
MAX_NANO_LINE_BYTES = 512
MAX_NANO_READ_BYTES = 4096
MISSING_BALL_GRACE_SECONDS = 0.08
WINDOW_NAME = "平衡球影像辨識"


@lru_cache(maxsize=1)
def _status_sprites() -> np.ndarray:
    """Load pre-rendered Traditional Chinese labels without a font dependency."""
    path = Path(__file__).with_name("status_zh_tw.png")
    data = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    sprites = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if sprites is None or sprites.ndim != 3 or sprites.shape[2] != 4 or sprites.shape[0] != 256:
        raise RuntimeError("無法載入中文狀態圖示，請確認 status_zh_tw.png 存在且未損壞。")
    return sprites


def _draw_status(overlay: np.ndarray, state_index: int, connected: bool) -> None:
    sprites = _status_sprites()
    index = state_index * 2 + int(connected)
    sprite = sprites[index * 32:(index + 1) * 32]
    height = min(sprite.shape[0], overlay.shape[0])
    width = min(sprite.shape[1], overlay.shape[1])
    alpha = sprite[:height, :width, 3:4].astype(np.float32) / 255.0
    target = overlay[:height, :width]
    target[:] = (sprite[:height, :width, :3] * alpha + target * (1 - alpha)).astype(np.uint8)


@dataclass(frozen=True)
class PlatformGeometry:
    x_min_pct: float
    x_max_pct: float
    y_min_pct: float
    y_max_pct: float

    def as_command(self) -> str:
        return (
            "GEOM,"
            f"{self.x_min_pct:.2f},{self.x_max_pct:.2f},"
            f"{self.y_min_pct:.2f},{self.y_max_pct:.2f}"
        )


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as config_file:
        config = json.load(config_file)
    if "pid" in config:
        config.pop("pid", None)
        print("舊版設定：已忽略 camera_config.json 中的 pid；PID 參數由 Nano 韌體管理。")
    control = config.setdefault("control", {})
    if "zero_reference_mm" in control and "zero_reference_pct" not in control:
        control.pop("zero_reference_mm", None)
        print("舊版設定：已忽略 zero_reference_mm；零點固定為校正後的平台中心。")
    return config


def save_config(path: Path, config: dict[str, Any]) -> None:
    path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


CORNER_LABELS_TLTRBRBL = ("TL", "TR", "BR", "BL")


def _polygon_area(points: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    )


def _corner_cross(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]
) -> float:
    return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])


def normalize_platform_corners(
    corners: list[tuple[float, float]] | tuple[tuple[float, float], ...],
) -> tuple[list[tuple[float, float]] | None, str | None]:
    """Validate and return platform corners only in TL, TR, BR, BL order."""
    if len(corners) != 4:
        return None, "平台校正必須有四個角點，請按 C 重新校正。"
    try:
        points = [(float(x), float(y)) for x, y in corners]
    except (TypeError, ValueError):
        return None, "平台角點座標不是數字，請按 C 重新校正。"
    if not all(math.isfinite(value) for point in points for value in point):
        return None, "平台角點座標不是有限數值，請按 C 重新校正。"
    for index, point in enumerate(points):
        for other in points[index + 1:]:
            distance_sq = (point[0] - other[0]) ** 2 + (point[1] - other[1]) ** 2
            if distance_sq <= 1.0:
                return None, "平台角點重複或重疊，請按 C 重試。"

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    if span_x <= 5.0 or span_y <= 5.0:
        return None, "平台角點形狀退化或範圍太小，請按 C 重試。"

    center_x = sum(xs) / 4.0
    center_y = sum(ys) / 4.0
    ordered = sorted(points, key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x))
    area = _polygon_area(ordered)
    min_area = max(25.0, span_x * span_y * 0.05)
    if abs(area) < min_area:
        return None, "平台角點形狀退化，面積太小，請按 C 重試。"
    if area < 0:
        ordered.reverse()

    crosses = [
        _corner_cross(ordered[index - 1], ordered[index], ordered[(index + 1) % 4])
        for index in range(4)
    ]
    min_cross = max(10.0, span_x * span_y * 0.01)
    if any(abs(cross) < min_cross for cross in crosses) or not all(cross > 0 for cross in crosses):
        return None, "平台角點形成凹形、交叉或接近共線，請按 C 重試。"

    tolerance_x = max(3.0, span_x * 0.02)
    tolerance_y = max(3.0, span_y * 0.02)
    labels_by_point: dict[tuple[float, float], str] = {}
    for point in ordered:
        x, y = point
        if x < center_x - tolerance_x and y < center_y - tolerance_y:
            label = "TL"
        elif x > center_x + tolerance_x and y < center_y - tolerance_y:
            label = "TR"
        elif x > center_x + tolerance_x and y > center_y + tolerance_y:
            label = "BR"
        elif x < center_x - tolerance_x and y > center_y + tolerance_y:
            label = "BL"
        else:
            return None, "無法明確判斷平台角點位置，請按 C 重試。"
        if label in labels_by_point.values():
            return None, "無法分別判斷左上、右上、右下與左下角點，請按 C 重試。"
        labels_by_point[point] = label
    if set(labels_by_point.values()) != set(CORNER_LABELS_TLTRBRBL):
        return None, "無法分別判斷左上、右上、右下與左下角點，請按 C 重試。"

    clicked_labels = [labels_by_point[point] for point in points]
    if tuple(clicked_labels) != CORNER_LABELS_TLTRBRBL:
        mismatches = [
            f"第 {index + 1} 點為 {actual}，應為 {expected}"
            for index, (actual, expected) in enumerate(zip(clicked_labels, CORNER_LABELS_TLTRBRBL))
            if actual != expected
        ]
        return (
            None,
            "平台角點須依左上、右上、右下、左下順序點選。"
            + "; ".join(mismatches)
            + "。請按 C 重試。",
        )
    return points, None


def _read_proc_text(path: Path) -> str:
    try:
        return path.read_text(errors="replace").strip()
    except OSError:
        return ""


def _proc_command(pid: str) -> str:
    cmdline_path = Path("/proc") / pid / "cmdline"
    try:
        raw = cmdline_path.read_bytes().replace(b"\0", b" ").strip()
    except OSError:
        raw = b""
    if raw:
        return raw.decode(errors="replace")
    return _read_proc_text(Path("/proc") / pid / "comm") or "?"


def _proc_parent_pid(pid: str) -> str:
    for line in _read_proc_text(Path("/proc") / pid / "status").splitlines():
        if line.startswith("PPid:"):
            fields = line.split(None, 1)
            return fields[1] if len(fields) == 2 else "?"
    return "?"


def serial_port_holder_details(port: str) -> list[str]:
    if not sys.platform.startswith("linux"):
        return []
    try:
        target = os.path.realpath(port)
    except OSError:
        target = port
    holders: list[tuple[int, str]] = []
    for proc_entry in Path("/proc").iterdir():
        if not proc_entry.name.isdigit():
            continue
        fd_dir = proc_entry / "fd"
        try:
            fd_entries = list(fd_dir.iterdir())
        except OSError:
            continue
        for fd_entry in fd_entries:
            try:
                linked = os.path.realpath(fd_entry)
            except OSError:
                continue
            if linked == target:
                holders.append((int(proc_entry.name), fd_entry.name))
                break

    details: list[str] = []
    for pid, fd in sorted(holders):
        pid_text = str(pid)
        parent_pid = _proc_parent_pid(pid_text)
        parent_command = _proc_command(parent_pid) if parent_pid.isdigit() else "?"
        details.append(
            f"PID {pid_text} fd {fd}: {_proc_command(pid_text)} | parent {parent_pid}: {parent_command}"
        )
    return details


def is_serial_busy_error(error: BaseException) -> bool:
    err_no = getattr(error, "errno", None)
    if err_no == errno.EBUSY:
        return True
    message = str(error)
    return "Errno 16" in message or "EBUSY" in message or "Device or resource busy" in message


def print_serial_busy_diagnostics(port: str) -> None:
    print(f"Nano 埠忙碌：{port} 已被其他程式開啟。")
    holders = serial_port_holder_details(port)
    if holders:
        for holder in holders:
            print(f"占用埠的程式：{holder}")
    else:
        print(
            "無法確認目前占用埠的程式。"
        )


class NanoTransport:
    def __init__(self, port: str | None, baudrate: int, disabled: bool) -> None:
        self.connection: serial.Serial | None = None
        self.baudrate = baudrate
        self.last_telemetry = "未選擇序列埠；可進行 READY 設定。" if disabled else "Nano 未連線；按 n 搜尋。"
        self.last_terminal_controller_state: str | None = None
        self.joystick_target: tuple[float, float, bool] | None = None
        self.last_joystick_armed: bool | None = None
        self._rx_buffer = bytearray()
        self._rx_discarding_line = False
        if disabled:
            print("未選擇 Nano，目前可進行 READY 設定；稍後可按 n 搜尋。")
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
        if sys.platform == "win32":
            # Windows COM devices are not filesystem paths; opening the port is
            # the reliable way to detect access errors or an occupied port.
            visible_ports = sorted(metadata)
            likely_ports = [
                device for device in visible_ports if any(marker in metadata[device] for marker in markers)
            ]
            return visible_ports, likely_ports, []
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
        if self.connection is not None:
            self.close(send_ready=False)
        if sys.platform != "win32" and not os.access(port, os.R_OK | os.W_OK):
            self.connection = None
            self.last_telemetry = f"沒有 Nano 埠讀寫權限：{port}"
            print(
                f"Nano 連線錯誤（{port}）：目前使用者沒有讀寫權限。"
                "請檢查序列埠群組或 udev 規則，再重新插入 Nano。"
            )
            return False
        try:
            connection = serial.Serial(port, baudrate=self.baudrate, timeout=0, write_timeout=0.2)
            self.connection = connection
            self._rx_buffer.clear()
            self._rx_discarding_line = False
            time.sleep(2.0)  # A classic Nano normally resets when USB serial opens.
            self.send("READY")
            self.send("PING")
            if self.connection is None:
                return False
            self.last_telemetry = f"Nano 已連線：{port}"
            print(f"Nano 已連線：{port}，鮑率 {self.baudrate}。")
            return True
        except (OSError, serial.SerialException) as error:
            self.connection = None
            self.last_telemetry = f"Nano 連線失敗：{error}"
            print(f"Nano 連線錯誤（{port}）：{error}")
            if is_serial_busy_error(error):
                print_serial_busy_diagnostics(port)
            return False

    def search_and_connect(self) -> bool:
        if self.connection is not None:
            print("Nano 已經連線。")
            return True
        visible_ports, likely_ports, blocked_ports = self._candidate_ports()
        if len(likely_ports) == 1:
            print(f"搜尋 Nano：使用唯一符合的 USB 序列埠 {likely_ports[0]}。")
            return self._connect(likely_ports[0])
        if len(visible_ports) == 1:
            print(f"搜尋 Nano：使用唯一可用的 USB 序列埠 {visible_ports[0]}。")
            return self._connect(visible_ports[0])
        if not visible_ports:
            if blocked_ports:
                self.last_telemetry = "找到 USB 序列埠，但沒有讀寫權限；修正後按 n。"
                print(
                    "搜尋 Nano：目前使用者無法讀寫以下 USB 序列埠："
                    + ", ".join(blocked_ports)
                )
                print("請修正序列埠群組或 udev 規則，重新插入 Nano，再按 n 重試。")
            else:
                self.last_telemetry = "找不到 USB 序列埠；按 n 重試。"
                print("搜尋 Nano：找不到序列埠。請插入 Nano，再按 n 重試。")
        else:
            self.last_telemetry = "找到多個 USB 序列埠；選定後按 n。"
            print(f"搜尋 Nano：找到多個可用序列埠：{', '.join(visible_ports)}")
            if blocked_ports:
                print(f"已略過沒有權限的裝置：{', '.join(blocked_ports)}")
            print("請拔除其他 USB 序列裝置，再按 n 重試；也可用 --serial-port 手動指定。")
        return False

    def send(self, line: str) -> bool:
        if self.connection is None:
            return False
        try:
            self.connection.write((line + "\n").encode("ascii"))
            return True
        except (OSError, serial.SerialException, serial.SerialTimeoutException) as error:
            self.last_telemetry = f"Nano 連線中斷：{error}"
            print(f"Nano 寫入錯誤：{error}。伺服控制已安全停止；按 n 重新連線。")
            self.close(send_ready=False)
            return False

    def discard_pending_input(self) -> None:
        if self.connection is None:
            return
        try:
            self.connection.reset_input_buffer()
            self._rx_buffer.clear()
            self._rx_discarding_line = False
        except (OSError, serial.SerialException) as error:
            self.last_telemetry = f"Nano 連線中斷：{error}"
            print(f"Nano 讀取錯誤：{error}。請按 n 重新連線。")
            self.close(send_ready=False)

    def poll(self) -> list[str]:
        events: list[str] = []
        if self.connection is None:
            return events
        try:
            waiting = min(self.connection.in_waiting, MAX_NANO_READ_BYTES)
            if not waiting:
                return events
            received = self.connection.read(waiting)
            lines: list[bytes | None] = []
            for value in received:
                if self._rx_discarding_line:
                    if value == 10:
                        self._rx_discarding_line = False
                elif value == 10:
                    lines.append(bytes(self._rx_buffer))
                    self._rx_buffer.clear()
                elif len(self._rx_buffer) < MAX_NANO_LINE_BYTES:
                    self._rx_buffer.append(value)
                else:
                    self._rx_buffer.clear()
                    self._rx_discarding_line = True
                    lines.append(None)
            for raw_line in lines:
                if raw_line is None:
                    self.last_telemetry = "ERROR,SERIAL_LINE_TOO_LONG"
                    print("Nano 錯誤：序列資料行過長，已捨棄至換行。")
                    events.append("ERROR,SERIAL_LINE_TOO_LONG")
                    continue
                line = raw_line.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                # New: TEL,state,link,ballX,ballY,eXPct,eYPct,uXPct,uYPct,tiltXDeg,tiltYDeg,servoX,servoY,ageMs
                # Legacy 12-field TEL is accepted only for RUN startup compatibility before reflashing.
                parts = line.split(",")
                if parts[0] == "TGT" and len(parts) == 4:
                    try:
                        x, y = float(parts[1]), float(parts[2])
                        if math.isfinite(x) and math.isfinite(y) and parts[3] in ("0", "1"):
                            self.joystick_target = (x, y, parts[3] == "1")
                            armed = parts[3] == "1"
                            if armed != getattr(self, "last_joystick_armed", None):
                                print("搖桿：已啟用。" if armed else "搖桿：請放回中立位置以啟用。")
                                self.last_joystick_armed = armed
                    except ValueError:
                        pass
                elif parts[0] == "TEL" and len(parts) in (12, 14):
                    self.last_telemetry = line
                    events.append(line)
                    # Keep setup quiet: print one status when READY changes, then
                    # stream live servo feedback only after the user presses r.
                    controller_state = parts[1]
                    if controller_state != "RUN":
                        events.append(f"TEL_STATE,{controller_state}")
                    if controller_state == "RUN" or controller_state != self.last_terminal_controller_state:
                        if len(parts) == 14:
                            detail = (
                                f" 誤差=({parts[5]},{parts[6]})%"
                                f" 輸出=({parts[7]},{parts[8]})%"
                                f" 傾角=({parts[9]},{parts[10]})度"
                                f" 伺服角度=X:{parts[11]}度 Y:{parts[12]}度"
                                f" 位置資料時間差={parts[13]}毫秒"
                            )
                        else:
                            detail = (
                                f" 舊版誤差毫米=({parts[5]},{parts[6]})"
                                f" 舊版傾角=({parts[7]},{parts[8]})度"
                                f" 伺服角度=X:{parts[9]}度 Y:{parts[10]}度"
                                f" 位置資料時間差={parts[11]}毫秒"
                            )
                        print(
                            "伺服狀態"
                            f" 模式={controller_state} 連線={parts[2]}"
                            f" 球位置=({parts[3]},{parts[4]})"
                            f"{detail}"
                        )
                    self.last_terminal_controller_state = controller_state
                else:
                    self.last_telemetry = line
                    if line.startswith(("ERROR,", "STATE,", "POS,OK", "GEOM,OK")):
                        events.append(line)
                    if line != "POS,OK":  # Keep ACKs for the state machine, without flooding the terminal.
                        print(f"NANO {line}")
        except (OSError, serial.SerialException) as error:
            self.last_telemetry = f"Nano 連線中斷：{error}"
            print(f"Nano 讀取錯誤：{error}。請按 n 重新連線。")
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
            self.joystick_target = None
            self.last_joystick_armed = None
            self._rx_buffer.clear()
            self._rx_discarding_line = False


class BallVision:
    def __init__(self, config_path: Path, config: dict[str, Any], transport: NanoTransport, headless: bool) -> None:
        self.config_path = config_path
        self.config = config
        self.transport = transport
        self.headless = headless
        self.mode = "normal"
        self.corners: list[tuple[float, float]] = [tuple(point) for point in config["platform"]["corners_px"]]
        self.pending_corners: list[tuple[float, float]] = []
        self.homography: np.ndarray | None = None
        self.corner_order_message: str | None = None
        self.corner_success_message: str | None = None
        self.filtered_position: np.ndarray | None = None
        self.last_valid_ball_center: tuple[float, float] | None = None
        self.zero_reference = np.array([0.0, 0.0], dtype=np.float32)
        self.latest_frame: np.ndarray | None = None
        self.cross_check_marker: tuple[float, float] | None = None
        self.cross_check_marker_until = 0.0
        self.latest_hsv: np.ndarray | None = None
        self.corner_labels = CORNER_LABELS_TLTRBRBL
        self.pid_running = False
        self.run_start_phase: str | None = None
        self.run_start_deadline = 0.0
        self.run_start_ack_position: tuple[float, float] | None = None
        self.run_start_pos_sent_at = 0.0
        self.run_start_events_seen = 0
        self.run_start_last_event: str | None = None
        self.last_sent_at = 0.0
        self.last_lost_at: float | None = None
        self.missing_ball_since: float | None = None
        self.latest_position: tuple[float, float] | None = None
        self.last_outside_platform_position: tuple[float, float] | None = None
        self.last_frame_at = time.monotonic()
        self.fps = 0.0
        self._rebuild_homography(persist_normalized=True)

    def _rebuild_homography(self, persist_normalized: bool = False) -> None:
        self.corner_order_message = None
        if len(self.corners) != 4:
            self.homography = None
            return
        normalized, problem = normalize_platform_corners(self.corners)
        if problem is not None or normalized is None:
            self.homography = None
            self.corner_order_message = f"儲存的平台角點設定無效：{problem}"
            print(self.corner_order_message)
            return
        if normalized != self.corners:
            old_corners = self.corners
            self.corners = normalized
            self.config["platform"]["corners_px"] = [list(point) for point in normalized]
            self.zero_reference = np.array([0.0, 0.0], dtype=np.float32)
            control_config = self.config.setdefault("control", {})
            control_config.pop("zero_reference_pct", None)
            control_config.pop("zero_reference_mm", None)
            self.corner_success_message = "平台四角已儲存，計算出的中心為零點。"
            print("已依左上、右上、右下、左下儲存平台角點；中心為 X=0、Y=0。")
            print(f"原順序：{old_corners}")
            print(f"新順序：{self.corners}")
            if persist_normalized:
                save_config(self.config_path, self.config)
        source = np.float32(self.corners)
        destination = np.float32([
            [0, 0],
            [PLATFORM_FULL_SPAN_PCT, 0],
            [PLATFORM_FULL_SPAN_PCT, PLATFORM_FULL_SPAN_PCT],
            [0, PLATFORM_FULL_SPAN_PCT],
        ])
        self.homography = cv2.getPerspectiveTransform(source, destination)

    def _has_ball_hsv(self) -> bool:
        settings = self.config.get("ball_hsv", {})
        return all(key in settings for key in ("hue", "hue_tolerance", "saturation_min", "value_min"))

    def _has_platform(self) -> bool:
        return len(self.corners) == 4 and self.homography is not None and self.corner_order_message is None

    def _ready_items(self) -> dict[str, bool]:
        return {
            "B 球體顏色 HSV": self._has_ball_hsv(),
            "C 平台邊界（中心為零點）": self._has_platform(),
        }

    def _ready_complete(self) -> bool:
        return all(self._ready_items().values())

    def _ready_prompt(self) -> str:
        if self.mode == "ball_colour":
            return "B：請點選球中心，擷取 HSV 顏色。"
        if self.mode == "corners":
            next_index = min(len(self.pending_corners) + 1, 4)
            return f"C：請依左上、右上、右下、左下順序點選第 {next_index}/4 個平台角點。"
        if self.corner_order_message is not None:
            return self.corner_order_message
        if self._ready_complete():
            return "READY 設定完成。按 R 啟動；按 P 維持待命；按 B 或 C 重新校正。"
        if not self._has_ball_hsv():
            return "下一步：按 B，再點選球中心以擷取 HSV 顏色。"
        if not self._has_platform():
            return "下一步：按 C，依左上、右上、右下、左下順序點選平台四角。"
        return "C 校正完成後會自動計算平台中心。"

    def _print_ready_checklist(self) -> None:
        states = " | ".join(f"{'[x]' if done else '[ ]'} {name}" for name, done in self._ready_items().items())
        print(f"READY 準備清單：{states}")
        if self.corner_order_message is not None:
            print(self.corner_order_message)
        print(self._ready_prompt())

    def _save_and_report_ready(self) -> None:
        save_config(self.config_path, self.config)
        self._print_ready_checklist()

    def _calibration_click_point(self, x: int, y: int) -> tuple[float, float] | None:
        if self.latest_frame is None:
            return None
        frame_height, frame_width = self.latest_frame.shape[:2]
        if 0 <= x < frame_width and 0 <= y < frame_height:
            return float(x), float(y)
        print(
            f"已忽略校正點 ({x}, {y})：點選位置超出攝影機畫面 "
            f"{frame_width}x{frame_height}。請保持預覽原始大小，並點選畫面內的區域。"
        )
        return None

    def _apply_saved_config(self) -> None:
        self.pid_running = False
        self.run_start_phase = None
        self.run_start_ack_position = None
        self.run_start_pos_sent_at = 0.0
        self.transport.send("READY")
        self.config = load_config(self.config_path)
        self.cross_check_marker = None
        self.corners = [tuple(point) for point in self.config["platform"].get("corners_px", [])]
        self.pending_corners = []
        self.zero_reference = np.array([0.0, 0.0], dtype=np.float32)
        self.filtered_position = None
        self.mode = "normal"
        self._rebuild_homography(persist_normalized=True)
        print("已套用儲存的 camera_config.json。")
        self._print_ready_checklist()

    def _platform_position(self, point: tuple[float, float]) -> tuple[float, float] | None:
        if self.homography is None:
            return None
        mapped = cv2.perspectiveTransform(np.float32([[point]]), self.homography)[0][0]
        return (
            float(mapped[0] - PLATFORM_HALF_SPAN_PCT),
            float(PLATFORM_HALF_SPAN_PCT - mapped[1]),
        )

    def _zero_reference_is_inside_platform(self, position: np.ndarray | tuple[float, float] | None) -> bool:
        if position is None:
            return False
        try:
            zero_x = float(position[0])
            zero_y = float(position[1])
        except (TypeError, ValueError, IndexError):
            return False
        return (
            math.isfinite(zero_x)
            and math.isfinite(zero_y)
            and -PLATFORM_HALF_SPAN_PCT < zero_x < PLATFORM_HALF_SPAN_PCT
            and -PLATFORM_HALF_SPAN_PCT < zero_y < PLATFORM_HALF_SPAN_PCT
        )

    def _position_is_inside_platform(self, position: tuple[float, float] | None) -> bool:
        if position is None:
            return False
        x, y = position
        return (
            math.isfinite(x)
            and math.isfinite(y)
            and -PLATFORM_HALF_SPAN_PCT <= x <= PLATFORM_HALF_SPAN_PCT
            and -PLATFORM_HALF_SPAN_PCT <= y <= PLATFORM_HALF_SPAN_PCT
        )

    def _platform_geometry(self) -> PlatformGeometry | None:
        if not self._zero_reference_is_inside_platform(self.zero_reference):
            return None
        zero_x = float(self.zero_reference[0])
        zero_y = float(self.zero_reference[1])
        geometry = PlatformGeometry(
            x_min_pct=-PLATFORM_HALF_SPAN_PCT - zero_x,
            x_max_pct=PLATFORM_HALF_SPAN_PCT - zero_x,
            y_min_pct=-PLATFORM_HALF_SPAN_PCT - zero_y,
            y_max_pct=PLATFORM_HALF_SPAN_PCT - zero_y,
        )
        return geometry

    def _send_geometry_config(self) -> bool:
        geometry = self._platform_geometry()
        if geometry is None:
            print("無法啟動 RUN：零點不在校正的平台邊界內，請按 C 重新校正。")
            return False
        if not self.transport.send(geometry.as_command()):
            print("無法啟動 RUN：平台幾何資料傳送失敗。請按 n 重新連線，再按 R 重試。")
            return False
        print(
            "已傳送校正後的平台幾何資料至 Nano："
            f"X[{geometry.x_min_pct:+.1f},{geometry.x_max_pct:+.1f}]%, "
            f"Y[{geometry.y_min_pct:+.1f},{geometry.y_max_pct:+.1f}]%."
        )
        return True

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
        mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        # A narrow dark seam can split one ball into two HSV contours. Join
        # only small gaps before candidate tracking so it does not follow a
        # fragment with a misleading centre/radius. This does not relax the
        # contact-point boundary check.
        seam_size = max(1, min(31, int(self.config["detection"].get("seam_close_kernel_px", 7)))) | 1
        if seam_size > kernel_size:
            seam_kernel = np.ones((seam_size, seam_size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, seam_kernel)
        return mask

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
        if self.pid_running and self.last_valid_ball_center is not None:
            # Associate candidates with the last ball seen on the platform.
            # Do not remove out-of-platform candidates: a real ball crossing
            # the boundary must still be detected and stop control.
            previous_x, previous_y = self.last_valid_ball_center
            _, center, radius = min(
                candidates,
                key=lambda item: (item[1][0] - previous_x) ** 2 + (item[1][1] - previous_y) ** 2,
            )
        else:
            _, center, radius = max(candidates, key=lambda item: item[0])
        return center, radius

    def _ball_contact_point(self, center: tuple[float, float], radius: float) -> tuple[float, float]:
        """Estimate the ball's platform contact point from its visible circle.

        The homography is calibrated on the platform plane, not at the ball's
        elevated visual centre. Use the optional measured image-space direction;
        otherwise use the platform's near-edge direction for older configurations.
        """
        if len(self.corners) != 4:
            return center
        far_mid = np.mean(np.float32(self.corners[:2]), axis=0)
        near_mid = np.mean(np.float32(self.corners[2:]), axis=0)
        direction = near_mid - far_mid
        measured_direction = self.config.get("detection", {}).get("contact_direction_px")
        if isinstance(measured_direction, (list, tuple)) and len(measured_direction) == 2:
            try:
                measured = np.array(measured_direction, dtype=np.float32)
                if np.all(np.isfinite(measured)) and float(np.linalg.norm(measured)) > 1e-6:
                    direction = measured
            except (TypeError, ValueError, OverflowError):
                pass  # Invalid optional direction: retain the legacy near-edge estimate.
        length = float(np.linalg.norm(direction))
        if not math.isfinite(length) or length <= 1e-6:
            return center
        configured = float(self.config.get("detection", {}).get("contact_offset_radius", 0.85))
        offset_ratio = min(1.0, max(0.0, configured))
        offset = direction / length * (max(0.0, float(radius)) * offset_ratio)
        return float(center[0] + offset[0]), float(center[1] + offset[1])

    def _send_position_command(self, position: tuple[float, float]) -> bool:
        timestamp_ms = int(time.monotonic() * 1000)
        sent = self.transport.send(f"POS,{position[0]:.2f},{position[1]:.2f},{timestamp_ms}")
        if sent:
            self.last_sent_at = time.monotonic()
        return sent

    def _send_position(
        self,
        position: tuple[float, float] | None,
        loss_reason: str = "NO_DETECTION",
        outside_position: tuple[float, float] | None = None,
    ) -> bool:
        # Calibration may run while connected, but no live position commands are
        # sent until r explicitly starts PID control.
        if not self.pid_running:
            return False
        now = time.monotonic()
        if position is None:
            if loss_reason == "NO_DETECTION":
                if self.missing_ball_since is None:
                    self.missing_ball_since = now
                if now - self.missing_ball_since < MISSING_BALL_GRACE_SECONDS:
                    # Do not resend an old position while the ball is missing.
                    # Nano's independent position watchdog remains active.
                    return False
            else:
                self.missing_ball_since = None
            interval = 1.0 / float(self.config["transport"]["lost_heartbeat_hz"])
            if self.last_lost_at is None or now - self.last_lost_at >= interval:
                if self.transport.send("LOST"):
                    self.last_lost_at = now
                    detail = ""
                    if outside_position is not None:
                        detail = f"，位置=({outside_position[0]:.2f},{outside_position[1]:.2f})%"
                    explanation = {"NO_DETECTION": "未偵測到球", "OUTSIDE": "超出平台邊界", "INVALID_GEOMETRY": "平台幾何資料無效"}.get(loss_reason, "其他原因")
                    print(f"影像追蹤中斷：{explanation}（{loss_reason}）{detail}；已傳送 LOST 至 Nano。")
                    return True
            return False
        self.missing_ball_since = None
        self.last_lost_at = None
        self._send_position_command(position)
        return False

    def _save_outside_frame(
        self, frame: np.ndarray, ball: tuple[tuple[float, float], float],
        contact: tuple[float, float], position: tuple[float, float],
    ) -> None:
        """Keep the actual frame that triggered LOST, after sending the stop."""
        center, radius = ball
        print(
            f"追蹤點超出邊界：球心像素=({center[0]:.1f},{center[1]:.1f}) "
            f"半徑像素={radius:.1f} 接觸點像素=({contact[0]:.1f},{contact[1]:.1f})"
        )
        output_path = self.config_path.parent / "vision_last_outside.png"
        try:
            annotated = self._draw_overlay(frame, ball, None)
            text = f"X={position[0]:.1f}% Y={position[1]:.1f}%"
            cv2.putText(annotated, text, (10, max(20, annotated.shape[0] - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
            if not cv2.imwrite(str(output_path), annotated):
                print(f"無法將越界影格儲存至 {output_path}。")
                return
        except (OSError, ValueError, cv2.error) as error:
            print(f"無法儲存越界影格：{error}")
            return
        print(f"已儲存越界影格：{output_path}")

    def _safe_ready(self, message: str) -> None:
        was_active = self.pid_running or self.run_start_phase is not None
        self.pid_running = False
        self.run_start_phase = None
        self.run_start_ack_position = None
        self.run_start_pos_sent_at = 0.0
        self.run_start_events_seen = 0
        self.run_start_last_event = None
        self.filtered_position = None
        self.last_sent_at = 0.0
        self.last_lost_at = None
        self.missing_ball_since = None
        self.transport.send("READY")
        if was_active:
            print(message)
        else:
            print("目前已在 READY 待命；不會持續傳送位置或伺服控制指令。")

    def _request_run(self, position: tuple[float, float]) -> None:
        self.transport.discard_pending_input()
        if not self._send_geometry_config():
            return
        self.run_start_phase = "wait_geom_ack"
        self.run_start_ack_position = position
        self.run_start_pos_sent_at = 0.0
        self.run_start_events_seen = 0
        self.run_start_last_event = None
        self.run_start_deadline = time.monotonic() + 0.6
        print("已要求啟動 RUN：等待 Nano 回覆 GEOM,OK，之後才會傳送位置與啟動指令。")

    def _send_initial_pos_after_geometry(self) -> None:
        position = self.latest_position
        if position is None:
            self._safe_ready("已取消 RUN：Nano 接受幾何資料前失去球的位置。請讓球入鏡，再按 R。")
            return
        if not self._send_position_command(position):
            self._safe_ready("已取消 RUN：幾何資料確認後無法傳送新位置。請按 n 重新連線，再按 R。")
            return
        self.run_start_phase = "wait_pos_ack"
        self.run_start_ack_position = position
        self.run_start_pos_sent_at = time.monotonic()
        self.run_start_events_seen = 0
        self.run_start_last_event = None
        self.run_start_deadline = self.run_start_pos_sent_at + 0.6
        print("Nano 已接受平台幾何資料並傳送目前位置；確認後會再傳送最新位置與 RUN。")

    def _is_implicit_pos_ack(self, nano_event: str) -> bool:
        if self.run_start_ack_position is None:
            return False
        parts = nano_event.split(",")
        if len(parts) not in (12, 14) or parts[:3] != ["TEL", "READY", "OK"]:
            return False
        try:
            telemetry_x = float(parts[3])
            telemetry_y = float(parts[4])
            age_ms = float(parts[13] if len(parts) == 14 else parts[11])
        except ValueError:
            return False
        if not all(math.isfinite(value) for value in (telemetry_x, telemetry_y, age_ms)):
            return False
        sent_x, sent_y = self.run_start_ack_position
        return age_ms <= 500.0 and abs(telemetry_x - sent_x) <= 2.0 and abs(telemetry_y - sent_y) <= 2.0

    def _is_ready_state_pos_ack(self, nano_event: str) -> bool:
        # Older already-flashed firmware may answer the first accepted POS from WAIT_LINK
        # with only STATE,READY. The final POS sent below is still the freshness gate.
        return (
            nano_event == "STATE,READY"
            and self.run_start_ack_position is not None
            and self.run_start_pos_sent_at > 0.0
            and time.monotonic() - self.run_start_pos_sent_at <= 0.6
        )

    def _send_final_pos_and_run(self, ack_label: str) -> None:
        position = self.latest_position
        if position is None:
            self._safe_ready("已取消 RUN：收到 Nano 確認前失去球的位置。請讓球入鏡，再按 R。")
            return
        if not self._send_position_command(position):
            self._safe_ready("已取消 RUN：啟動前無法傳送最新位置。請按 n 重新連線，再按 R。")
            return
        self.transport.send("RUN")
        self.run_start_phase = "wait_run_ack"
        self.run_start_ack_position = None
        self.run_start_pos_sent_at = 0.0
        self.run_start_events_seen = 0
        self.run_start_last_event = None
        self.run_start_deadline = time.monotonic() + 0.6
        print(f"Nano 已透過 {ack_label} 確認位置；已傳送最新位置及 RUN，等待 STATE,RUN。")

    def _run_handshake_timeout_detail(self) -> str:
        if self.run_start_events_seen == 0:
            if self.run_start_phase == "wait_geom_ack":
                return (
                    "傳送 GEOM 後未收到 Nano 回應。請燒錄最新版 Arduino Nano 韌體；舊版沒有安全的幾何資料相容機制。"
                )
            return (
                "傳送 POS 後未收到 Nano 回應。可能是韌體未確認 POS、開啟 USB 時 Nano 重置，或序列埠 RX 未收到資料。"
            )
        return f"已收到 {self.run_start_events_seen} 筆 Nano 事件；最後一筆為 {self.run_start_last_event!r}。"

    def _handle_nano_events(self) -> None:
        for nano_event in self.transport.poll():
            if self.run_start_phase is not None:
                self.run_start_events_seen += 1
                self.run_start_last_event = nano_event
            if self.run_start_phase == "wait_geom_ack":
                if nano_event == "GEOM,OK":
                    self._send_initial_pos_after_geometry()
                    continue
                if nano_event.startswith("ERROR,"):
                    self._safe_ready(f"Nano 已取消 RUN（{nano_event}）。若不支援 GEOM，請燒錄最新版 Nano 韌體。")
                    continue
            if self.run_start_phase == "wait_pos_ack":
                if nano_event.startswith("POS,OK"):
                    self._send_final_pos_and_run("POS,OK")
                    continue
                if self._is_implicit_pos_ack(nano_event):
                    self._send_final_pos_and_run("TEL,READY,OK 相容模式")
                    continue
                if self._is_ready_state_pos_ack(nano_event):
                    self._send_final_pos_and_run("STATE,READY 相容模式")
                    continue
                if nano_event.startswith("ERROR,"):
                    self._safe_ready(f"Nano 已取消 RUN（{nano_event}），並回到 READY 待命。")
                    continue
            if self.run_start_phase == "wait_run_ack":
                if nano_event == "STATE,RUN":
                    self.pid_running = True
                    self.run_start_phase = None
                    self.last_sent_at = 0.0
                    self.last_lost_at = None
                    self.missing_ball_since = None
                    print("Nano 已確認 RUN；程式正隨每個影格傳送最新位置。")
                    continue
                if nano_event.startswith("ERROR,"):
                    self._safe_ready(f"Nano 已拒絕 RUN（{nano_event}），並回到 READY 待命。")
                    continue
            if self.pid_running and (nano_event.startswith("ERROR,") or nano_event in ("STATE,READY", "TEL_STATE,READY")):
                self._safe_ready(f"Nano 已停止 RUN（{nano_event}）並回到 READY；請排除問題後再按 R。")
                break
        if self.run_start_phase is not None and time.monotonic() > self.run_start_deadline:
            phase = self.run_start_phase
            detail = self._run_handshake_timeout_detail()
            self.run_start_ack_position = None
            self.run_start_pos_sent_at = 0.0
            self._safe_ready(f"啟動 RUN 時等待 Nano 回應逾時（{phase}）。{detail} 已回到 READY；請檢查序列埠 RX/TX 與 Nano 韌體，再按 R。")

    def _check_platform_cross(self, frame: np.ndarray, sample: tuple[float, float]) -> tuple[tuple[float, float], float, float, float, float] | None:
        """Estimate a two-axis mark on one READY frame; never alter calibration."""
        if not self._has_platform():
            return None
        sx, sy = round(sample[0]), round(sample[1])
        if not (0 <= sx < frame.shape[1] and 0 <= sy < frame.shape[0]):
            return None
        hue, saturation, value = (int(v) for v in cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[sy, sx])
        warped_h = np.diag([2.0, 2.0, 1.0]) @ self.homography
        warped = cv2.warpPerspective(frame, warped_h, (401, 401))
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        if value < 100:
            mask = cv2.inRange(hsv[:, :, 2], 0, min(115, value + 35))
        elif saturation >= 65:
            hue_delta = np.abs(hsv[:, :, 0].astype(np.int16) - hue)
            hue_delta = np.minimum(hue_delta, 180 - hue_delta)
            mask = ((hue_delta <= 10) & (hsv[:, :, 1] >= max(50, saturation - 80)) &
                    (hsv[:, :, 2] >= max(45, value - 110))).astype(np.uint8) * 255
        else:
            print("十字線檢查：取樣太亮或有反光，請點選清楚的黑色線條。")
            return None
        # Exclude extrapolated borders, where perspective fill or calibration edges can look like ink.
        mask[:3, :] = 0
        mask[-3:, :] = 0
        mask[:, :3] = 0
        mask[:, -3:] = 0
        if cv2.countNonZero(mask) > mask.size * 0.30:
            print("十字線檢查：黑色區域太多，可能混入背景或陰影，請改善背景及光線。")
            return None
        segments = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=45,
                                   minLineLength=90, maxLineGap=25)
        if segments is None:
            return None
        sample_warp = cv2.perspectiveTransform(np.float32([[sample]]), warped_h)[0, 0]
        if not np.all(np.isfinite(sample_warp)) or not (3 <= sample_warp[0] <= 397 and 3 <= sample_warp[1] <= 397):
            print("十字線檢查：請點選四角框內的黑色線條。")
            return None
        horizontal, vertical = [], []
        for line in segments[:, 0, :]:
            x1, y1, x2, y2 = (float(v) for v in line)
            dx, dy = x2 - x1, y2 - y1
            length = math.hypot(dx, dy)
            if abs(dx) >= 90 and abs(dy) <= abs(dx) * 0.45:
                m = dy / dx
                horizontal.append((m, y1 - m * x1, length, min(x1, x2), max(x1, x2)))
            elif abs(dy) >= 90 and abs(dx) <= abs(dy) * 0.45:
                m = dx / dy
                vertical.append((m, x1 - m * y1, length, min(y1, y2), max(y1, y2)))
        best = None
        for mh, bh, lh, hmin, hmax in horizontal:
            for mv, bv, lv, vmin, vmax in vertical:
                denominator = 1.0 - mv * mh
                if abs(denominator) < 1e-6:
                    continue
                px = (mv * bh + bv) / denominator
                py = mh * px + bh
                if not (3 <= px <= 397 and 3 <= py <= 397):
                    continue
                # Both segments must actually reach the intersection; sampled ink must
                # lie on at least one of the two selected lines, not an unrelated object.
                if px < hmin - 35 or px > hmax + 35 or py < vmin - 35 or py > vmax + 35:
                    continue
                dh = abs(sample_warp[1] - (mh * sample_warp[0] + bh)) / math.hypot(1, mh)
                dv = abs(sample_warp[0] - (mv * sample_warp[1] + bv)) / math.hypot(1, mv)
                if min(dh, dv) > 12:
                    continue
                score = lh + lv
                if best is None or score > best[0]:
                    best = (score, px, py, mh, mv)
        if best is None:
            return None
        _, px, py, mh, mv = best
        cross_img = cv2.perspectiveTransform(np.float32([[[px, py]]]), np.linalg.inv(warped_h))[0, 0]
        if not np.all(np.isfinite(cross_img)):
            return None
        # dx,dy are percentages of the whole calibrated width/height, not PID coordinates.
        return ((float(cross_img[0]), float(cross_img[1])), (px - 200.0) / 4.0,
                (200.0 - py) / 4.0, math.degrees(math.atan(mh)), math.degrees(math.atan(mv)))

    def _draw_overlay(self, frame: np.ndarray, ball: tuple[tuple[float, float], float] | None,
                      position: tuple[float, float] | None) -> np.ndarray:
        overlay = frame.copy()
        display_corners = self.pending_corners if self.mode == "corners" and self.pending_corners else self.corners
        if display_corners:
            points = np.int32(display_corners)
            cv2.polylines(overlay, [points], len(display_corners) == 4, (255, 255, 0), 2)
            for point in points:
                cv2.circle(overlay, tuple(point), 3, (255, 255, 0), -1)
        if ball:
            center, radius = ball
            contact = self._ball_contact_point(center, radius)
            cv2.circle(overlay, (round(center[0]), round(center[1])), round(radius), (0, 255, 0), 2)
            cv2.circle(overlay, (round(center[0]), round(center[1])), 3, (0, 0, 255), -1)
            cv2.circle(overlay, (round(contact[0]), round(contact[1])), 4, (0, 165, 255), -1)

        target = getattr(self.transport, "joystick_target", None)
        if self.pid_running and target is not None and self.homography is not None:
            mapped = np.float32([[[target[0] + PLATFORM_HALF_SPAN_PCT,
                                   PLATFORM_HALF_SPAN_PCT - target[1]]]])
            point = cv2.perspectiveTransform(mapped, np.linalg.inv(self.homography))[0][0]
            if np.all(np.isfinite(point)):
                center = (int(round(point[0])), int(round(point[1])))
                cv2.drawMarker(overlay, center, (255, 0, 255), cv2.MARKER_CROSS, 18, 2)

        marker = getattr(self, "cross_check_marker", None)
        if marker is not None and time.monotonic() < self.cross_check_marker_until:
            cv2.drawMarker(overlay, (round(marker[0]), round(marker[1])), (255, 0, 0),
                           cv2.MARKER_CROSS, 20, 2)

        if self.pid_running:
            state_index = 0
        elif self.run_start_phase is not None:
            state_index = 1
        elif self.mode != "normal" or not self._ready_complete():
            state_index = 2
        else:
            state_index = 3
        # OPEN means the serial port is open; it is not a live Nano heartbeat.
        _draw_status(overlay, state_index, self.transport.is_connected)
        return overlay

    def _mouse_callback(self, event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.latest_frame is None:
            return
        click_point = self._calibration_click_point(x, y)
        if click_point is None:
            return
        click_x, click_y = click_point
        if self.mode == "cross_sample":
            self.mode = "normal"
            self.cross_check_marker = None
            result = self._check_platform_cross(self.latest_frame, (click_x, click_y))
            if result is None:
                print("十字線檢查：無法可靠辨識兩條長線。請移開球、減少反光，按 X 後再點黑色線條。")
            else:
                self.cross_check_marker, dx, dy, h_angle, v_angle = result
                self.cross_check_marker_until = time.monotonic() + 5.0
                print(f"十字線檢查：交點相對四角中心 X={dx:+.1f}% 平台寬、Y={dy:+.1f}% 平台高（右與遠端為正）。")
                print(f"十字線檢查：透視校正後兩軸偏轉角約 {h_angle:+.1f}°、{v_angle:+.1f}°；藍色十字顯示 5 秒。")
                if abs(dx) > 5.0 or abs(dy) > 5.0 or abs(h_angle) > 8.0 or abs(v_angle) > 8.0:
                    print("十字線檢查：中心差超過寬／高的 5% 或方向偏轉超過 8°；請檢查黑線、反光與四角點選。")
                else:
                    print("十字線檢查：中心差及方向在暫定提醒門檻內。")
                print("十字線只供輔助檢查，無法保證四角正確；結果可能受遮擋、反光與背景影響。校正和 PID 值沒有改變。")
            return
        if self.mode == "ball_colour" and self.latest_hsv is not None:
            if 0 <= int(click_y) < self.latest_hsv.shape[0] and 0 <= int(click_x) < self.latest_hsv.shape[1]:
                hue, saturation, value = (int(component) for component in self.latest_hsv[int(click_y), int(click_x)])
                settings = self.config["ball_hsv"]
                settings["hue"] = hue
                settings["saturation_min"] = max(20, saturation - 70)
                settings["value_min"] = max(20, value - 70)
                self.mode = "normal"
                print(
                    f"B 已儲存球中心像素的 HSV：H={hue}, "
                    f"S>={settings['saturation_min']}, V>={settings['value_min']}."
                )
                self._save_and_report_ready()
            return

        if self.mode == "corners":
            self.pending_corners.append(click_point)
            print(f"C 已記錄角點 {len(self.pending_corners)}/4（{click_x:.0f}, {click_y:.0f}）。")
            if len(self.pending_corners) == 4:
                normalized, problem = normalize_platform_corners(self.pending_corners)
                if problem is not None or normalized is None:
                    self.pending_corners = []
                    self.mode = "normal"
                    print(f"C 平台邊界無效：{problem}")
                    print("已保留先前有效的平台設定，請按 C 重試。")
                    self._print_ready_checklist()
                    return
                self.corners = normalized
                self.pending_corners = []
                self.config["platform"]["corners_px"] = [list(point) for point in normalized]
                self._rebuild_homography()
                self.filtered_position = None
                self.zero_reference = np.array([0.0, 0.0], dtype=np.float32)
                control_config = self.config.setdefault("control", {})
                control_config.pop("zero_reference_pct", None)
                control_config.pop("zero_reference_mm", None)
                self.corner_success_message = "C 已儲存平台四角，計算出的中心為零點。"
                self.mode = "normal"
                print(f"C 已依左上、右上、右下、左下順序儲存平台邊界：{self.corners}")
                print("平台邊界已儲存，計算出的平台中心為零點；偵測到球後可按 R 啟動。")
                self._save_and_report_ready()
            else:
                next_label = self.corner_labels[len(self.pending_corners)]
                corner_name = {"TL": "左上", "TR": "右上", "BR": "右下", "BL": "左下"}[next_label]
                print(f"C 下一步：點選{corner_name}角（{len(self.pending_corners) + 1}/4）。")
            return

        print("READY 待命：按 B 設定球色，按 C 校正平台四角，設定完成後按 R 啟動。")

    def handle_key(self, key: int) -> bool:
        if key in (ord("q"), 27):
            return False
        if key == ord("n"):
            self._safe_ready("已停止控制並回到 READY，準備搜尋 Nano。")
            self.mode = "normal"
            self.transport.search_and_connect()
        elif key == ord("a"):
            self._apply_saved_config()
        elif key == ord("b"):
            self._safe_ready("已選擇 B 並回到 READY，停止傳送位置與伺服控制指令。")
            self.mode = "ball_colour"
            print("B：請點選球中心，只會儲存 HSV 顏色。")
        elif key in (ord("x"), ord("X")):
            self._safe_ready("已選擇 X：停止控制以檢查十字線。")
            self.mode = "normal"
            self.cross_check_marker = None
            if not self._has_platform():
                print("十字線檢查：請先用 C 校正平台四角。")
            else:
                self.mode = "cross_sample"
                print("十字線檢查：移開球，點一下黑色十字線的清楚線段；兩條線都要可見。按 P 取消。")
        elif key == ord("c"):
            self.cross_check_marker = None
            self._safe_ready("已選擇 C 並回到 READY，停止傳送位置與伺服控制指令。")
            self.pending_corners = []
            self.filtered_position = None
            self.mode = "corners"
            print("C: 依序點四角：TL -> TR -> BR -> BL（左上、右上、右下、左下）。")
            print("C 的點選進度會顯示在終端機；新四角通過驗證前，仍保留先前有效設定。")
        elif key in (ord("p"), ord("P")):
            self._safe_ready("已按 P、回到 READY 並停止傳送位置與伺服控制指令。")
            self.mode = "normal"
        elif key in (ord("r"), ord("R")):
            if self.pid_running:
                self._safe_ready("運行中按下 R，已回到 READY 並停止控制。")
            elif self.run_start_phase is not None:
                print("啟動 RUN 正在等待 Nano 確認；按 P 可安全取消並回到 READY。")
            elif self.mode != "normal":
                print("無法啟動 RUN：請先完成目前的設定步驟，或按 A 套用已儲存設定。")
            elif not self.transport.is_connected:
                print("無法啟動 RUN：Nano 尚未連線，請按 n 搜尋。")
            elif not self._ready_complete():
                print("無法啟動 RUN：READY 準備清單尚未完成。")
                self._print_ready_checklist()
            elif self.latest_position is None:
                if self.last_outside_platform_position is not None:
                    x, y = self.last_outside_platform_position
                    print(
                        "無法啟動 RUN：HSV 偵測位置超出校正後的平台 "
                        f"(X={x:+.1f}%, Y={y:+.1f}%)。請檢查 B 球色設定，或將球放在 C 校正邊界內。"
                    )
                else:
                    print("無法啟動 RUN：尚未偵測到球。請讓球入鏡，再按 R 重試。")
            else:
                self.cross_check_marker = None
                self._request_run(self.latest_position)
        return True

    def run(self, camera_index: int, width: int, height: int, requested_fps: int, max_frames: int) -> None:
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
        capture = cv2.VideoCapture(camera_index, backend)
        if not capture.isOpened():
            raise RuntimeError(f"無法開啟攝影機 index {camera_index}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, requested_fps)
        if not self.headless:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(WINDOW_NAME, self._mouse_callback)
            print("操作說明：PID 參數由 Nano 韌體管理。A 套用儲存設定；B 設定球色；C 依左上、右上、右下、左下校正四角；X 檢查十字線；R 啟動；P 暫停；Q 離開。")
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
                contact_point = self._ball_contact_point(*ball) if ball else None
                position = self._platform_position(contact_point) if contact_point else None
                loss_reason = "NO_DETECTION" if ball is None else "INVALID_GEOMETRY"
                if position is not None and self.zero_reference is not None:
                    position = (
                        position[0] - float(self.zero_reference[0]),
                        position[1] - float(self.zero_reference[1]),
                    )
                if not self._position_is_inside_platform(position):
                    self.last_outside_platform_position = position
                    if position is not None:
                        loss_reason = "OUTSIDE" if all(math.isfinite(value) for value in position) else "INVALID_GEOMETRY"
                    position = None
                else:
                    self.last_outside_platform_position = None
                    if ball is not None:
                        self.last_valid_ball_center = ball[0]
                    sample = np.array(position, dtype=np.float32)
                    alpha = float(self.config["detection"]["filter_alpha"])
                    self.filtered_position = sample if self.filtered_position is None else alpha * sample + (1 - alpha) * self.filtered_position
                    position = (float(self.filtered_position[0]), float(self.filtered_position[1]))
                if position is None:
                    self.filtered_position = None
                self.latest_position = position
                lost_sent = self._send_position(position, loss_reason, self.last_outside_platform_position)
                if (lost_sent and loss_reason == "OUTSIDE" and ball is not None
                        and contact_point is not None and self.last_outside_platform_position is not None):
                    self._save_outside_frame(frame, ball, contact_point, self.last_outside_platform_position)
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
    parser = argparse.ArgumentParser(description="平衡球影像辨識與 Arduino Nano 通訊")
    default_config = Path(__file__).with_name("camera_config.json")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--camera-index", type=int)
    parser.add_argument("--serial-port", help="手動指定序列埠，例如 Windows 的 COM3 或 Linux 的 /dev/ttyUSB0")
    parser.add_argument("--no-serial", action="store_true", help="不連線 Nano，直接開啟 READY 設定")
    parser.add_argument("--headless", action="store_true", help="不顯示預覽視窗，供攝影機診斷使用")
    parser.add_argument("--max-frames", type=int, default=0, help="處理指定影格數後停止（0 表示持續執行）")
    return parser.parse_args()


def _request_graceful_shutdown(signal_number: int, _frame: Any) -> None:
    """Turn shell Ctrl+C/SIGTERM into Python unwinding so run() releases the camera."""
    signal_name = signal.Signals(signal_number).name
    print(f"收到 {signal_name}：正在關閉攝影機視窗並讓 Nano 回到 READY。")
    raise KeyboardInterrupt


def main() -> int:
    # start_camera_vision.sh uses exec, so stopping that shell targets this
    # process directly. Handle both terminal Ctrl+C and ordinary SIGTERM.
    signal.signal(signal.SIGINT, _request_graceful_shutdown)
    signal.signal(signal.SIGTERM, _request_graceful_shutdown)
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
    except KeyboardInterrupt:
        print("攝影機程式已安全停止。")
        return 0
    except RuntimeError as error:
        print(f"錯誤：{error}", file=sys.stderr)
        return 2
    finally:
        transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
