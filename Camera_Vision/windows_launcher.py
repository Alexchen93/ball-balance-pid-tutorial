"""Interactive Windows camera and Nano selection after environment setup."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import cv2
from serial.tools import list_ports


CAMERA_SCAN_LIMIT = 10
PREVIEW_WINDOW = "Camera preview - press Y, N or Q"


def camera_can_read(index: int) -> bool:
    capture = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    try:
        if not capture.isOpened():
            return False
        for _ in range(3):
            ok, frame = capture.read()
            if ok and frame is not None and frame.size:
                return True
        return False
    except cv2.error:
        return False
    finally:
        capture.release()


def find_cameras() -> list[int]:
    return [index for index in range(CAMERA_SCAN_LIMIT) if camera_can_read(index)]


def preview_camera(index: int) -> bool | None:
    """Show live frames; True accepts, False retries, None quits."""
    capture = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    window_created = False
    try:
        if not capture.isOpened():
            print(f"Could not reopen camera index {index}. Try scanning again.")
            return False
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_AUTOSIZE)
        window_created = True
        print("Click the preview window, then press Y to use it, N to choose again, or Q to quit.")
        while True:
            ok, frame = capture.read()
            if not ok or frame is None or not frame.size:
                print("Could not read the camera preview. Close other camera programs and retry.")
                return False
            cv2.putText(frame, f"Camera {index}: Y=use  N=choose again  Q=quit", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(frame, f"Camera {index}: Y=use  N=choose again  Q=quit", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.imshow(PREVIEW_WINDOW, frame)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("y"), ord("Y")):
                return True
            if key in (ord("n"), ord("N")):
                return False
            if key in (ord("q"), ord("Q"), 27):
                return None
            if cv2.getWindowProperty(PREVIEW_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                return False
    except cv2.error as error:
        print(f"Camera preview error: {error}")
        return False
    finally:
        capture.release()
        if window_created:
            try:
                cv2.destroyWindow(PREVIEW_WINDOW)
                cv2.waitKey(1)
            except cv2.error:
                pass


def choose_camera() -> int | None:
    while True:
        print(f"\nSearching for readable cameras (indexes 0 to {CAMERA_SCAN_LIMIT - 1}). Please wait...", flush=True)
        cameras = find_cameras()
        if cameras:
            print("Available cameras:")
            for number, index in enumerate(cameras, start=1):
                print(f"  {number}) Camera index {index}")
        else:
            print("No readable cameras found. Check the connection, Windows camera permissions, and other camera programs.")
        print("Enter a list number, M to enter an index, R to search again, or Q to quit.")
        choice = input("Camera: ").strip().lower()
        if choice == "q":
            return None
        if choice == "r":
            continue
        if choice == "m":
            value = input("Camera index (0 or greater), or B to go back: ").strip().lower()
            if value == "b":
                continue
            if not value.isascii() or not value.isdigit() or len(value) > 3:
                print("Enter a camera index from 0 to 999.")
                continue
            index = int(value)
        elif choice.isascii() and choice.isdigit() and 1 <= int(choice) <= len(cameras):
            index = cameras[int(choice) - 1]
        else:
            print("Invalid choice. Please try again.")
            continue
        accepted = preview_camera(index)
        if accepted is None:
            return None
        if accepted:
            print(f"Selected camera index {index}.")
            return index


def choose_port() -> tuple[bool, str | None]:
    """Return (proceed, port); None port means READY without serial."""
    while True:
        print("\n=== Ball Balance PID | Launch menu ===")
        print("1) Search for and connect Arduino Nano")
        print("2) Open READY setup without Nano")
        print("Q) Quit")
        choice = input("Select [1/2/Q]: ").strip().lower()
        if choice == "q":
            return False, None
        if choice == "2":
            return True, None
        if choice != "1":
            print("Enter 1, 2 or Q.")
            continue

        while True:
            ports = sorted(list_ports.comports(), key=lambda port: port.device)
            if not ports:
                print("No COM ports found. Connect Nano, then search again.")
                choice = input("R to search again, B for menu, or Q to quit: ").strip().lower()
                if choice == "q":
                    return False, None
                if choice == "b":
                    break
                continue
            print("\nDetected serial ports (a listed port is not yet a confirmed Nano):")
            for number, port in enumerate(ports, start=1):
                description = port.description or "Unknown device"
                manufacturer = f"; {port.manufacturer}" if port.manufacturer else ""
                print(f"  {number}) {port.device} - {description}{manufacturer}")
            print("Close Arduino Serial Monitor before connecting. The program will check the connection when it opens.")
            choice = input("Select a port number, R to refresh, B for menu, or Q to quit: ").strip().lower()
            if choice == "q":
                return False, None
            if choice == "b":
                break
            if choice == "r":
                continue
            if choice.isascii() and choice.isdigit() and 1 <= int(choice) <= len(ports):
                port = ports[int(choice) - 1].device
                print(f"Selected {port}. The camera will open in READY mode.")
                return True, port
            print("Invalid choice. Please try again.")


def main() -> int:
    try:
        camera = choose_camera()
        if camera is None:
            return 0
        proceed, port = choose_port()
        if not proceed:
            return 0
        program = Path(__file__).with_name("ball_vision.py")
        command = [sys.executable, str(program), "--camera-index", str(camera)]
        command += ["--serial-port", port] if port else ["--no-serial"]
        return subprocess.run(command, cwd=program.parent, check=False).returncode
    except (EOFError, KeyboardInterrupt):
        print("\nCamera Vision launcher cancelled.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
