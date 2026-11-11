#!/usr/bin/env bash
# Interactive launcher: choose and confirm a mode before the camera opens.
set -Eeuo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python_bin="$script_dir/.venv/bin/python"
[[ -x "$python_bin" ]] || { echo "Python environment not found: $python_bin" >&2; exit 1; }

dev_dir="${CAMERA_VISION_DEV_DIR:-/dev}"
sys_tty_dir="${CAMERA_VISION_SYS_TTY_DIR:-/sys/class/tty}"
launcher_test="${CAMERA_VISION_LAUNCHER_TEST:-}"
manual_port=""
args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial-port) manual_port="${2:?--serial-port needs a path}"; shift 2;;
    --serial-port=*) manual_port="${1#--serial-port=}"; shift;;
    *) args+=("$1"); shift;;
  esac
done
cd "$script_dir"
run() { exec "$python_bin" ball_vision.py "$@"; }

camera_display_name() {
  local device="$1" name detail
  name=$(basename "$device")
  detail=$(python3 - "$name" <<'PY'
import sys
from pathlib import Path
name = sys.argv[1]
node = Path("/sys/class/video4linux") / name / "device"
try:
    node = node.resolve()
except OSError:
    pass
parts = []
for parent in [node, *list(node.parents)[:7]]:
    for field in ("manufacturer", "product", "idVendor", "idProduct"):
        try:
            value = (parent / field).read_text(errors="ignore").strip()
        except OSError:
            value = ""
        if value and value not in parts:
            parts.append(value)
print(" ".join(parts) or "unknown camera")
PY
)
  printf '%s' "$detail"
}

camera_can_read_frame() {
  local device="$1"
  "$python_bin" - "$device" >/dev/null 2>&1 <<'PY'
import cv2
import sys

capture = cv2.VideoCapture(sys.argv[1], cv2.CAP_V4L2)
try:
    ok, frame = capture.read()
    raise SystemExit(0 if ok and frame is not None and frame.size else 1)
finally:
    capture.release()
PY
}

choose_camera_index() {
  local cameras=() unavailable=() device choice index
  local all_cameras=()
  shopt -s nullglob
  all_cameras=(/dev/video*)
  shopt -u nullglob
  if [[ ${#all_cameras[@]} -eq 0 ]]; then
    echo "No camera found. Check that the built-in or USB camera is connected." >&2
    return 1
  fi
  # One UVC camera can expose both a real capture stream and a metadata-only
  # /dev/video node. Probe once so students cannot select the metadata node.
  for device in "${all_cameras[@]}"; do
    if camera_can_read_frame "$device"; then
      cameras+=("$device")
    else
      unavailable+=("$device")
    fi
  done
  if [[ ${#cameras[@]} -eq 0 ]]; then
    echo "Cameras were found, but none can read a frame. Close other camera programs and reconnect the USB camera." >&2
    [[ ${#unavailable[@]} -gt 0 ]] && printf 'Cannot read: %s\n' "${unavailable[*]}" >&2
    return 1
  fi
  echo
  echo "=== Select camera ==="
  echo "Only cameras that can read a frame are listed; metadata and unreadable nodes are omitted."
  echo "Select the camera to use. A newly connected USB camera usually appears as a new item."
  for index in "${!cameras[@]}"; do
    device=${cameras[$index]}
    printf '%d) %s - %s\n' "$((index + 1))" "$device" "$(camera_display_name "$device")"
  done
  while true; do
    read -r -p "Camera number [1-${#cameras[@]}], or q to quit: " choice
    [[ "$choice" =~ ^[Qq]$ ]] && return 2
    if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#cameras[@]} )); then
      device=${cameras[$((choice - 1))]}
      [[ $(basename "$device") =~ ^video([0-9]+)$ ]] || { echo "Invalid camera device: $device" >&2; continue; }
      CAMERA_INDEX="${BASH_REMATCH[1]}"
      return 0
    fi
    echo "Enter a number from the list."
  done
}

port_is_busy() {
  local port="$1" busy_port
  for busy_port in ${CAMERA_VISION_BUSY_PORTS:-}; do
    [[ "$busy_port" == "$port" ]] && return 0
  done
  fuser "$port" >/dev/null 2>&1
}

show_port_holders() {
  local port="$1"
  if [[ -n "${CAMERA_VISION_BUSY_PORTS:-}" ]]; then
    printf 'Test ports marked busy: %s\n' "$CAMERA_VISION_BUSY_PORTS" >&2
    return 0
  fi
  fuser -v "$port" 2>&1 || true
}

ensure_port_free() {
  local port="$1"
  if port_is_busy "$port"; then
    printf 'Nano serial port is in use by another program: %s\n' "$port" >&2
    show_port_holders "$port" >&2
    printf 'Close Arduino Serial Monitor / serial-monitor and retry; Camera Vision will not take over a busy port.\n' >&2
    return 1
  fi
}

run_with_serial_port() {
  local port="$1"
  if ensure_port_free "$port"; then
    run --serial-port "$port" "${args[@]}"
  fi
  return 1
}

find_nano() {
  mapfile -t ports < <("$python_bin" - <<'PY'
import os
from pathlib import Path

markers = ("arduino", "ch340", "ch341", "cp210", "usb serial", "ftdi")
dev_dir = Path(os.environ.get("CAMERA_VISION_DEV_DIR", "/dev"))
sys_tty_dir = Path(os.environ.get("CAMERA_VISION_SYS_TTY_DIR", "/sys/class/tty"))
busy_ports = set(os.environ.get("CAMERA_VISION_BUSY_PORTS", "").split())


def read_text(path: Path) -> str:
    try:
        return path.read_text(errors="ignore").strip().lower()
    except OSError:
        return ""


def metadata_for(name: str) -> str:
    parts: list[str] = []
    node = sys_tty_dir / name / "device"
    try:
        current = node.resolve()
    except OSError:
        current = node
    for parent in [current, *current.parents[:6]]:
        for field in ("manufacturer", "product", "interface", "idVendor", "idProduct", "modalias"):
            value = read_text(parent / field)
            if value:
                parts.append(value)
        driver = parent / "driver"
        try:
            if driver.exists():
                parts.append(driver.resolve().name.lower())
        except OSError:
            pass
    return " ".join(parts)

candidates = sorted({*dev_dir.glob("ttyUSB*"), *dev_dir.glob("ttyACM*")})
for device in candidates:
    path = str(device)
    name = device.name
    meta = metadata_for(name)
    confidence = "likely" if any(marker in meta for marker in markers) else "unknown"
    access = "ok" if os.access(path, os.R_OK | os.W_OK) else "blocked"
    busy = "busy" if path in busy_ports else "free"
    print(path + "\t" + confidence + "\t" + access + "\t" + busy)
PY
  )
  local visible=() likely=() blocked=() busy=() record device rest confidence access state
  for record in "${ports[@]}"; do
    device=${record%%$'\t'*}
    rest=${record#*$'\t'}; confidence=${rest%%$'\t'*}
    rest=${rest#*$'\t'}; access=${rest%%$'\t'*}
    state=${record##*$'\t'}
    if [[ "$access" != ok ]]; then
      blocked+=("$device")
    elif [[ "$state" == busy ]] || port_is_busy "$device"; then
      busy+=("$device")
    else
      visible+=("$device"); [[ "$confidence" == likely ]] && likely+=("$device")
    fi
  done
  if [[ ${#likely[@]} -eq 1 ]]; then printf '%s' "${likely[0]}"; return 0; fi
  if [[ ${#visible[@]} -eq 1 ]]; then printf '%s' "${visible[0]}"; return 0; fi
  if [[ ${#visible[@]} -eq 0 ]]; then
    if [[ ${#busy[@]} -gt 0 ]]; then printf 'USB serial ports in use by another program: %s\n' "${busy[*]}" >&2; fi
    if [[ ${#blocked[@]} -gt 0 ]]; then printf 'USB serial ports not accessible to this user: %s\n' "${blocked[*]}" >&2; fi
    if [[ ${#busy[@]} -eq 0 && ${#blocked[@]} -eq 0 ]]; then echo "No USB serial device found." >&2; fi
  else
    printf 'Multiple usable USB serial ports found: %s\n' "${visible[*]}" >&2
    [[ ${#busy[@]} -gt 0 ]] && printf 'Ignored busy USB serial ports: %s\n' "${busy[*]}" >&2
    [[ ${#blocked[@]} -gt 0 ]] && printf 'Ignored inaccessible USB serial ports: %s\n' "${blocked[*]}" >&2
  fi
  return 1
}

case "$launcher_test" in
  find_nano) find_nano; exit $? ;;
  ensure_port_free) ensure_port_free "${args[0]:?ensure_port_free needs a path}"; exit $? ;;
  "") ;;
  *) printf 'Unknown CAMERA_VISION_LAUNCHER_TEST: %s\n' "$launcher_test" >&2; exit 2 ;;
esac

if [[ -n "$manual_port" ]]; then
  echo "Manually selected Nano: $manual_port"
  run_with_serial_port "$manual_port"
fi

# Always ask first: students choose a camera by number, never by device path.
CAMERA_INDEX=""
if choose_camera_index; then
  camera_index="$CAMERA_INDEX"
else
  status=$?
  [[ $status -eq 2 ]] && exit 0
  exit "$status"
fi
args+=(--camera-index "$camera_index")
echo "Selected camera index $camera_index."

while true; do
  cat <<'GUIDE'

=== Ball Balance PID | Launch menu ===
1) Search for and connect Arduino Nano (recommended)
2) Open READY setup without Nano
q) Quit

READY setup: A applies saved settings; B samples the ball HSV; C sets four
platform corners in top-left, top-right, bottom-right, bottom-left order.
Once B and C are complete, press R to RUN. Platform centre is the zero point.
The latest Nano firmware acknowledges valid positions with POS,OK. Older firmware
can use a matching recent TEL,READY,OK for compatibility; updating is recommended.
GUIDE
  read -r -p "Select [1/2/q]: " choice
  case "$choice" in
    1)
      if port=$(find_nano); then
        echo "Found Nano: $port"
        read -r -p "Press Enter for READY setup, r to search again, or b for the menu: " confirm
        [[ "$confirm" == r ]] && continue
        [[ "$confirm" == b ]] && continue
        if ! run_with_serial_port "$port"; then
          echo "Close Arduino Serial Monitor / serial-monitor, then select 1 to retry."
        fi
      else
        echo "Connect Nano and close Arduino Serial Monitor, then select 1 to retry."
      fi
      ;;
    2) echo "Opening READY without Nano. Press n to search; RUN requires Nano."; run --no-serial "${args[@]}" ;;
    q|Q) exit 0 ;;
    *) echo "Enter 1, 2 or q." ;;
  esac
done
