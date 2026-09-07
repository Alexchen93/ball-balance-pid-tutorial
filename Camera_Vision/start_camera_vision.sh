#!/usr/bin/env bash
# Desktop launcher for the camera vision application.

set -Eeuo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python_bin="$script_dir/.venv/bin/python"

if [[ ! -x "$python_bin" ]]; then
  echo "找不到 Python 環境：$python_bin" >&2
  echo "請先依 Camera_Vision/README.md 安裝相依套件。" >&2
  exit 1
fi

serial_port=$("$python_bin" - <<'PY'
from serial.tools import list_ports

markers = ("arduino", "ch340", "cp210", "usb serial", "ftdi")
candidates = [
    port.device
    for port in list_ports.comports()
    if any(marker in " ".join((port.description, port.manufacturer or "")).lower() for marker in markers)
]
print(candidates[0] if len(candidates) == 1 else "")
PY
)

cd "$script_dir"
if [[ -n "$serial_port" ]]; then
  echo "已偵測到 Nano：$serial_port"
  exec "$python_bin" ball_vision.py --serial-port "$serial_port"
fi

echo "未偵測到 Nano，已改為安全的純攝影機預覽模式。"
exec "$python_bin" ball_vision.py --no-serial
