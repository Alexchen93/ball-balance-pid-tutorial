#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")/.."

launcher=./start_camera_vision.sh
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/dev" "$tmp/sys/class/tty/ttyUSB0/device"
touch "$tmp/dev/ttyUSB0"
printf 'USB Serial CH340\n' > "$tmp/sys/class/tty/ttyUSB0/device/product"

found=$(CAMERA_VISION_DEV_DIR="$tmp/dev" CAMERA_VISION_SYS_TTY_DIR="$tmp/sys/class/tty" CAMERA_VISION_LAUNCHER_TEST=find_nano "$launcher")
[[ "$found" == "$tmp/dev/ttyUSB0" ]]

if CAMERA_VISION_DEV_DIR="$tmp/dev" CAMERA_VISION_SYS_TTY_DIR="$tmp/sys/class/tty" CAMERA_VISION_BUSY_PORTS="$tmp/dev/ttyUSB0" CAMERA_VISION_LAUNCHER_TEST=find_nano "$launcher" >/tmp/camera-vision-find.out 2>/tmp/camera-vision-find.err; then
  echo "busy mocked port should not be selected" >&2
  exit 1
fi
grep -q '正在被其他程式使用' /tmp/camera-vision-find.err

if CAMERA_VISION_BUSY_PORTS="$tmp/dev/ttyUSB0" CAMERA_VISION_LAUNCHER_TEST=ensure_port_free "$launcher" "$tmp/dev/ttyUSB0" >/tmp/camera-vision-busy.out 2>/tmp/camera-vision-busy.err; then
  echo "ensure_port_free should reject a busy mocked port" >&2
  exit 1
fi
grep -q 'Camera Vision 不會接手忙碌的 port' /tmp/camera-vision-busy.err

if grep -q 'serial.tools.list_ports' "$launcher"; then
  echo "launcher detector must not import pyserial list_ports" >&2
  exit 1
fi
