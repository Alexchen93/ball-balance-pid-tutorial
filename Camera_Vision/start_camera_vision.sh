#!/usr/bin/env bash
# Interactive launcher: choose and confirm a mode before the camera opens.
set -Eeuo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python_bin="$script_dir/.venv/bin/python"
[[ -x "$python_bin" ]] || { echo "找不到 Python 環境：$python_bin" >&2; exit 1; }

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
    printf '測試標記為忙碌的 Serial port：%s\n' "$CAMERA_VISION_BUSY_PORTS" >&2
    return 0
  fi
  fuser -v "$port" 2>&1 || true
}

ensure_port_free() {
  local port="$1"
  if port_is_busy "$port"; then
    printf 'Nano serial port 正在被其他程式使用：%s\n' "$port" >&2
    show_port_holders "$port" >&2
    printf '請關閉 Arduino Serial Monitor / serial-monitor 後再重試；Camera Vision 不會接手忙碌的 port。\n' >&2
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
    if [[ ${#busy[@]} -gt 0 ]]; then printf 'USB serial 正在被其他程式使用：%s\n' "${busy[*]}" >&2; fi
    if [[ ${#blocked[@]} -gt 0 ]]; then printf 'USB serial 權限不足，Alex 無法讀寫：%s\n' "${blocked[*]}" >&2; fi
    if [[ ${#busy[@]} -eq 0 && ${#blocked[@]} -eq 0 ]]; then echo "未找到 USB serial 裝置。" >&2; fi
  else
    printf '偵測到多個可用 USB serial：%s\n' "${visible[*]}" >&2
    [[ ${#busy[@]} -gt 0 ]] && printf '已忽略忙碌的 USB serial：%s\n' "${busy[*]}" >&2
    [[ ${#blocked[@]} -gt 0 ]] && printf '已忽略權限不足的 USB serial：%s\n' "${blocked[*]}" >&2
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
  echo "手動選擇 Nano：$manual_port"
  run_with_serial_port "$manual_port"
fi

while true; do
  cat <<'GUIDE'

=== 平衡球 PID｜啟動選單 ===
1) 搜尋並連線 Arduino Nano（建議）
2) 不連 Nano 開啟 READY 設定畫面
q) 離開

開啟後一律停在 READY checklist：A 套用設定、B 點球心 HSV、C 依 TL→TR→BR→BL 點平台四角、D 點平衡零點。
只有三項完成且沒有正在進行的步驟時，才可按 R 進 RUN。
新版 Nano 對合法 POS 會回 POS,OK；若已燒錄舊版沒有 POS,OK，Python 會用座標相符且新鮮的 TEL,READY,OK 相容啟動，但正式建議重燒最新版 .ino。
GUIDE
  read -r -p "請選擇 [1/2/q]：" choice
  case "$choice" in
    1)
      if port=$(find_nano); then
        echo "已找到 Nano：$port"
        read -r -p "按 Enter 開啟 READY 設定畫面；輸入 r 重新搜尋；輸入 b 回選單：" confirm
        [[ "$confirm" == r ]] && continue
        [[ "$confirm" == b ]] && continue
        if ! run_with_serial_port "$port"; then
          echo "請關閉 Arduino Serial Monitor / serial-monitor 後，再選 1 重試。"
        fi
      else
        echo "請插上 Nano、關閉 Arduino Serial Monitor 後，再選 1 重試。"
      fi
      ;;
    2) echo "已選不連 Nano 的 READY 設定畫面。畫面中可按 n 搜尋 Nano；RUN 仍需 Nano 連線。"; run --no-serial "${args[@]}" ;;
    q|Q) exit 0 ;;
    *) echo "請輸入 1、2 或 q。" ;;
  esac
done
