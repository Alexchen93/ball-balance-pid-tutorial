#!/usr/bin/env bash
# Interactive launcher: choose and confirm a mode before the camera opens.
set -Eeuo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python_bin="$script_dir/.venv/bin/python"
[[ -x "$python_bin" ]] || { echo "找不到 Python 環境：$python_bin" >&2; exit 1; }

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

find_nano() {
  mapfile -t ports < <("$python_bin" - <<'PY'
from pathlib import Path
from serial.tools import list_ports
markers=("arduino","ch340","cp210","usb serial","ftdi")
meta={p.device:f"{p.description or ''} {p.manufacturer or ''}".lower() for p in list_ports.comports()}
all_ports={p for p in meta if p.startswith(("/dev/ttyUSB","/dev/ttyACM"))}
all_ports.update(str(p) for p in Path("/dev").glob("ttyUSB*")); all_ports.update(str(p) for p in Path("/dev").glob("ttyACM*"))
for p in sorted(all_ports): print(p + "\t" + ("likely" if any(m in meta.get(p,"") for m in markers) else "unknown"))
PY
)
  local visible=() likely=() record device confidence
  for record in "${ports[@]}"; do
    device=${record%%$'\t'*}; confidence=${record#*$'\t'}
    visible+=("$device"); [[ "$confidence" == likely ]] && likely+=("$device")
  done
  if [[ ${#likely[@]} -eq 1 ]]; then printf '%s' "${likely[0]}"; return 0; fi
  if [[ ${#visible[@]} -eq 1 ]]; then printf '%s' "${visible[0]}"; return 0; fi
  if [[ ${#visible[@]} -eq 0 ]]; then echo "未找到 USB serial 裝置。" >&2; else printf '偵測到多個 USB serial：%s\n' "${visible[*]}" >&2; fi
  return 1
}

if [[ -n "$manual_port" ]]; then
  echo "手動選擇 Nano：$manual_port"
  run --serial-port "$manual_port" "${args[@]}"
fi

while true; do
  cat <<'GUIDE'

=== 平衡球 PID｜啟動選單 ===
1) 搜尋並連線 Arduino Nano（建議）
2) 不連 Nano 開啟 READY 設定畫面
q) 離開

開啟後一律停在 READY checklist：A 套用設定、B 點球心 HSV、C 依 TL→TR→BR→BL 點平台四角、D 點平衡零點。
只有三項完成且沒有正在進行的步驟時，才可按 R 進 RUN。
GUIDE
  read -r -p "請選擇 [1/2/q]：" choice
  case "$choice" in
    1)
      if port=$(find_nano); then
        echo "已找到 Nano：$port"
        read -r -p "按 Enter 開啟 READY 設定畫面；輸入 r 重新搜尋；輸入 b 回選單：" confirm
        [[ "$confirm" == r ]] && continue
        [[ "$confirm" == b ]] && continue
        run --serial-port "$port" "${args[@]}"
      else
        echo "請插上 Nano、關閉 Arduino Serial Monitor 後，再選 1 重試。"
      fi
      ;;
    2) echo "已選不連 Nano 的 READY 設定畫面。畫面中可按 n 搜尋 Nano；RUN 仍需 Nano 連線。"; run --no-serial "${args[@]}" ;;
    q|Q) exit 0 ;;
    *) echo "請輸入 1、2 或 q。" ;;
  esac
done
