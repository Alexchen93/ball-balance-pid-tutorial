# Desktop 攝影機：OpenCV 球體定位與 Nano PID 通訊

`ball_vision.py` 從固定在平台正上方的 Desktop USB 攝影機取得影像，以 HSV 顏色追蹤球體；將球心透過四角透視校正轉為平台毫米座標，再以 USB Serial 傳送給 Arduino Nano。PID 參數與 Servo 安全保護全部在 Nano 韌體內執行；Camera Vision 不保存、下發或覆寫 PID。

## 一次性安裝

```bash
cd ~/文件/平衡球PID教案/Camera_Vision
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

## 執行

在此資料夾執行 `./start_camera_vision.sh` 即可啟動。啟動後一律先停在 READY checklist；按 `A` 可套用既有 `camera_config.json`，依序完成 `B`、`C`、`D` 後才可按 `R` 進入 RUN。

接上已燒錄 `../Arduino_Nano_Ball_Balance/Arduino_Nano_Ball_Balance.ino` 的 Nano 後，先確認 serial 埠：

```bash
python3 -m serial.tools.list_ports -v
.venv/bin/python ball_vision.py --serial-port /dev/ttyUSB0
```

若埠是 `/dev/ttyACM0`，改用該路徑。程式會在 Desktop 顯示 READY checklist；READY 中不送 `POS`、`LOST` 或 `TARGET`，也不輪詢或顯示 Servo telemetry。離開程式時會先送 `READY`，而 Nano 本身也會在 300 ms 未收到新座標時回中立角。

只檢查鏡頭、不連 Nano：

```bash
.venv/bin/python ball_vision.py --no-serial
```

## 視窗操作

| 操作 | 功能 |
| --- | --- |
| `a` | 套用上一版 `camera_config.json`，回到 READY checklist |
| `b` | 只點球中心，從該中心像素提取 HSV 並儲存 |
| `c` | 重新校正平台；依 TL→TR→BR→BL 點擊四角，會儲存到 `camera_config.json` |
| `d` | 只點平衡終點；該平台位置會成為 `(0, 0)` |
| `r` | 三項 checklist 完成、沒有進行中步驟且 Nano 已連線時，從 READY 進入 RUN；RUN 中再按一次回 READY |
| `q` / `Esc` | 安全停止並離開 |

初次使用請讓球與背景有明顯顏色差異（黃、紅、藍球都適合），完成 `b` 取樣後確認綠色圓圈穩定包住球。再完成 `c` 四角校正與 `d` 零點；平台尺寸可在 `camera_config.json` 的 `width_mm`／`height_mm` 改成實際值。

## 安全順序

1. 固定鏡頭，讓整個平台及四角完整入鏡；校正完不可移動鏡頭或平台。
2. 先以 `--no-serial` 完成或確認 READY checklist，再接 Nano。
3. Servo 電源必須是獨立 5–6 V，並和 Nano 共地。空載時確認兩軸中心角、方向與安全限幅。
4. 先按 `r` 只測一個短暫、小幅的中心修正；若方向相反，立即按 `r` 停止並調整 Nano 的 `SERVO_*_DIRECTION`。
5. 以 Nano 韌體目前的 normalized P-only 常數先做單軸、後雙軸測試。任何 PID 或 max tilt 變更都必須修改並重新燒錄 `.ino`；重啟 Camera Vision 不會改變 Nano PID。

## 通訊與限制

- 只有 RUN 中 PC 才會傳送 `POS,x_mm,y_mm,timestamp_ms`；找不到球時才會送 `LOST`。
- Camera Vision 只管理 HSV、平台四角、零點與影像座標傳輸；舊 `camera_config.json` 的 `pid` 欄位會被視為 deprecated 並忽略。
- `PIDX`/`PIDY` 不再是控制協定；新版 Nano 收到後會回 `ERROR,PID_MANAGED_BY_NANO`。
- 最新正式 Nano 韌體成功接受合法 `POS` 時會回單行 `POS,OK`。若已燒錄的舊版韌體沒有 `POS,OK`，Python 在啟動 RUN 的初始 ACK 階段會相容接受座標相符且新鮮的 `TEL,READY,OK`，再送最後一筆 fresh `POS` 緊接 `RUN`；正式使用仍建議重燒最新版 `.ino`，讓協議一致。
- 正式 Nano 韌體接受 X ±260 mm、Y ±200 mm 內的位置；需要更大平台時，同步調整 `Arduino_Nano_Ball_Balance.ino` 的 `POSITION_LIMIT_X_MM` / `POSITION_LIMIT_Y_MM`。
- 正式 Nano 韌體目前是未實機驗證的 normalized P-only 控制模型；Nano 內建起始值只作低風險驗證，不能保證已能平衡。
