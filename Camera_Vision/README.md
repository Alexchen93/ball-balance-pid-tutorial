# Desktop 攝影機：OpenCV 球體定位與 Nano PID 通訊

`ball_vision.py` 從固定在平台正上方的 Desktop USB 攝影機取得影像，以 HSV 顏色追蹤球體；將球心透過四角透視校正轉為平台毫米座標，再以 USB Serial 傳送給 Arduino Nano。PID 和 Servo 安全保護全部在 Nano 韌體內執行。

## 一次性安裝

```bash
cd ~/文件/平衡球PID教案/Camera_Vision
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

## 執行

在此資料夾執行 `./start_camera_vision.sh` 即可啟動。它會自動偵測唯一的 Nano serial 埠：偵測到時進入可控制模式，未偵測到時則安全地只開啟攝影機預覽。

接上已燒錄 `../Arduino_Nano_Ball_Balance/Arduino_Nano_Ball_Balance.ino` 的 Nano 後，先確認 serial 埠：

```bash
python3 -m serial.tools.list_ports -v
.venv/bin/python ball_vision.py --serial-port /dev/ttyUSB0
```

若埠是 `/dev/ttyACM0`，改用該路徑。程式會在 Desktop 顯示鏡頭預覽與 Nano telemetry；離開程式時會先送 `READY`，而 Nano 本身也會在 300 ms 未收到新座標時回中立角。

只檢查鏡頭、不連 Nano：

```bash
.venv/bin/python ball_vision.py --no-serial
```

## 視窗操作

| 操作 | 功能 |
| --- | --- |
| `c` | 重新校正平台；依序點擊左上、右上、右下、左下四角，會儲存到 `camera_config.json` |
| `b` | 點擊球，從 11×11 像素區域取樣 HSV 顏色並儲存 |
| `r` | PID 的 `READY` / `RUN` 切換；未完成平台四角校正不能啟動 |
| `0` | 將目標重設為平台中心 `(0, 0)` mm |
| 一般左鍵 | 將該位置設為新的平面座標目標 |
| `q` / `Esc` | 安全停止並離開 |

初次使用請讓球與背景有明顯顏色差異（黃、紅、藍球都適合），完成 `b` 取樣後確認綠色圓圈穩定包住球。再完成 `c` 四角校正；平台尺寸可在 `camera_config.json` 的 `width_mm`／`height_mm` 改成實際值。

## 安全順序

1. 固定鏡頭，讓整個平台及四角完整入鏡；校正完不可移動鏡頭或平台。
2. 先以 `--no-serial` 檢查球心、校正座標與 FPS，再接 Nano。
3. Servo 電源必須是獨立 5–6 V，並和 Nano 共地。空載時確認兩軸中心角、方向與安全限幅。
4. 先按 `r` 只測一個短暫、小幅的中心修正；若方向相反，立即按 `r` 停止並調整 Nano 的 `SERVO_*_DIRECTION`。
5. 以 `Ki=0` 從小 `Kp` 開始，先單軸、後雙軸調參。不要直接套用其他機構的 PID 值。

## 通訊與限制

- PC 每秒最多傳送 30 筆 `POS,x_mm,y_mm,timestamp_ms`；找不到球時會送 `LOST`。
- Nano 僅接受 ±150 mm 內的位置；需要更大平台時，同步調整 Nano 的 `POSITION_LIMIT_MM`。
- 相機偵測、平台尺寸與 Servo 機構尚未實測，PID 起始值只作低風險驗證，不能保證已能平衡。
