# Arduino Nano：電腦視覺球平衡 PID 韌體

此版本由 Desktop 的 `Camera_Vision/ball_vision.py` 以 USB Serial 傳送 OpenCV 偵測後的球座標；Nano 不再讀取 A0/A1。Nano 收到每筆新 `POS` 資料時才更新雙軸 PID，持續逾 300 ms 未收到資料或收到 `LOST` 時，會清除積分並將兩顆 Servo 回到中立角。

## 接線

| Nano 腳位 | 用途 |
| --- | --- |
| D9 | Servo X 訊號 |
| D10 | Servo Y 訊號 |
| USB | Desktop 供電與 Serial（115200 baud） |
| GND | Servo 外部電源的共地 |

Servo 必須使用獨立、足夠電流的 5–6 V BEC／DC-DC 供電，且其 GND 必須接到 Nano GND；絕不可由 Nano 5 V 腳供應兩顆 Servo。

## 燒錄與機構校正

1. 安裝 Arduino Library Manager 的 `Servo`。
2. 燒錄 `Arduino_Nano_Ball_Balance.ino` 到 Arduino Nano（115200 baud）。若舊款 Nano 無法燒錄，選擇 `ATmega328P (Old Bootloader)`。
3. 先讓平台空載，校正程式頂端的 `SERVO_X_CENTER`、`SERVO_Y_CENTER`、`SERVO_MIN_ANGLE`、`SERVO_MAX_ANGLE`。
4. 以小角度測試每軸，若平台修正方向相反，將對應 `SERVO_*_DIRECTION` 改為 `-1`。
5. 完成 Desktop 攝影機的四角校正與球色取樣後，才從 Desktop 視窗按 `r` 開始 PID。

## 通訊協定

Desktop 會送出：

```text
POS,12.4,-8.7,12345
LOST
RUN
READY
TARGET,0.0,0.0
PIDX,0.10,0.00,0.25
PIDY,0.10,0.00,0.25
```

Nano 每秒回傳 10 筆 `TEL,...` 資料，包含目前座標、誤差、PID 輸出、Servo 角度與連線／球遺失狀態。

初始 PID 參數只是一個低風險起點。先固定單一軸並以 `Ki=0` 測試小 `Kp`，確認方向正確後再提高 `Kp`、加入少量 `Kd`；若有固定偏差才少量加入 `Ki`。
