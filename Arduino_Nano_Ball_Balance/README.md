# Arduino Nano：電腦視覺球平衡 PID 韌體

此版本是正式 Camera PID 韌體，由 Desktop 的 `Camera_Vision/ball_vision.py` 以 USB Serial 傳送 OpenCV 偵測後的球座標；Nano 不再讀取 A0/A1，也不提供搖桿 TEST 模式。Nano 收到每筆新 `POS` 資料時才更新雙軸 PID，持續逾 300 ms 未收到資料或收到 `LOST` 時，會清除積分並將兩顆 Servo 回到中立角。

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
3. 先用 `../Arduino_Nano_Ball_Balance_Joystick_Modes/` 的搖桿校正韌體讓平台空載找中心與端點；依 `SETC` / `SHOW` 與學習單結果，回填本正式 PID 檔開頭「學生校正參數（校正完成後只修改本區）」中的 `SERVO_X_CENTER`、`SERVO_Y_CENTER`、`SERVO_LIMIT_OFFSET_DEG`；`SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE` 會由 X 中心 ± offset 推導。此為 Servo 實體安全端點，和 `MAX_PLATFORM_TILT_X_DEG` / `MAX_PLATFORM_TILT_Y_DEG` 的平台最大要求傾角是兩件事。
4. 以小角度測試每軸，若平台修正方向相反，將對應 `SERVO_*_DIRECTION` 改為 `-1`。
5. 完成 Desktop 攝影機的四角校正與球色取樣後，才從 Desktop 視窗按 `r` 開始 PID。

Arduino IDE Serial Monitor 會獨占 Nano serial port；校正時可用它輸入 `SETC` / `SHOW`，但啟動 Camera Vision 前必須關閉 Serial Monitor 或任何其他 serial 工具。

## 通訊協定

Desktop 會送出：

```text
POS,12.4,-8.7,12345
LOST
RUN
READY
TARGET,0.0,0.0
```

Nano 每秒回傳 10 筆 `TEL,...` 資料，包含目前座標、normalized error 百分比、normalized output 百分比、要求平台傾角、Servo 角度與連線／球遺失狀態。格式為：

```text
TEL,state,link,x_mm,y_mm,e_x_pct,e_y_pct,u_x_pct,u_y_pct,tilt_x_deg,tilt_y_deg,servo_x,servo_y,age_ms
```

Nano 韌體是唯一 PID 參數來源與控制權威；Camera Vision 不會保存、下發或覆寫 PID。
成功接受合法 `POS` 時，正式韌體會立即回 `POS,OK`。Camera Vision 會用這個 ACK 作為 RUN 前的 freshness gate，接著再送最後一筆 fresh `POS` 與 `RUN`，並等待 `STATE,RUN` 後才開始連續傳送座標。
目前正式 `.ino` 使用新的未實機驗證控制模型：先以 `POSITION_LIMIT_X_MM=260.0`、`POSITION_LIMIT_Y_MM=200.0` 正規化座標誤差，
`e_norm = clamp((target_mm - ball_mm) / POSITION_LIMIT_AXIS_MM, -1, +1)`；再於無單位 normalized space 計算
`u_norm = Kp*e_norm + Ki*integral(e_norm*dt) + Kd*derivative(e_norm)`，並 clamp 到 `[-1,+1]`；最後 `tilt_deg = u_norm * MAX_PLATFORM_TILT_AXIS_DEG`。
P-only 常數為 X/Y `Kp=1.00`、`Ki=0.00`、`Kd=0.00`，平台最大要求傾角為 X/Y `MAX_PLATFORM_TILT_*=8.0` 度。
含義：球在該軸 full-scale error 時，`Kp=1.0` 要求 100% 最大平台傾角；若 `Kp=0.5` 則只要求 50%。
若要讓任何 PID 或 max tilt 變更生效，必須修改並重新燒錄 `.ino`。舊 `PIDX`/`PIDY` 命令會被明確拒絕並回 `ERROR,PID_MANAGED_BY_NANO`。
