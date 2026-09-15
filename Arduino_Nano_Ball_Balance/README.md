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
3. 先用 `../Arduino_Nano_Ball_Balance_Joystick_Modes/` 的搖桿校正韌體讓平台空載找中心與端點；依 `SETC` / `SHOW` 與學習單結果，回填本正式 PID 檔開頭「學生校正參數（校正完成後只修改本區）」中的 `SERVO_X_CENTER`、`SERVO_Y_CENTER`、`SERVO_LIMIT_OFFSET_DEG`。正式 PID 會分別推導 `SERVO_X_MIN/MAX_ANGLE` 與 `SERVO_Y_MIN/MAX_ANGLE`，避免 Y 軸誤用 X 軸端點。`SERVO_LIMIT_OFFSET_DEG=20` 是校正安全端點，不是實體硬停點。
4. 以小角度測試每軸，若平台修正方向相反，將對應 `SERVO_*_DIRECTION` 改為 `-1`。
5. 完成 Desktop 攝影機的四角校正與球色取樣後，才從 Desktop 視窗按 `r` 開始 PID。

Arduino IDE Serial Monitor 會獨占 Nano serial port；校正時可用它輸入 `SETC` / `SHOW`，但啟動 Camera Vision 前必須關閉 Serial Monitor 或任何其他 serial 工具。

## 通訊協定

Desktop 會送出：

```text
GEOM,-290.0,230.0,-180.0,220.0
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

Nano 韌體是唯一 PID 參數來源與控制權威；Camera Vision 不會保存、下發或覆寫 PID。Camera Vision 只會在 RUN 前依目前 `C` 四角 homography 與 `D` 零點送一次 `GEOM,x_min,x_max,y_min,y_max`，Nano 驗證零點落在四邊內後回 `GEOM,OK`。若 Nano 回 `ERROR,BAD_GEOM` / `ERROR,GEOM_REQUIRED`，或舊韌體回 `ERROR,UNKNOWN_COMMAND`，Camera Vision 會留在 READY；沒有使用舊 ±260/±200 的 fallback。
成功接受合法 `POS` 時，正式韌體會立即回 `POS,OK`。Camera Vision 的 RUN handshake 是 `GEOM -> GEOM,OK -> fresh POS -> POS,OK -> final fresh POS + RUN -> STATE,RUN`，之後才開始連續傳送座標。
正式 `.ino` 以 Camera 傳入的校正邊界正規化座標誤差。對每軸：`error_mm = target_mm - ball_mm`；若 `error_mm >= 0`，分母為 `target_mm - min_mm`；若 `error_mm < 0`，分母為 `max_mm - target_mm`；`e_norm = clamp(error_mm / denominator, -1, +1)`。因此零點偏離中心時，近邊與遠邊會各自映射到該方向的 full-scale error，而不是共用對稱分母。
PID 仍在 Nano 內以 `u_norm = Kp*e_norm + Ki*integral(e_norm*dt) + Kd*derivative(e_norm)` 計算並 clamp 到 `[-1,+1]`；`tilt_deg = u_norm * 20`，再依 `SERVO_*_DIRECTION` 寫到各軸中心 ±20° 的校正安全端點。P-only 常數為 X/Y `Kp=1.00`、`Ki=0.00`、`Kd=0.00`。
若要讓任何 PID、方向或安全端點變更生效，必須修改並重新燒錄 `.ino`。舊 `PIDX`/`PIDY` 命令會被明確拒絕並回 `ERROR,PID_MANAGED_BY_NANO`。
