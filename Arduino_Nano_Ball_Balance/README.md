# Arduino Nano：正式 Camera PID 韌體

這個資料夾是正式控制韌體。Desktop 的 `Camera_Vision/ball_vision.py` 只傳送平台座標與安全狀態；Nano 是 PID、Servo 方向、限幅與失聯保護的唯一控制權威。

> `Arduino_Nano_Ball_Balance_Joystick_Modes/` 是校正韌體，不是正式 Camera PID 燒錄目標。

注意：目前搖桿校正韌體的角度限制由 X 中心計算，X 和 Y 共用同一組 `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE`；目前為 57–97°，與正式韌體 Y 軸的 67–107° 並不相同。校正程式顯示的限制不能直接當成正式 Y 端點安全的證明，需先確認機構安全範圍。

## 接線與安全

| Nano 腳位 | 用途 |
| --- | --- |
| D9 | Servo X 訊號 |
| D10 | Servo Y 訊號 |
| USB | Desktop Serial（115200 baud）與燒錄 |
| GND | 與 Servo 外部電源 GND 共地 |

兩顆 Servo 必須使用獨立、足夠電流的 5–6 V 電源；不得由 Nano 的 5 V 腳供應。

## 燒錄與機構校正

1. Arduino IDE 安裝 `Servo` 函式庫。
2. 先使用搖桿校正韌體找平台水平、Servo 中心與安全端點。
3. 將 `SETC` / `SHOW` 結果填入 `.ino` 頂端的：
   - `SERVO_X_CENTER`
   - `SERVO_Y_CENTER`
   - `SERVO_X_DECREASE_OFFSET_DEG`、`SERVO_X_INCREASE_OFFSET_DEG`
   - `SERVO_Y_DECREASE_OFFSET_DEG`、`SERVO_Y_INCREASE_OFFSET_DEG`
4. 選擇 Arduino Nano 後燒錄；舊 Nano 上傳失敗時選 `ATmega328P (Old Bootloader)`。
5. 先以小角度測試方向。球被推得更遠時，立即停止，修改相對應 `SERVO_X_DIRECTION` 或 `SERVO_Y_DIRECTION` 後重燒。

Arduino Serial Monitor 會佔用 USB serial；使用 Camera Vision 前務必關閉它。

## 控制座標與通訊

Camera Vision 的 `C` 四角將平台映射為百分比座標：

- 左 → 右：`X-` → `X+`
- 平台上方／遠端 TL、TR：`Y+`
- 平台下方／近端 BL、BR：`Y-`
- 平台中心：`(0,0)`

Nano 接收：

```text
GEOM,-100.00,100.00,-100.00,100.00
POS,12.4,-8.7,12345
LOST
RUN
READY
```

安全握手為：

```text
GEOM → GEOM,OK → fresh POS → POS,OK → final fresh POS + RUN → STATE,RUN
```

位置範圍由 `GEOM` 提供。Nano 只接受有界、有限且新鮮的 `POS`；超出範圍會回 `ERROR,POSITION_OUT_OF_RANGE`，找不到球或資料逾時會停止控制並回中立。

正式韌體由 RUN 停止時，會先輸出 `STOP_REASON`，再輸出 `STATE,READY`：

- `STOP_REASON,BALL`：攝影機程式送出 `LOST`，表示球當時無法辨識或不在校正範圍內。
- `STOP_REASON,LOST`：Nano 超過約 300 ms 未收到新的球位置。
- `STOP_REASON,RANGE`：Nano 收到超出幾何範圍的座標。
- `STOP_REASON,GEOM`：平台幾何資料無效或未設定。
- `STOP_REASON,OK`／`WAIT`：停止時位置仍新鮮／未連線，常見於手動送出 `READY`。

僅有 `STATE,READY` 無法區分這些原因。新輸出須重新燒錄本正式韌體；若 Nano 重開機或 USB 在訊息送出前已中斷，原因仍可能無法送達。

Telemetry 每秒約 10 次：

```text
TEL,state,link,x_pct,y_pct,e_x_pct,e_y_pct,u_x_pct,u_y_pct,tilt_x_deg,tilt_y_deg,servo_x,servo_y,age_ms
```

學生觀察邊緣回拉時，特別看：

- `ball`：球的位置。
- `error`：目標中心與球的正規化誤差。
- `output`：PID 輸出百分比。
- `tilt`：PID 要求的 Servo 相對中心偏移量（方向係數套用前），並非量測的平台傾角。
- `servo`：寫給 Servo 的目標角度，並非馬達實際角度的回授量測。

若球接近邊緣但 `tilt` 已接近 ±20°，代表可能是 Servo／機構回拉力或影像延遲，而不只是 PID 參數。

## 目前 PD 起始參數

```cpp
DEFAULT_KP_X = 1.25f;
DEFAULT_KI_X = 0.00f;
DEFAULT_KD_X = 0.50f;
DEFAULT_KP_Y = 1.25f;
DEFAULT_KI_Y = 0.00f;
DEFAULT_KD_Y = 0.60f; // 小幅測試值；原值 0.50f
```

以上為目前程式中的設定，並非已經完成實機調整或適用所有平台的最佳值。I 的運算已有實作，但 Ki 為 0，所以目前是 PD；每組完成校正與測試後才決定是否調整。

目前機構參數為 X 中心 77°、Y 中心 87°、兩軸安全行程各 ±20°、X 方向為 −1、Y 方向為 −1。X、Y 的 `SERVO_*_DECREASE_OFFSET_DEG` 與 `SERVO_*_INCREASE_OFFSET_DEG` 可分別調整：前者是 Servo 命令角度低於中心時的範圍，後者是高於中心時的範圍。它們並不直接表示平台哪一側上升或下降，需依實際連桿動作判斷。中心與安全端點必須依各組的實際校正結果確認，不能直接視為水平或安全。

X、Y 輸出滿幅會對應其 Servo 命令方向的設定行程；例如某側偏移從 20 改為 30，該側 50% 輸出也會從 10° 變成 15°。這會改變該側的控制靈敏度，不只是修改飽和端點；修改前必須先確認連桿全程安全，再重新燒錄並重新測試。預設四個偏移值均為 20，未擴大現有行程。

公式（誤差先依平台範圍正規化，時間以秒計）：

```text
error = target - ball
u = Kp × error + Ki × integral(error) + Kd × derivative(error)
servo_offset = clamp(u, -1, +1) × 對應軸及所命令 Servo 方向的偏移設定
```

- `Kp` 提高可讓球還未到邊緣時更早回拉。
- `Kd` 在球快速外移時提供提早煞車；過大會放大影像雜訊、造成抖動。
- `Ki` 現在為 0，因為它無法煞住移動中的球，且容易造成過衝。

`servo_offset` 是 Servo 命令角度相對中心的偏移，不等於實際平台傾角。Y 的 `Kd` 從 0.50 調至 0.60 是待實機驗證的測試值，增加的是對位置變化速度的反應；若已輸出飽和則不能超過現有角度限制，且較高的 `Kd` 可能放大辨識雜訊。若高頻抖動或更不穩定，停止並恢復 0.50；若回拉太慢且沒有明顯抖動，可考慮小幅提高 `Kp`。每次只改一個參數、重新燒錄並記錄結果。不要把 Servo 安全端點當成 PID 調參工具。

## Camera Vision 的責任邊界

Camera Vision 負責 HSV、平台四角與球接觸點估計。它不保存或下發 PID；`PIDX` / `PIDY` 命令會被 Nano 拒絕為 `ERROR,PID_MANAGED_BY_NANO`。任何 PID、Servo 中心、方向或安全端點變更，都必須修改此 `.ino` 後重新燒錄。
