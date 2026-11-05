# Arduino Nano：正式 Camera PID 韌體

這個資料夾是正式控制韌體。Desktop 的 `Camera_Vision/ball_vision.py` 只傳送平台座標與安全狀態；Nano 是 PID、Servo 方向、限幅與失聯保護的唯一控制權威。

> `Arduino_Nano_Ball_Balance_Joystick_Modes/` 是校正韌體，不是正式 Camera PID 燒錄目標。

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
   - `SERVO_LIMIT_OFFSET_DEG`
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

Telemetry 每秒約 10 次：

```text
TEL,state,link,x_pct,y_pct,e_x_pct,e_y_pct,u_x_pct,u_y_pct,tilt_x_deg,tilt_y_deg,servo_x,servo_y,age_ms
```

學生觀察邊緣回拉時，特別看：

- `ball`：球的位置。
- `error`：目標中心與球的正規化誤差。
- `output`：PID 輸出百分比。
- `tilt`：要求的平台傾斜角。
- `servo`：實際寫出的 Servo 角度。

若球接近邊緣但 `tilt` 已接近 ±20°，代表可能是 Servo／機構回拉力或影像延遲，而不只是 PID 參數。

## 目前 PD 起始參數

```cpp
DEFAULT_KP_X = 1.25f;
DEFAULT_KI_X = 0.00f;
DEFAULT_KD_X = 0.50f;
DEFAULT_KP_Y = 1.25f;
DEFAULT_KI_Y = 0.00f;
DEFAULT_KD_Y = 0.50f;
```

以上為目前程式中的設定，並非已經完成實機調整或適用所有平台的最佳值。I 的運算已有實作，但 Ki 為 0，所以目前是 PD；每組完成校正與測試後才決定是否調整。

目前機構參數為 X 中心 77°、Y 中心 87°、兩軸安全行程各 ±20°、X 方向為 −1、Y 方向為 −1。中心與安全端點必須依各組的實際校正結果確認，不能直接視為水平或安全。

公式（誤差先依平台範圍正規化，時間以秒計）：

```text
error = target - ball
u = Kp × error + Ki × integral(error) + Kd × derivative(error)
servo_offset = clamp(u, -1, +1) × SERVO_LIMIT_OFFSET_DEG
```

- `Kp` 提高可讓球還未到邊緣時更早回拉。
- `Kd` 在球快速外移時提供提早煞車；過大會放大影像雜訊、造成抖動。
- `Ki` 現在為 0，因為它無法煞住移動中的球，且容易造成過衝。

`servo_offset` 是 Servo 命令角度相對中心的偏移，不等於實際平台傾角。若高頻抖動，先停止並檢查辨識穩定性與機構，再考慮小幅降低 `Kd`；若回拉太慢且沒有明顯抖動，可考慮小幅提高 `Kp`。每次只改一個參數、重新燒錄並記錄結果。不要把 Servo 安全端點當成 PID 調參工具。

## Camera Vision 的責任邊界

Camera Vision 負責 HSV、平台四角與球接觸點估計。它不保存或下發 PID；`PIDX` / `PIDY` 命令會被 Nano 拒絕為 `ERROR,PID_MANAGED_BY_NANO`。任何 PID、Servo 中心、方向或安全端點變更，都必須修改此 `.ino` 後重新燒錄。
