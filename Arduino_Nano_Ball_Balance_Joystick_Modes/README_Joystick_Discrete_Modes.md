# Arduino Nano 球平衡平台：搖桿離散硬體校正模式

此版本在原本 Camera/OpenCV/PID 流程外，提供 5-pin 雙軸搖桿的安全機構校正模式。完整專案操作請見根目錄 [`README.md`](../README.md)。

## 接線

| Nano | 搖桿 | 功能 |
| --- | --- | --- |
| A0 | VRx | X 軸類比輸入 |
| A1 | VRy | Y 軸類比輸入 |
| D2 | SW | 按鈕，使用 `INPUT_PULLUP` |
| 5V | VCC / +5V | 搖桿供電 |
| GND | GND | 共地 |

Servo X 使用 D9、Servo Y 使用 D10。Servo 仍須由獨立 5–6 V 電源供應，並與 Nano 共地。

## 開機與 TEST 模式

目前版本會在開機後**自動進入 TEST 模式**，不需要先長按搖桿按鈕。進入後預設為 MODE1。

每一次推桿只接受一個軸的一次動作：

- 搖桿必須回到中立區才會重新 armed。
- 動作間至少保留約 350 ms 的安全間隔。
- 斜推時只接受偏移較大的那個軸，因此不會同時改動 X、Y。
- 持續把搖桿推在同一方向不會連續累加。

## Mode 與按鈕操作

| Mode | 每次搖桿動作 |
| --- | --- |
| MODE1 | 選定軸 ±1° |
| MODE2 | 正方向前往 110°；負方向前往 70° |

- **短按 SW**：在 MODE1 / MODE2 之間切換。
- **在 TEST 長按 SW 1.5 秒**：X、Y 回到 `testCenterX` / `testCenterY`。

預設中心是 90°，端點範圍是 70°～110°；這些數值必須依實際連桿調整。

## 建議校正流程

1. 平台空載，確認 Servo 有獨立供電與共地。
2. MODE1 以 1° 單步找平台水平。
3. MODE2 逐一測試 X+/X-/Y+/Y-，確認 70°～110° 端點不會卡住；若有風險，縮小 `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE`。
4. 在 115200 baud、Newline 的 Serial Monitor 輸入 `SETC`，再輸入 `SHOW`。
5. 將 `SHOW` 印出的 `SERVO_X_CENTER` / `SERVO_Y_CENTER` 寫回 `.ino` 頂端的常數並重新燒錄，才會永久保存。
6. 完成機構校正後，才進行攝影機校正與 PID 控制。

## 常用 Serial 指令

- `STATUS`：顯示 Servo、中心、mode 與 armed 狀態。
- `SHOW`：印出可複製回程式的校正常數。
- `SETC` / `SETCX` / `SETCY`：設定測試中心。
- `CENTER`：回到測試中心。
- `MODE,1` / `MODE,2`：選擇 1° 微調或端點模式。
- `STEP,1`：選擇 1° 微調。
- `X+`、`X-`、`Y+`、`Y-`：手動移動指定軸。
- `X,角度`、`Y,角度`、`XY,X角度,Y角度`：直接指定安全範圍內的角度。
- `JOY`：讀取搖桿 ADC 值；`JOYCAL`：重新校正搖桿中立值。
- `TESTX`、`TESTY`、`TESTBOTH`：執行內建來回機構測試。

## 可調參數

```cpp
constexpr int JOYSTICK_DEADZONE = 100;
constexpr uint16_t JOYSTICK_ACTION_DELAY_MS = 350;
constexpr uint16_t JOYSTICK_LONG_PRESS_MS = 1500;
```

若中立時會誤觸發，增加 `JOYSTICK_DEADZONE`；若每次操作要更慢，增加 `JOYSTICK_ACTION_DELAY_MS`。
