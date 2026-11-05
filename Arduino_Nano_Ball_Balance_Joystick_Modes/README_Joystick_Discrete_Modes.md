# Arduino Nano 球平衡平台：搖桿離散硬體校正模式

此版本在原本 Camera/OpenCV/PID 流程外，提供 5-pin 雙軸搖桿的安全機構校正模式。完整專案操作請見根目錄 [`README.md`](../README.md)。

## 接線

| Nano | 搖桿 | 功能 |
| --- | --- | --- |
| A0 | VRx | X 軸類比輸入 |
| A1 | VRy | Y 軸類比輸入 |
| D7 | SW | 按鈕，使用 `INPUT_PULLUP`；以本程式的 `JOYSTICK_SW_PIN` 為準 |
| 5V | VCC / +5V | 搖桿供電 |
| GND | GND | 共地 |

Servo X 使用 D9、Servo Y 使用 D10。Servo 仍須由獨立 5–6 V 電源供應，並與 Nano 共地。

## 開機與 TEST 模式

目前版本會在開機後**自動進入 TEST 模式**，不需要先長按搖桿按鈕。進入後預設為 MODE1。每次程式啟動會先在 Serial Monitor 輸出一行 `RESET/BOOT`，接著輸出 MODE、X、Y 三行狀態。按板子 Reset、重新上電或其他重新啟動都可能看到此提示；它不區分重啟原因。請在開機／重設時放開搖桿，讓程式取得正確的中立值。Serial Monitor 若在提示送出後才連線，可能看不到該次啟動訊息；可在視窗開啟時按板子的 Reset。

每一次推桿只接受一個軸的一次動作：

- 搖桿必須回到中立區才會重新 armed。
- 動作間至少保留約 350 ms 的安全間隔。
- 斜推時只接受偏移較大的那個軸，因此不會同時改動 X、Y。
- 持續把搖桿推在同一方向不會連續累加。

## Mode 與按鈕操作

| Mode | 每次搖桿動作 |
| --- | --- |
| MODE1 | 選定軸 ±1° |
| MODE2 | 正方向前往共用 MAX；負方向前往共用 MIN，可能產生大幅動作 |

- **短按 SW**：在 MODE1 / MODE2 之間切換。
- **在 TEST 長按 SW 1.5 秒**：X、Y 回到 `testCenterX` / `testCenterY`。

目前程式設為 X 中心 77°、Y 中心 87°、offset 20°。**校正韌體目前兩軸共用 X 中心推得的 57°～97° 限幅**，不是各自中心 ±20°；正式 PID 韌體則依各軸中心各自計算。這些數值不保證符合實際機構，尤其要先確認 Y 軸安全範圍；方向檢查只用 MODE1 小步，不用 MODE2。

## 建議校正流程

1. 平台空載，確認 Servo 有獨立供電與共地。
2. MODE1 以 1° 單步找平台水平。
3. 確認兩軸共用限幅適合各軸機構，若不適合，先修正與測試限幅；不得把 MODE2 視為自動安全。以空載小幅度、逐軸確認無卡住或超限。
4. 在 115200 baud、Newline 的 Serial Monitor 輸入 `SETC`，再輸入 `SHOW`。
5. 將 `SHOW` 印出的 `SERVO_X_CENTER` / `SERVO_Y_CENTER` / `SERVO_LIMIT_OFFSET_DEG` 寫回 `.ino` 頂端的常數並重新燒錄，才會永久保存。
6. 完成機構校正後，才進行攝影機校正與 PID 控制。

## 看搖桿 ADC 及 Servo 增減

開機或執行 `JOYCAL` 時，先放開搖桿並維持中立；程式會以取樣值作為兩軸中心。Serial Monitor 設為 **115200 baud、Newline**。一次有效推桿固定輸出三行 `MODE`、`X`、`Y`，例如：

```text
MODE,NAME=MODE1,ACTION=MICRO_STEP_1DEG,ARMED=0,SOURCE=JOYSTICK
X,ANGLE=78,CENTER=77,LIMIT=57-97,PIN=9,TEST_FW_PID_DIR=-1,JOY_ADC=760,JOY_DIR=INCREASE,SERVO_CHANGE=INCREASE
Y,ANGLE=87,CENTER=87,LIMIT=57-97,PIN=10,TEST_FW_PID_DIR=-1,JOY_ADC=510,JOY_DIR=CENTER,SERVO_CHANGE=NONE
```

放開搖桿回中立不會再額外輸出；下一次有效推桿才印出下一組。`ARMED` 等資訊反映輸出當下的狀態，不會因回中立而在舊文字上更新。手動指令 `JOY`、`DIR`、`SHOW`、`HELP` 仍會輸出各自的查詢或結果。

- `JOY_ADC` 是 Nano 將搖桿輸出**電壓**轉成的 0～1023 數字，不是直接量到的電阻值或歐姆數。`JOY_DIR=INCREASE`／`DECREASE` 表示目前 ADC 比開機時取樣的中立值高／低，並超出死區；`CENTER` 表示在死區內。這是相對中立的方向，不是相對上一筆取樣的變化速度。
- `SERVO_CHANGE=INCREASE`／`DECREASE` 是本次命令角度的增減，`NO_CHANGE_LIMIT` 表示此次動作命令的角度沒有改變，`NONE` 表示這列未記錄單步增減（例如單步操作中另一軸未動，或一般狀態查詢）。`SOURCE` 說明本次單步來自搖桿或 Serial 指令。Servo 沒有回授角度感測，命令角度無法證明機構實際移動了多少。
- `TEST_FW_PID_DIR` 是**校正程式內**舊控制路徑的固定值，不是正式 PID 韌體目前的設定，也不是測出的建議方向；正式 PID 方向應依下方步驟透過 `DIR` 和實際平台升降判斷。
- `JOYSTICK_X_DIRECTION/Y_DIRECTION` 只決定搖桿 ADC 如何對應校正模式的 Servo 增減；它們不會決定或修改正式 PID 的 `SERVO_X_DIRECTION/Y_DIRECTION`。數值反向不一定是裝反。

## 判斷正式 PID 的方向

此方法依**學生親眼觀察的實際升降**提供建議，程式無法從 ADC 自行知道平台哪一側升高。預設與正式程式一致：攝影機校正後 X 正為右側，Y 正為遠端（TL/TR 那一側）；正式 PID 在球靜止於正側、目標為中心時，應讓正側升高。先確認 X Servo 確實控制左右軸、Y Servo 確實控制前後軸；若軸交叉或相反，先檢查接線、機構和攝影機四角，不可僅改正負號。

1. 球移開、平台空載，確認電源、共地、機構與兩軸中立及安全行程。切換程式及接線操作時依實際設備安全流程處理；按 `CENTER` 回已設定的測試中心，確認平台水平。選 `MODE,1`。
2. **X 軸：** 用搖桿單步，或輸入 `X+`，直到輸出 `SERVO_CHANGE=INCREASE`，從中心增加 1～3°。每次推桿須回中立才重新觸發；注意搖桿推向右邊不一定代表 Servo 命令增加。觀察平台**右側**相對中央是升高還是降低。
3. 右側升高時輸入 `DIR,X,UP`；右側降低時輸入 `DIR,X,DOWN`。若看不出變化、卡住、移動的是另一軸、或方向不確定，先停止檢查，不要選 UP 或 DOWN。若只是抖動、沒有實際移動，即使命令角度有變化也不能判定。
4. 按 `CENTER` 回中立，再確認 `MODE,1`。**Y 軸：** 讓 Y 的 Servo 命令從中心增加 1～3°（例如 `Y+`），觀察平台**遠端**升降；升高輸入 `DIR,Y,UP`，降低輸入 `DIR,Y,DOWN`。
5. 輸入 `SHOW`，可看到已觀察軸的建議 `SERVO_*_DIRECTION`；未完成會顯示 `NOT TESTED`。建議值只存在 Nano 記憶體；依各組實際結果抄到正式 PID `.ino` 並重新燒錄，並先低幅、單軸確認回拉方向。程式不會自動改正式韌體。

觀察與建議的對應：

| Servo 命令角度增加時，該軸正側 | 建議的正式 PID 方向 |
| --- | --- |
| 上升（`UP`） | `-1` |
| 下降（`DOWN`） | `1` |

原因是靜止的球位於正側、目標為中心時，正式 PID 的誤差為負；若要以 Servo 角度增加來抬高正側，就需要方向係數 `-1`。此判斷假設 Servo 角度變化確實帶動相應平台軸，且攝影機座標與方向如上，並非一般搖桿接線都能自動推得的定值。

## 常用 Serial 指令

- `STATUS`：固定印出三行狀態：`MODE`、`X`、`Y`；可直接判讀模式、目前角度、中心、限制範圍、接腳與方向。
- `SHOW`：印出可複製回程式的校正常數。
- `SETC` / `SETCX` / `SETCY`：設定測試中心。
- `CENTER`：回到測試中心。
- `MODE,1` / `MODE,2`：選擇 1° 微調或端點模式。
- `STEP,1`：選擇 1° 微調。
- `X+`、`X-`、`Y+`、`Y-`：手動移動指定軸。
- `X,角度`、`Y,角度`、`XY,X角度,Y角度`：直接指定安全範圍內的角度。
- `JOY`：讀取搖桿 ADC、中心、偏差與 HIGH／LOW／CENTER；`JOYCAL`：重新校正搖桿中立值。
- `DIR,X,UP` / `DIR,X,DOWN` / `DIR,Y,UP` / `DIR,Y,DOWN`：在符合上述小步測試條件後，輸入觀察結果並顯示正式 PID 的方向建議。
- `TESTX`、`TESTY`、`TESTBOTH`：執行內建來回機構測試。

## 可調參數

```cpp
constexpr int JOYSTICK_DEADZONE = 100;
constexpr uint16_t JOYSTICK_ACTION_DELAY_MS = 350;
constexpr uint16_t JOYSTICK_LONG_PRESS_MS = 1500;
```

若中立時會誤觸發，增加 `JOYSTICK_DEADZONE`；若每次操作要更慢，增加 `JOYSTICK_ACTION_DELAY_MS`。
