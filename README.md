# Arduino Nano 電腦視覺球平衡控制

以固定在平台上方的 USB 攝影機辨識球的位置，Desktop 上的 Python / OpenCV 將座標傳給 Arduino Nano；Nano 執行雙軸 PID、Servo 限幅、失聯保護與硬體校正。專案包含一般 PID 韌體，以及具備 5-pin 搖桿離散校正模式的韌體。

> **安全優先：** 兩顆 Servo 必須使用獨立、足夠電流的 5–6 V 電源，且外部電源 GND 必須與 Nano GND 共地。不要用 Nano 的 5 V 腳直接供應 Servo。第一次測試請空載、使用小角度，並隨時準備斷開 Servo 電源。

## 系統架構

```text
USB 攝影機 → OpenCV 球體辨識／透視校正 → USB Serial（POS / LOST）
                                                ↓
                         Arduino Nano：雙軸 PID／安全限幅／失聯保護
                                                ↓
                               D9 / D10 Servo → 球平衡平台

5-pin 搖桿（A0、A1、D2）→ Nano TEST 模式 → 機構校正
```

## 專案檔案與用途

| 路徑 | 用途 |
| --- | --- |
| `README.md` | 本文件：完整操作、校正與故障排除流程。 |
| `Arduino_Nano_Ball_Balance/Arduino_Nano_Ball_Balance.ino` | Nano 正式 Camera PID 韌體；**不含搖桿功能**。完成機構校正後，將中心／限幅常數回填到此檔並燒錄，用於正式相機控制。 |
| `Arduino_Nano_Ball_Balance/README.md` | 正式 Camera PID 韌體的接線、通訊協定與調參說明。 |
| `Arduino_Nano_Ball_Balance_Joystick_Modes/Arduino_Nano_Ball_Balance_Joystick_Modes.ino` | 搖桿機構校正版；用於空載找中心、測端點與輸出應回填的常數，不是正式 Camera PID 燒錄目標。 |
| `Arduino_Nano_Ball_Balance_Joystick_Modes/README_Joystick_Discrete_Modes.md` | 搖桿 TEST 模式、MODE1/MODE2 與校正指令說明。 |
| `Camera_Vision/ball_vision.py` | OpenCV 球體追蹤、透視轉換、Nano Serial 通訊與視窗互動。 |
| `Camera_Vision/start_camera_vision.sh` | 啟動攝影機程式的便利腳本。 |
| `Camera_Vision/requirements.txt` | Python 相依套件清單。 |
| `Camera_Vision/camera_config.json` | 攝影機、平台尺寸／四角、球色 HSV、零點與 Serial 設定；會由校正操作更新，不保存 PID。 |
| `Camera_Vision/README.md` | 攝影機安裝、執行與視窗快捷鍵說明。 |
| `3d列印檔案/` | 平台、Servo 固定座、鏡頭座等 3D 列印檔；目前依 `.gitignore` 不上傳。 |
| `平衡球.txt` | 本機參考資料；目前依 `.gitignore` 不上傳。 |

## 硬體接線

### Servo 與 Nano

| Nano 腳位 | 連接 |
| --- | --- |
| D9 | Servo X 訊號 |
| D10 | Servo Y 訊號 |
| GND | Servo 外部電源的共地 |
| USB | Desktop 連線、燒錄與 Serial（115200 baud） |

### 5-pin 搖桿（僅搖桿校正版）

| Nano 腳位 | 搖桿腳位 | 用途 |
| --- | --- | --- |
| A0 | VRx | X 軸類比輸入 |
| A1 | VRy | Y 軸類比輸入 |
| D2 | SW | 按鈕，程式使用 `INPUT_PULLUP` |
| 5V | VCC / +5V | 搖桿供電 |
| GND | GND | 共地 |

## 第一次使用：建議順序

### 1. 安裝 Arduino 函式庫並燒錄校正韌體

在 Arduino IDE 的 Library Manager 安裝：

- `Servo`

先開啟並燒錄搖桿機構校正版：

```text
Arduino_Nano_Ball_Balance_Joystick_Modes/
  Arduino_Nano_Ball_Balance_Joystick_Modes.ino
```

選擇 **Arduino Nano**；若舊款 Nano 無法上傳，改選 `ATmega328P (Old Bootloader)`。

> Arduino 會把同一 sketch 資料夾內所有 `.ino` 檔一起編譯；該資料夾只能保留主 `.ino`，不要將備份 `.ino` 放入其中。

### 2. 用搖桿完成機構校正，再燒錄正式 Camera PID 韌體

此版本開機後會**自動進入 TEST 模式**，不需要在開機時長按按鈕。預設為 MODE1。

- 每次推桿只觸發一個軸的一次動作；推桿要先回到中立，至少等待約 350 ms 才能進行下一步。
- 短按 SW 在 MODE1 / MODE2 之間切換。
- 在 TEST 模式長按 SW 1.5 秒：兩軸回到目前設定的中心角度。

| 模式 | 每次動作 |
| --- | --- |
| MODE1 | 選定軸 ±1°；用於找平台水平。 |
| MODE2 | 直接前往推導後的 MIN / MAX 端點；目前中心 90、範圍 ±20，為 70° / 110°。只在空載、隨時可斷電時使用。 |

建議做法：

1. 平台空載，以 MODE1 逐步找到水平位置。
2. 切 MODE2，逐一測試四個端點；若有卡住風險，立刻縮小 `SERVO_LIMIT_OFFSET_DEG`。
3. 在 Arduino Serial Monitor 設為 **115200 baud**、行尾選擇 Newline，輸入：

   ```text
   SETC
   SHOW
   ```

   `SETC` 將目前角度設為測試中心；`SHOW` 印出可複製的 `SERVO_X_CENTER`、`SERVO_Y_CENTER`、`SERVO_LIMIT_OFFSET_DEG` 常數，並顯示推導後的 MIN/MAX 端點。把輸出值寫回 `Arduino_Nano_Ball_Balance/Arduino_Nano_Ball_Balance.ino` 頂端的同名常數，再燒錄此正式 Camera PID 韌體，中心與安全行程才會永久保存並交給相機控制使用。

   Arduino Serial Monitor 會獨占 Nano serial port；完成 `SETC` / `SHOW` 後請關閉 Serial Monitor，再啟動 Camera Vision。若 Serial Monitor 或其他 serial 工具仍開著，launcher 會拒絕接手該 port 並印出忙碌診斷。

### 3. 安裝並校正攝影機

```bash
cd ~/文件/平衡球PID教案/Camera_Vision
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
./start_camera_vision.sh
```

啟動選單中，`1` 會搜尋可讀寫且沒有被 Arduino Serial Monitor 佔用的 Nano，確認後開啟 READY 設定畫面；`2` 會以 `--no-serial` 開啟同一個 READY 設定畫面，只做鏡頭、HSV、四角與零點檢查，RUN 仍需稍後連上 Nano。

先不接 Nano，確認辨識品質：

```bash
.venv/bin/python ball_vision.py --no-serial
```

在視窗中操作：

| 按鍵／操作 | 功能 |
| --- | --- |
| `a` | 重新套用已儲存的 `camera_config.json`，回到 READY checklist。 |
| `b`，再點球 | 取樣球的 HSV 顏色；確認綠色圓圈穩定包住球。 |
| `c` | 依序點平台左上、右上、右下、左下四角；幾何與順序正確才存入 `camera_config.json`。成功後會清除舊零點，必須重新按 `d`。 |
| `d`，再點平台 | 將點到的位置設為平衡零點 `(0, 0)`%。 |
| `n` | 回 READY 並重新搜尋 Nano。 |
| `p` | 回 READY，送出 `READY` 並停止 live `POS` / `LOST` / Servo 控制。 |
| `r` | checklist 完成、球可見、Nano 已連線時啟動 RUN；RUN 中再按一次回 READY。 |
| `q` / `Esc` | 安全停止並結束。 |

固定鏡頭後才做四角校正；鏡頭或平台一移動，就要重新按 `c` 校正。若點擊超出影像範圍、四角重複、退化、內凹或不是 TL→TR→BR→BL，程式會拒絕新設定並保留上一版有效平台設定。平台實際寬、高可修改 `camera_config.json` 的 `width_mm`、`height_mm`。

### 4. 調整 PID

1. 先確認 Servo 中心、限幅與兩軸方向都已完成。
2. 放球後先以單軸、小動作測試；按 `r` 啟動後若球被推得更遠，立刻按 `r` 停止，將對應的 `SERVO_X_DIRECTION` 或 `SERVO_Y_DIRECTION` 改為 `-1`。
3. Nano 韌體是唯一 PID 參數與控制權威；Camera Vision 只送 `GEOM`、`POS`、`LOST`、`READY`、`RUN`，不送 PID 或 Servo 角。
4. 正式 `.ino` 現在採用未實機驗證的 asymmetric normalized P-only 映射：Camera 先送 `GEOM,x_min_pct,x_max_pct,y_min_pct,y_max_pct`；Nano 以 `error_pct = target_pct - ball_pct`，正向誤差除以 `target_pct - min_pct`，負向誤差除以 `max_pct - target_pct`，再 clamp 到 `[-1,+1]`。
5. 目前 X/Y `Kp=1.00`、`Ki=0.00`、`Kd=0.00`；full-scale edge error 在 `Kp=1.0` 時要求該軸中心 ±20° 的校正安全端點，`Kp=0.5` 時要求 50%。
6. 若要調整 PID、方向或安全端點，修改並重新燒錄 Nano `.ino`；重啟 Camera Vision 或改 `camera_config.json` 不會改 Nano PID。

`GEOM` 來自當次 `C` 四角與 `D` 零點：`x_min=-100-zero_x`、`x_max=100-zero_x`、`y_min=-100-zero_y`、`y_max=100-zero_y`。近邊與遠邊距離可不對稱，但都會各自映射到 full-scale safe endpoint；舊固定 X/Y mm 範圍與 ±8° 映射已移除。預設 P-only 參數是新控制模型的安全起點，尚未完成實機閉迴路驗證。

## 常用 Serial 指令（搖桿校正版）

Serial Monitor 設定為 **115200 baud**，行尾選擇 Newline。

| 指令 | 用途 |
| --- | --- |
| `STATUS` | 顯示目前 Servo、中心、模式與搖桿 armed 狀態。 |
| `SETC` | 將目前兩軸 Servo 角度設成測試中心。 |
| `SETCX` / `SETCY` | 僅設定目前 X / Y 的測試中心。 |
| `SHOW` | 印出應複製回程式的中心與 offset 常數，並顯示推導後的限幅端點。 |
| `CENTER` | 兩軸回到目前測試中心。 |
| `MODE,1` / `MODE,2` | 從 Serial 選擇 MODE1 微調或 MODE2 端點模式。 |
| `STEP,1` | 從 Serial 選擇 MODE1 的 1° 微調。 |
| `X+`、`X-`、`Y+`、`Y-` | 手動使指定軸移動一步。 |
| `X,90` / `Y,90` / `XY,90,90` | 直接指定安全範圍內的角度。 |
| `TESTX` / `TESTY` / `TESTBOTH` | 執行內建機構來回測試。 |
| `JOY` | 印出目前搖桿 ADC 值與中心值。 |
| `JOYCAL` | 以目前搖桿中立位置重新取樣中心。 |
| `TELNOW` | 取得一次 Nano telemetry。 |

## 授權

本專案採用 [MIT License](LICENSE)。你可以自由使用、修改、散布、再授權與商業使用本專案的程式碼與文件；請保留原始授權與著作權聲明。

硬體組裝、Servo 供電與機械結構具有風險，使用者應自行完成安全檢查、限幅設定與測試；作者不對由使用、修改或組裝本專案造成的損害負責。
