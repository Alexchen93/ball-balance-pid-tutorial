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
| `Arduino_Nano_Ball_Balance/Arduino_Nano_Ball_Balance.ino` | 原始 Nano 正式 PID 韌體；**不含搖桿功能**。適合完成機構校正後的純相機控制。 |
| `Arduino_Nano_Ball_Balance/README.md` | 原始 PID 韌體的接線、通訊協定與調參說明。 |
| `Arduino_Nano_Ball_Balance_Joystick_Modes/Arduino_Nano_Ball_Balance_Joystick_Modes.ino` | 建議燒錄的搖桿校正版；保留相機/PID 功能，並增加開機自動 TEST 與搖桿單步控制。 |
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

### 1. 安裝 Arduino 函式庫並燒錄

在 Arduino IDE 的 Library Manager 安裝：

- `Servo`

開啟並燒錄：

```text
Arduino_Nano_Ball_Balance_Joystick_Modes/
  Arduino_Nano_Ball_Balance_Joystick_Modes.ino
```

選擇 **Arduino Nano**；若舊款 Nano 無法上傳，改選 `ATmega328P (Old Bootloader)`。

> Arduino 會把同一 sketch 資料夾內所有 `.ino` 檔一起編譯；該資料夾只能保留主 `.ino`，不要將備份 `.ino` 放入其中。

### 2. 用搖桿完成機構校正

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

   `SETC` 將目前角度設為測試中心；`SHOW` 印出可複製的 `SERVO_X_CENTER`、`SERVO_Y_CENTER`、`SERVO_LIMIT_OFFSET_DEG` 常數，並顯示推導後的 MIN/MAX 端點。把輸出值寫回 `.ino` 頂端的同名常數，再重新燒錄，中心與安全行程才會永久保存。

### 3. 安裝並校正攝影機

```bash
cd ~/文件/平衡球PID教案/Camera_Vision
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
./start_camera_vision.sh
```

先不接 Nano，確認辨識品質：

```bash
.venv/bin/python ball_vision.py --no-serial
```

在視窗中操作：

| 按鍵／操作 | 功能 |
| --- | --- |
| `b`，再點球 | 取樣球的 HSV 顏色；確認綠色圓圈穩定包住球。 |
| `c` | 依序點平台左上、右上、右下、左下四角；結果存入 `camera_config.json`。 |
| `0` | 將目標重設為平台中心 `(0, 0)` mm。 |
| 左鍵點平台 | 設定新的平面座標目標。 |
| `r` | 在 `READY` / `RUN` 間切換 PID。 |
| `q` / `Esc` | 安全停止並結束。 |

固定鏡頭後才做四角校正；鏡頭或平台一移動，就要重新按 `c` 校正。平台實際寬、高可修改 `camera_config.json` 的 `width_mm`、`height_mm`。

### 4. 調整 PID

1. 先確認 Servo 中心、限幅與兩軸方向都已完成。
2. 放球後先以單軸、小動作測試；按 `r` 啟動後若球被推得更遠，立刻按 `r` 停止，將對應的 `SERVO_X_DIRECTION` 或 `SERVO_Y_DIRECTION` 改為 `-1`。
3. Nano 韌體是唯一 PID 參數來源；Camera Vision 只送 `POS`、`LOST`、`READY`、`RUN` 與必要 `TARGET`。
4. 本 P-only 測試值在 `Arduino_Nano_Ball_Balance.ino`：X/Y `Kp=0.10`、`Ki=0.00`、`Kd=0.00`。
5. 若要調整 `Kp`、`Ki` 或 `Kd`，修改並重新燒錄 Nano `.ino`；重啟 Camera Vision 或改 `camera_config.json` 不會改 Nano PID。

預設 P-only 參數只是保守起點，不保證適用於不同連桿、球體、平台剛性或攝影機延遲。

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
