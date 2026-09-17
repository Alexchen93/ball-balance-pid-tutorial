# Arduino Nano 電腦視覺球平衡控制

這是一套給學生學習「影像辨識、座標轉換、回授控制與 PID」的球平衡平台。Desktop 的 USB 攝影機辨識球的位置，Python 將平台座標傳給 Arduino Nano；Nano 依座標控制兩顆 Servo，讓球回到平台中心。

> **安全第一：** Servo 必須使用獨立、足夠電流的 5–6 V 電源，外部電源 GND 必須與 Nano GND 共地。不要使用 Nano 的 5 V 腳供應 Servo。第一次測試請空載、小角度，並隨時可斷開 Servo 電源。

## 系統架構

```text
USB 攝影機 → OpenCV：球色辨識、接觸點估計、透視校正
                                  ↓ POS / LOST / GEOM / RUN
Desktop ───────────────────── USB Serial ─────────────────→ Arduino Nano
                                                               ↓
                                                     雙軸 PD / 安全限幅
                                                               ↓
                                                       D9、D10 Servo → 平台
```

## 專案內容

| 路徑 | 用途 |
| --- | --- |
| `Camera_Vision/` | Desktop 的 OpenCV 程式、設定與學生操作說明。 |
| `Arduino_Nano_Ball_Balance/` | 正式 Camera PID 韌體與接線／調參說明。 |
| `Arduino_Nano_Ball_Balance_Joystick_Modes/` | **機構校正用**搖桿韌體；不是正式 Camera PID 韌體。 |
| `3d列印檔案/` | 平台、Servo 固定座、鏡頭座等模型（依 `.gitignore` 不上傳）。 |

## 學生先懂這三件事

### 1. 影像像素座標不等於平台控制座標

相機原始影像中：向右是像素 `x` 增加、向下是像素 `y` 增加。因為鏡頭斜拍，平台在畫面裡是梯形；不能直接把像素座標拿去控制 Servo。

按 `C` 依序點 **TL → TR → BR → BL** 後，程式建立 homography（透視校正），把梯形平台轉為正方形控制座標：

| 平台控制座標 | 畫面中的方向 |
| --- | --- |
| `X` 負 → 正 | 左 → 右 |
| `Y` 正 | 平台上方／遠端：TL、TR 的方向 |
| `Y` 負 | 平台下方／近端：BL、BR 的方向 |
| `(0, 0)` | 由四角算出的平台幾何中心 |

所以「影像左右」是 **X 軸**；影像原始向下雖是 `y` 增加，但本專案的控制 `Y` 軸刻意翻轉，讓上方為正、下方為負。

### 2. 綠色圓與橘色 CONTACT 點不同

- **綠色圓**：HSV 找到的球外框與其影像圓心。
- **橘色 `CONTACT` 點**：估計球接觸平台的位置，才是送給 Nano 控制與判斷是否在平台內的點。

斜拍時，球的影像圓心不在平台平面上。程式會依球半徑、平台近端方向，將圓心往接觸方向補償 `0.85 × 半徑`，再做透視轉換。因此請確認**橘色點**而非綠色圓心落在球接觸平台的位置附近。

### 3. PID 在 Nano，不在 Desktop

Camera Vision 只傳送球的位置與安全狀態；正式 Nano 韌體才是 PID、Servo 方向與安全角度的唯一來源。目前起始參數是保守 PD：

```text
X/Y：Kp = 1.25、Ki = 0.00、Kd = 0.12
```

- `P`：球偏離中心越遠，回拉傾斜越大。
- `D`：球快速往外滾時，提早加強煞車。
- `I`：目前保持 0，避免積分造成過衝。

詳見 `Arduino_Nano_Ball_Balance/README.md`。

## 第一次使用流程

### 1. 校正機構並燒錄正式 Nano 韌體

1. 在 Arduino IDE 安裝 `Servo` 函式庫。
2. 先用 `Arduino_Nano_Ball_Balance_Joystick_Modes/` 校正平台水平、Servo 中心與安全端點。
3. 把 `SETC` / `SHOW` 的結果填入正式 `Arduino_Nano_Ball_Balance/Arduino_Nano_Ball_Balance.ino` 的中心與限幅常數。
4. 選擇 **Arduino Nano**（舊板上傳失敗時試 `ATmega328P (Old Bootloader)`），燒錄正式 `.ino`。
5. 使用 Camera Vision 前關閉 Arduino Serial Monitor；它會佔用 Nano 的 serial port。

### 2. 在 Desktop 安裝 Camera Vision

```bash
cd ~/文件/平衡球PID教案/Camera_Vision
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

若顯示 Nano 沒有讀寫權限，將目前 Desktop 使用者加入 serial 群組，登出再登入並重新插拔 Nano：

```bash
sudo usermod -aG dialout "$USER"
```

### 3. 固定鏡頭並完成 B、C

1. 鏡頭固定後，讓四角和整個平台都入鏡；鏡頭或平台移動後一定要重新按 `C`。
2. 執行 `./start_camera_vision.sh`。
3. 選 `2` 可先只測鏡頭，選 `1` 會連接可用 Nano。
4. 按 `B`，點球中心取樣 HSV。綠色圓應穩定包住球。
5. 按 `C`，依序點 TL、TR、BR、BL 四角。成功後，橘色 CONTACT 點用來確認接觸位置。
6. 連上 Nano、球在平台內可辨識時，按 `R` 進 RUN；按 `P` 回 READY，按 `Q` / `Esc` 安全離開。

> 此版本已移除 `D` 零點操作：完成 `C` 時，四角計算出的平台中心自動成為 `(0,0)`。

## PID 學習與測試順序

1. 先空載確認兩軸中心、方向與安全限幅。
2. 放球在中心，單軸小幅偏移測試；若球被推得更遠，立刻按 `P`，再調整對應 `SERVO_X_DIRECTION` 或 `SERVO_Y_DIRECTION`。
3. 若球接近邊緣才來得及回拉，先看 telemetry 的 `tilt`：
   - 已接近 ±20°：機構回拉力／Servo 速度或影像延遲可能不足。
   - 尚未接近最大：可小幅提高 `Kp`，或調整 `Kd` 讓高速外移提早煞車。
4. 若出現高頻抖動，先將 `Kd` 由 `0.12` 降為 `0.08`；不要先增加 `Ki`。

## 文件索引

- [Camera Vision 操作與故障排除](Camera_Vision/README.md)
- [Nano 韌體、接線、PD 參數與 telemetry](Arduino_Nano_Ball_Balance/README.md)
- [搖桿機構校正說明](Arduino_Nano_Ball_Balance_Joystick_Modes/README_Joystick_Discrete_Modes.md)

## 授權

本專案採用 [MIT License](LICENSE)。硬體、Servo 電源與機械結構具有風險；使用者應自行完成安全檢查與測試。
