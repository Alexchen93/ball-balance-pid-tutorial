# Camera Vision：Desktop 影像辨識與 Nano 通訊

`ball_vision.py` 在 Desktop 取得 USB 攝影機畫面，以 HSV 追蹤球，將平台座標送到 Arduino Nano。Nano 才負責 PID、Servo 方向、限幅與失聯保護。

## Desktop 一次性設定

```bash
cd ~/文件/平衡球PID教案/Camera_Vision
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

Nano 使用 USB serial。確認裝置與權限：

```bash
python3 -m serial.tools.list_ports -v
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

常見裝置是 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。若顯示目前使用者無法讀寫，執行下列命令後**登出再登入**，並重新插拔 Nano：

```bash
sudo usermod -aG dialout "$USER"
```

Arduino IDE Serial Monitor、其他 serial monitor 或另一個 Camera Vision 視窗都可能佔用 port；啟動前要關閉它們。

## 啟動方式

```bash
cd ~/文件/平衡球PID教案/Camera_Vision
./start_camera_vision.sh
```

啟動後第一步會顯示攝影機**編號選單**。啟動器會先測試並只列出實際能讀取影像的節點，避免選到 USB 攝影機的中繼資料節點。直接選擇內建或外接 USB 鏡頭的編號即可；不需要輸入 `/dev/video*` 路徑、鏡頭 index 或其他攝影機參數。其後的 READY／RUN、HSV 與平台校正操作均與原本相同。

啟動選單：

| 選項 | 用途 |
| --- | --- |
| `1` | 搜尋一個可讀寫、未被占用的 Nano，連線後進 READY。 |
| `2` | 以 `--no-serial` 只檢查鏡頭與校正；不能 RUN。 |
| `q` | 離開。 |

手動指定 serial port：

```bash
.venv/bin/python ball_vision.py --serial-port /dev/ttyUSB0
```

## 座標與畫面標記

### 相機像素與平台座標

相機像素：右方 `x` 增加、下方 `y` 增加。平台控制座標由 `C` 四角透視校正後產生：

| 控制座標 | 對應畫面方向 |
| --- | --- |
| `X-` / `X+` | 左 / 右 |
| `Y+` | TL、TR 的上方／遠端 |
| `Y-` | BL、BR 的下方／近端 |
| `(0,0)` | 四角算出的平台中心 |

請使用校正後的平台座標判讀，不要直接以斜拍畫面的梯形或像素 `y` 判斷方向。

### 球的位置

- **綠色圓**：HSV 偵測到的球外框；紅點是影像圓心。
- **橘色 `CONTACT` 點**：推估的球與平台接觸點；程式實際傳送給 Nano、檢查是否在邊框內的就是這個點。

球有高度，斜拍時綠色圓心不在平台平面。程式以 `camera_config.json` 的 `detection.contact_offset_radius`（目前 `0.85`）乘球半徑，朝 TL/TR 到 BR/BL 的近端方向補償。如果橘點長期在接觸點上方或下方，再小幅調整此值；不要直接以綠色圓心取代它。

## READY → RUN 操作

| 操作 | 功能 |
| --- | --- |
| `a` | 重新讀取已儲存的 `camera_config.json`。 |
| `b` | 點球中心，取樣 HSV。 |
| `c` | 點 TL → TR → BR → BL 四角；完成後自動把幾何中心設為零點。 |
| `n` | 重新搜尋 Nano。 |
| `p` | 回 READY；停止 live `POS` / `LOST` / Servo 控制。 |
| `r` | Nano 已連線、球可辨識、B/C 完成時進 RUN；RUN 中再按一次回 READY。 |
| `q` / `Esc` | 安全停止並離開。 |
| `Ctrl+C`、停止 `.sh` 程序或 `SIGTERM` | 安全關閉影像視窗、釋放攝影機，並送 Nano 回 `READY`。 |

> 沒有 `D` 操作。C 成功後，平台中心自動是 `(0,0)`。

### 校正順序

1. 固定鏡頭與平台，四角完整入鏡。
2. 按 `B` 點球中心；確認綠色圓穩定包住球。
3. 按 `C`，依序點 TL、TR、BR、BL。錯誤順序、重複、內凹或退化四邊形會被拒絕，上一份有效設定會保留。
4. 確認橘色 CONTACT 點靠近球接觸平台的位置。
5. 連接 Nano，讓球留在平台內，再按 `R`。

`C` 成功後會寫入 `camera_config.json`。鏡頭、平台或解析度改變後必須重新做 `C`。

## 安全與故障排除

- READY 中不會送 `POS`、`LOST` 或 Servo 控制；按 `P` 可隨時回 READY。
- Nano 連續約 300 ms 沒收到新位置或收到 `LOST`，會回中立角。
- `Cannot RUN: Nano is not connected`：按 `n` 搜尋，檢查 USB、權限與 Serial Monitor。
- `Cannot RUN: HSV candidate is outside...`：目前辨識到的候選接觸點不在 C 的平台內。重新做 B / C，或檢查橘色 CONTACT 點補償。
- `ERROR,POSITION_OUT_OF_RANGE`：Nano 拒絕範圍外的位置；確認使用最新版 Camera Vision，並檢查 B、C 和橘色 CONTACT 點。
- 若球被推向更遠方向，立即按 `P`，再調整 Nano 的 `SERVO_X_DIRECTION` 或 `SERVO_Y_DIRECTION`；Camera Vision 不設定 PID。

## 通訊摘要

RUN 前的安全握手：

```text
GEOM,-100,100,-100,100 → GEOM,OK
fresh POS                → POS,OK
final fresh POS + RUN    → STATE,RUN
```

RUN 中，Desktop 傳送 `POS,x_pct,y_pct,timestamp_ms`；找不到球才傳 `LOST`。Nano 韌體不接受 Camera Vision 下發 PID 參數，`PIDX` / `PIDY` 會回 `ERROR,PID_MANAGED_BY_NANO`。
