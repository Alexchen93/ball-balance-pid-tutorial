# Camera Vision：Desktop 影像辨識與 Nano 通訊

`ball_vision.py` 在 Desktop 取得 USB 攝影機畫面，以 HSV 追蹤球，將平台座標送到 Arduino Nano。Nano 才負責 PID、Servo 方向、限幅與失聯保護。

## Windows 啟動方式

現有的 `start_camera_vision.sh` 是 Linux 專用，不能直接在 Windows 執行。將更新後的整個 `Camera_Vision` 資料夾複製到 Windows 電腦，然後在檔案總管**雙擊 `start_camera_vision.bat`**。不要複製 Linux 的 `.venv` 或另一台電腦的 `.venv-win`。

首次使用需已安裝 Windows 版 Python 3.10、3.11 或 3.12，並連接網路。啟動器會自動建立 `.venv-win` 並安裝必要套件；可能需要幾分鐘。若尚未安裝 Python，視窗會提示安裝，安裝後再雙擊一次。之後雙擊即可直接啟動。

程式會使用 `camera_config.json` 中的 `camera.index`（預設 0），並自動搜尋 Nano；沒有 Nano 仍可進入 READY 畫面。如果開到錯的鏡頭，關閉程式並將 `camera_config.json` 的 `camera.index` 改為其他編號，例如 `1`，再雙擊啟動。程式啟動失敗時會保留錯誤訊息，方便檢查。

若有多個 COM 埠，可用 Windows 裝置管理員確認 Nano 的 COM 編號，然後手動啟動（將 `COM3` 及 index 換成實際值）：

```powershell
.\.venv-win\Scripts\python.exe ball_vision.py --camera-index 0 --serial-port COM3
```

沒有畫面時，先用 Windows「相機」程式確認鏡頭可用，再關閉它及其他使用鏡頭的程式；檢查 Windows 的攝影機隱私權設定允許桌面應用程式存取。若 Nano 連不上，也請關閉 Arduino Serial Monitor，並檢查 COM 埠與驅動程式。若仍失敗，請提供終端機完整錯誤訊息，以及是在 PowerShell、命令提示字元、Git Bash 或 WSL 執行。

> Linux 與 Windows 的攝影機位置、解析度及 index 可能不同；更換電腦或鏡頭後，請在 READY 重新執行 B、C 校正並檢查接觸點，再使用 RUN。

## Linux Desktop 一次性設定

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

## Linux 啟動方式

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

RUN 時若同時出現多個符合 HSV 的區域，優先選擇最接近上一個有效球心的候選，減少背景突然取代球的情況；這不是身分保證，遮擋或相近顏色仍可能誤判。候選不會因位於邊框外而被刪除，偵測到的接觸點越界仍立即停機。READY 時仍以最大候選供校正和啟動檢查。

球體的窄黑色接縫可能把 HSV 區域切成兩塊；辨識時會用預設 7 像素的閉合運算連接小縫隙，避免追到半顆球。可在 `detection.seam_close_kernel_px` 設定，設為 1 可停用額外閉合。過大的值可能把靠近的其他區域合併，仍需確認綠圈與橘點準確。

球有高度，斜拍時綠色圓心不在平台平面。程式以 `camera_config.json` 的 `detection.contact_offset_radius` 乘球半徑，再沿 `contact_direction_px` 指定的影像方向推估接觸點；方向向量只取方向，影像右方為 X 正、下方為 Y 正。未設定有效方向時，沿平台遠端中點到近端中點方向補償。

目前設定為比例 `0.98`、方向 `[0.014, 1.0]`：依固定攝影機畫面中球心約 `(335, 193)`、半徑約 52 像素及黃色參考標記中心約 `(336, 244)` 估算，只是單一位置的初始校正，尚未經實機複核。這些座標是截圖的像素位置，僅用來說明推導，不是平台控制座標。綠色圓是 HSV 區域的最小外接圓，不是平台接觸點的標準答案；不要只為了讓橘點落在綠線上而調整。

更新程式碼後先重新啟動 Camera Vision，使新的方向功能生效；之後若只修改設定，可在 READY 按 `A` 重新讀取。在不啟動 RUN 的 READY 狀態，把球依序放在可辨識的中央、左右及前後位置，核對接觸位置與橘點。手不要遮住球或參考位置；若球會移動，先確保平台及球穩定。比例和方向只採固定近似，偏差可能隨位置、攝影機角度和球色辨識而變化；不符時先停止、記錄畫面，勿用 PID 參數掩蓋感測誤差。攝影機、平台或球體改動後需重新核對；按 C 只校正平台四角，**不會重新計算這組接觸點方向與比例**。

## READY → RUN 操作

| 操作 | 功能 |
| --- | --- |
| `a` | 重新讀取已儲存的 `camera_config.json`。 |
| `b` | 點球中心，取樣 HSV。 |
| `c` | 點 TL → TR → BR → BL 四角；完成後自動把幾何中心設為零點，點選進度顯示於終端機。 |
| `x` | 回 READY 並啟動黑色十字線的輔助檢查；移開球後點一處線條取樣。 |
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

## 黑色十字線輔助檢查四角

先在壓克力上畫黑色十字：交點位於平台中心，兩條線沿平台長寬方向，四端朝各邊中點。畫線及四角點選本身都要盡量準確。

1. 完成 `C` 四角校正，按 `P` 保持 READY，將球移開；檢查時平台和攝影機位置須固定。
2. 按 `X`（若原本在 RUN，會先停止控制），依終端提示點一下黑色十字線上清楚、沒有反光的線段。兩個方向的線都需要可見；按 `P` 可取消。
3. 程式將四角範圍做透視轉換，搜尋兩條交會的長線，終端顯示交點相對四角中心的 X／Y 偏移，分別以**整個平台寬、高的百分比**表示，X 正為右、Y 正為遠端；同時顯示兩線在轉正後的方向偏轉角。辨識到的交點會以藍色十字顯示 5 秒。
4. 當任一方向中心差超過整體寬／高的 5%，或任一方向偏轉超過 8°，終端提示檢查線條與四角。這些是暫定的提醒門檻，不是儀器精度或驗收標準。如果辨識不到兩條線，請減少反光、避開壓克力下方的深色背景或線材，再重新按 X 點線；線條需有足夠長度與對比，且盡量保持兩條線使用同一支黑色筆。

十字線只能提供輔助：手繪位置、線條彎曲、背景或遮擋都可能讓偵測結果不準；即使檢查結果在門檻內，也不能保證四角都正確。程式不會自動改動四角、控制原點、CONTACT 或 PID；四角不正確時仍由使用者按 `C` 重做校正。每次檢查只分析取樣時的畫面，不會連續追蹤，平台或攝影機移動後要重新檢查。

攝影機視窗上方以繁體中文顯示狀態與 Nano 連線情況，其餘操作與結果看終端機。「Nano：已連接」表示序列埠已開啟，不等於持續收到 Nano 心跳；連線或控制錯誤會在終端機報告。綠圈、紅色球心、橘色接觸點、青色邊框與藍色十字是圖形標記。`status_zh_tw.png` 為畫面中的中文字樣，複製程式時須一起保留。

## 安全與故障排除

- READY 中不會送 `POS`、`LOST` 或 Servo 控制；按 `P` 可隨時回 READY。
- Nano 連續約 300 ms 沒收到新位置或收到 `LOST`，會回中立角。
- 正式韌體更新並重新燒錄後，停止時終端會顯示 `NANO STOP_REASON,BALL`（球未辨識或超出範圍）、`LOST`（Nano 接收逾時）或 `RANGE`（Nano 拒絕範圍外座標）等原因；Nano 重開機或 USB 已中斷時可能沒有原因訊息。
- RUN 中若暫時未偵測到球，Camera Vision 等待約 80 ms；期間不重送舊座標，Servo 維持先前指令。若球仍未出現，傳送 `LOST` 並顯示「未偵測到球（NO_DETECTION）」。實際時間還受影格間隔影響；Nano 的約 300 ms 位置逾時保護仍獨立運作。
- 若辨識出的 CONTACT 座標超出校正邊框，立即傳送 `LOST` 並顯示「超出平台邊界（OUTSIDE）」及位置；幾何座標無效則顯示「平台幾何資料無效（INVALID_GEOMETRY）」。停機後仍須排除問題並按 `R` 重新啟動。
- OUTSIDE 停機後，程式另外輸出當幀球心、半徑及 CONTACT 像素座標，並將該幀含標記畫面存成 `vision_last_outside.png`，方便查看是否抓到球以外的紅色區域。每次 OUTSIDE 會覆寫這個本機檔案；儲存影像發生在送出 `LOST` 之後，並不延遲停機命令。
- 接收程式會等待完整換行才解析 Nano 資料；`POS,OK` 仍用於握手，但不再逐筆刷在終端機。
- 「無法啟動 RUN：Nano 尚未連線」：按 `n` 搜尋，檢查 USB、權限與 Serial Monitor。
- 「HSV 偵測位置超出校正後的平台」：目前辨識到的候選接觸點不在 C 的平台內。重新做 B / C，或檢查橘色 CONTACT 點補償。
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
