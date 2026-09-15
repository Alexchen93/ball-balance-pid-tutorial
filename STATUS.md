# 平衡球 PID 平台目前狀態

更新日期：2026-09-15

## 已完成且已做程式層驗證

- 正式控制流程由 Nano 韌體掌管 PID；Camera Vision 只在 RUN 前傳送平台幾何資訊與即時位置。
- RUN 握手固定為：`GEOM` → `GEOM,OK` → fresh `POS` → `POS,OK` → final fresh `POS` + `RUN` → `STATE,RUN`。
- 平台範圍由每次四角校正（C）與零點（D）推導，可處理不對稱的零點與平台邊界；Nano 依近、遠邊距離分別正規化 PID 誤差。
- Servo X/Y 各自使用安全中心與安全端點；目前程式中的中心為 X=74°、Y=76°，安全行程為各軸中心 ±20°。

## 2026-09-15 RUN 握手修正

### 問題

Nano 已正確回覆 `GEOM,OK`，但 Python 的 `NanoTransport.poll()` 只把 `ERROR`、`STATE`、`POS,OK` 交給 RUN 狀態機。`GEOM,OK` 因此只顯示在終端，沒有推進 `wait_geom_ack`，造成 RUN 逾時並回到 READY。

### 修正

- `Camera_Vision/ball_vision.py` 現在將 `GEOM,OK` 納入已解析事件。
- `Camera_Vision/tests/test_nano_transport_serial.py` 新增 `GEOM,OK` 回歸測試，防止同類 ACK 再次被僅列印、不交給狀態機。

### 驗證

在 `Camera_Vision/.venv` 執行全部單元測試：**19 tests passed**。

## 尚待實機確認

- 需要重新啟動 Camera Vision，使用已燒錄目前 `Arduino_Nano_Ball_Balance.ino` 的 Nano 做一次受控 RUN 測試。
- 測試前關閉 Arduino Serial Monitor，使用 A 套用已存設定，確認球置於中心；按 R 後預期會看到 `GEOM,OK`、`POS,OK`、`STATE,RUN` 與 `RUN confirmed`。
- 若位置、相機或機構調整，重新執行 C（TL → TR → BR → BL）與 D（零點）後再 RUN。
