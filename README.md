# Arduino Nano 電腦視覺球平衡控制

本專案以固定在平台上方的 USB 攝影機量測球的位置，Desktop 上的 Python / OpenCV 程式將位置座標經 USB Serial 傳給 Arduino Nano；Nano 負責雙軸 PID、Servo 限幅、失聯保護與 SSD1306 OLED 狀態顯示。

## 專案內容

- `Arduino_Nano_Ball_Balance/`：Nano 韌體、接線與機構校正說明。
- `Camera_Vision/`：OpenCV 球體偵測、透視校正、Serial 通訊與可執行啟動腳本。
- `3d列印檔案/`：球平衡平台的 STL 列印檔。
- `平衡球.txt`：原始參考網址。

## 系統架構

```text
USB 攝影機 → OpenCV 球心辨識／透視校正 → USB Serial POS/LOST
                                          ↓
Arduino Nano 雙軸 PID → D9 / D10 Servo → 球平衡平台
                    ↘ I2C SSD1306 OLED
```

電腦每秒傳送最多 30 筆位置資料；Nano 若收到 `LOST` 或超過 300 ms 沒有新的座標，會清除積分項並使 Servo 回到中立角。

## 快速開始

1. 依 `Arduino_Nano_Ball_Balance/README.md` 接線、安裝函式庫並燒錄 Nano 韌體。
2. Servo 使用獨立且足夠電流的 5–6 V 電源，**必須**與 Nano 共地。
3. 在 Desktop 執行：

   ```bash
   cd Camera_Vision
   ./start_camera_vision.sh
   ```

4. 視窗中按 `b` 後點球取樣顏色，再按 `c` 依序點平台左上、右上、右下、左下四角。
5. 確認綠色圓圈穩定包住球後，再按 `r` 開始 PID。按 `q` 或 `Esc` 會安全停止。

## 安全與調參

先在空載與小角度下測試 Servo 中立角和方向。PID 應先以單軸、`Ki=0` 和很小的 `Kp` 開始；確認方向正確後才逐步提高 `Kp`，再少量加入 `Kd`。不同的連桿、平台、球體和攝影機延遲都需要重新調參。

`Camera_Vision/camera_config.json` 包含現場校正值；若鏡頭或平台移動，請重新以 `c` 進行四角校正。
