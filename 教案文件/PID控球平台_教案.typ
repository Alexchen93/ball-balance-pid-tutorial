// 依據「PID控球平台_教案.docx」重新排版之 Typst 版本
#set page(
  paper: "a4",
  margin: (left: 0.9cm, right: 0.9cm, top: 1.25cm, bottom: 1.0cm),
  header: context [
    #set text(size: 12pt)
    生活科技-教案
  ],
)
#set text(
  font: ("Noto Sans CJK TC", "Noto Sans CJK SC", "Droid Sans Fallback"),
  size: 12pt,
  lang: "zh",
)
#set par(leading: 0.7em)
#set table(stroke: 0.55pt, inset: (x: 3pt, y: 3pt), align: horizon)

#let shade = luma(232)
#let section-color = rgb("#3e6575")
#let cell(body, colspan: 1, rowspan: 1, fill: none, align: left) = table.cell(
  colspan: colspan,
  rowspan: rowspan,
  fill: fill,
  align: align + horizon,
)[#body]
#let head(body, colspan: 1, rowspan: 1, align: center) = table.cell(
  colspan: colspan,
  rowspan: rowspan,
  fill: shade,
  align: align + horizon,
)[#strong(body)]
#let section(n, title) = [
  #v(4pt)
  #text(size: 14pt, weight: "bold", fill: section-color)[#n. #title]
  #v(3pt)
]
#let unit-title(title) = table.cell(colspan: 4, fill: shade, align: center + horizon)[#text(size: 14pt, weight: "bold")[#title]]

#align(center)[
  #text(size: 18pt, weight: "bold")[所教案名稱：PID 自動平衡控球平台教案]
]
#align(right)[#text(size: 12pt, weight: "bold")[教學設計：陳子紘]]

#section(1, [教案概述])
#table(
  columns: (1.2fr, 2.1fr, 1.3fr, 2.0fr),
  head([領域/科目別]), cell([生活科技], colspan: 3, align: center),
  head([教學對象]), cell([高中], align: center), head([教學時數]), cell([400 分鐘(8 堂，每堂 50 分鐘)], align: center),
  head([教學設備]), cell([Arduino Nano、2 顆 Servo、USB 攝影機、PID 控球平台、球體、電腦、獨立電源與連接線；軟體使用 Arduino IDE，以及教師預先完成之影像辨識、主控制與硬體校正程式。], colspan: 3),
  head([摘要]), cell([本課程以「如何讓球自動穩定在平台目標位置」為核心問題。教師提供固定規格之平台材料、完整控制程式與硬體校正程式，學生不需從零撰寫控制程式，而是先理解回授控制與 PID 基本概念，再完成平台組裝、Arduino IDE 基本操作及系統校正。後續以 Kp、Ki、Kd 為主要實驗變因，依 P → PD → PID 的順序進行預測、修改、測試、紀錄與分析，最後利用定位誤差、穩定時間、最大偏移及重複性等指標進行正式驗證，培養學生以實驗證據進行工程判斷與系統調校的能力。], colspan: 3),
  head([學習內容]), cell([生 N-V-2 工程、科技、科學與數學的統整與應用。\
生 P-V-1 工程設計與實作。\
生 A-V-1 機構與結構的設計與應用。\
生 A-V-2 機電整合與控制的設計與應用。], colspan: 3),
  head([學習表現]), cell([※設 k-V-1 能了解工程與工程設計的基本知識。\
※設 s-V-3 能運用科技工具維修及調校科技產品。\
※設 c-V-1 能運用工程設計流程，規劃、分析並執行專案計畫以解決實務問題。\
運 t-V-2 能使用程式設計實現運算思維的解題方法。\
運 t-V-3 能應用運算思維評估解題方法的優劣。\
運 c-V-3 能整合適當的資訊科技與他人合作完成專題製作。], colspan: 3),
  head([學習目標]), cell([1. 能說明回授控制、目標值、實際值與誤差的基本概念。\
2. 能說明 P、I、D 三項控制作用及參數改變可能造成的控制現象。\
3. 能辨認攝影機、Arduino、Servo、平台與球體在閉迴路系統中的角色。\
4. 能依固定材料完成平台組裝，並操作 Arduino IDE 與教師提供之校正程式。\
5. 能依 P → PD → PID 的順序進行參數調整，每次留下預測、測試結果與修改理由。\
6. 能利用定位誤差、穩定時間、最大偏移與重複性比較不同參數的控制表現。\
7. 能區分硬體、感測、控制參數及操作因素所造成的問題。\
8. 能以實驗數據說明最終 PID 參數的選擇與改善方向。], colspan: 3),
  head([先備知識]), cell([具備基本電腦操作與用電安全概念即可；不要求具備 Arduino 程式開發、PID 控制或 OpenCV 經驗。學生須能進行基本數值比較與簡單座標、比例概念。], colspan: 3),
  head([議題融入], rowspan: 2), head([實質內涵], colspan: 2), cell([無], align: center),
  head([所融入之學習重點], colspan: 2), cell([無], align: center),
)

#section(2, [評量方式])
#table(
  columns: (0.9fr, 3.3fr, 2.8fr, 2.7fr),
  head([項次]), head([以學習表現作為評量標準]), head([對應之學習內容類別]), head([具體評量方式]),
  cell([1], align: center), [能說明回授控制、誤差及 P、I、D 的基本作用，並判讀常見控制現象。], [生 N-V-2、\生 A-V-2], [控制系統與 PID 概念學習單、課堂提問與情境判讀。],
  cell([2], align: center), [能完成平台組裝，正確操作 Arduino IDE 與教師提供之硬體校正程式。], [生 P-V-1、\生 A-V-1、\生 A-V-2], [平台組裝檢核、Arduino 操作與 Calibration PASS 校正紀錄。],
  cell([3], align: center), [能依控制變因原則進行 Kp、Kd、Ki 的循序調整，並以實驗結果決定下一步。], [生 N-V-2、\生 A-V-2], [PID 調參紀錄表：預測、參數、現象、數據、判斷與下一步。],
  cell([4], align: center), [能以固定條件進行重複測試，利用量化指標評估控制系統表現。], [生 P-V-1、\生 A-V-2], [至少 3 次正式驗證，記錄定位誤差、穩定時間、最大偏移及重複性。],
  cell([5], align: center), [能整合測試資料說明最終參數選擇、問題來源與後續改善方向。], [生 P-V-1、\生 A-V-2], [小組成果說明、組間比較與個人課程反思。],
)

#section(3, [課程設計架構圖])
#v(8pt)
#align(center)[
  #text(size: 12pt)[（本區暫留，後續改以樹狀圖呈現四週課程架構，不以文字表格呈現。）]
]
#v(8pt)

#section(4, [教學活動])
#table(
  columns: (1.35fr, 3.8fr, 2.45fr, 1.05fr),
  unit-title([單元一：球為什麼停不住？]),
  head([活動簡述]), cell([先以完成之控球平台建立學生對自動控制的具體印象。第 1 堂聚焦控制、回授、開迴路／閉迴路、目標值、實際值與誤差；第 2 堂再進入 P、I、D 的直觀意義與常見控制現象。此階段不要求推導微積分公式，而以「看懂系統、看懂現象、能做初步判斷」為主。], colspan: 3),
  head([時間]), cell([100 分鐘（2 堂，每堂 50 分鐘）], colspan: 3, align: center),
  head([學習表現]), cell([※設 k-V-1 能了解工程與工程設計的基本知識。\
運 t-V-3 能應用運算思維評估解題方法的優劣。], colspan: 3),
  head([學習內容]), cell([生 N-V-2 工程、科技、科學與數學的統整與應用。\
生 A-V-2 機電整合與控制的設計與應用。], colspan: 3),
  head([學習目標]), cell([1. 能說明控制、自動控制及回授的基本意義。\
2. 能分辨開迴路與閉迴路控制。\
3. 能說明目標值、實際值與誤差的關係。\
4. 能指出控球平台中的感測器、控制器、致動器與受控系統。\
5. 能以直觀方式說明 P、I、D 三項控制作用。\
6. 能根據反應慢、振盪、殘留誤差等現象，初步判斷參數可能的調整方向。], colspan: 3),
  head([教學活動\(名稱)]), head([活動內容(含時間分配)]), head([學生學習活動]), head([時間]),
  head([第 1 堂：控制與回授], colspan: 4),
  cell([任務導入], align: center), cell([示範無控制、控制不佳與穩定控制三種狀態，引導「硬體相同，為何結果不同？」]), cell([觀察差異並提出可能原因。]), cell([5 分鐘], align: center),
  cell([控制的意義], align: center), cell([由冷氣、定速巡航等生活案例說明控制與自動控制。]), cell([找出案例中的目標與被控制量。]), cell([8 分鐘], align: center),
  cell([開迴路與閉迴路], align: center), cell([比較固定 Servo 輸出與讀取球位置後持續修正的差異。]), cell([判斷案例屬於開迴路或閉迴路。]), cell([10 分鐘], align: center),
  cell([誤差與回授], align: center), cell([介紹目標值、實際值、誤差 e = r - y 與「量測 → 比較 → 修正 → 再量測」。]), cell([根據球的位置判讀誤差大小與方向。]), cell([10 分鐘], align: center),
  cell([系統角色], align: center), cell([介紹 Sensor、Controller、Actuator、Plant、Feedback。]), cell([將攝影機、Arduino、Servo、平台與球配對到控制系統角色。]), cell([10 分鐘], align: center),
  cell([統整], align: center), cell([建立控球平台閉迴路流程圖。]), cell([完成簡易控制方塊圖與課堂出口題。]), cell([7 分鐘], align: center),
  head([第 2 堂：P、I、D 原理], colspan: 4),
  cell([誤差複習], align: center), cell([快速回顧 e = r - y。]), cell([判讀 2 個球位置案例。]), cell([5 分鐘], align: center),
  cell([P 控制], align: center), cell([以「現在偏多少，就修正多少」說明比例控制與 Kp。]), cell([預測 Kp 太小、適中、太大的現象。]), cell([10 分鐘], align: center),
  cell([P 現象示範], align: center), cell([實機或影片呈現反應慢、適中與振盪。]), cell([依現象判斷 Kp 設定。]), cell([8 分鐘], align: center),
  cell([D 控制], align: center), cell([以「不只看偏多少，也看誤差改變得多快」說明 D。]), cell([比較慢慢接近與快速衝向中心兩種情境。]), cell([8 分鐘], align: center),
  cell([I 控制], align: center), cell([以持續存在的小偏差說明累積修正。]), cell([判斷何種情境可能需要 I。]), cell([8 分鐘], align: center),
  cell([PID 統整], align: center), cell([整理 P 看現在、I 看累積、D 看變化。]), cell([完成 P/I/D 與控制現象配對。]), cell([6 分鐘], align: center),
  cell([調參規則], align: center), cell([說明後續實驗採「一次只改一個主要參數」。]), cell([寫下第一個想驗證的 PID 假設。]), cell([5 分鐘], align: center),
)

#v(7pt)
#table(
  columns: (1.35fr, 3.8fr, 2.45fr, 1.05fr),
  unit-title([單元二：把控制系統組起來]),
  head([活動簡述]), cell([學生在已理解 PID 基本原理後，使用教師提供之固定規格材料完成雙軸控球平台組裝。課程不要求學生自行選材、重設機構或從零撰寫控制程式，而是著重各元件在閉迴路中的功能、Arduino IDE 基本操作，以及利用教師提供的硬體校正程式確認 Servo 中立角、方向、輸出範圍、攝影機辨識與 X/Y 座標。完成 Calibration PASS 後，才能進入下一單元調參。], colspan: 3),
  head([時間]), cell([100 分鐘（2 堂，每堂 50 分鐘）], colspan: 3, align: center),
  head([學習表現]), cell([※設 s-V-3 能運用科技工具維修及調校科技產品。\
運 t-V-2 能使用程式設計實現運算思維的解題方法。\
運 c-V-3 能整合適當的資訊科技與他人合作完成專題製作。], colspan: 3),
  head([學習內容]), cell([生 P-V-1 工程設計與實作。\
生 A-V-1 機構與結構的設計與應用。\
生 A-V-2 機電整合與控制的設計與應用。], colspan: 3),
  head([學習目標]), cell([1. 能辨認控球平台各硬體元件及其控制角色。\
2. 能依教師提供的固定材料與步驟完成平台組裝。\
3. 能完成 Arduino Nano、Servo 與其他元件的基本連接。\
4. 能操作 Arduino IDE 完成開發板、連接埠與程式燒錄。\
5. 能使用教師提供之校正程式檢查 Servo 中立角、方向與安全輸出範圍。\
6. 能完成攝影機與平台座標校正，確認 X/Y 位置資料與控制方向正確。], colspan: 3),
  head([教學活動\(名稱)]), head([活動內容(含時間分配)]), head([學生學習活動]), head([時間]),
  head([第 3 堂：平台組裝], colspan: 4),
  cell([材料與任務確認], align: center), cell([發放固定材料並展示完成品，說明本堂只進行正確組裝，不做機構改版。]), cell([清點零件並確認組裝任務。]), cell([5 分鐘], align: center),
  cell([元件角色], align: center), cell([說明 Arduino、Servo、攝影機、平台與連桿在控制系統中的角色。]), cell([將各元件配對至 Sensor、Controller、Actuator、Plant。]), cell([5 分鐘], align: center),
  cell([關鍵組裝示範], align: center), cell([示範 Servo、連桿與平台安裝方向及注意事項。]), cell([觀察並確認安裝方向。]), cell([8 分鐘], align: center),
  cell([平台組裝], align: center), cell([巡迴協助機構安裝與基本故障排除。]), cell([依固定材料完成平台、Servo 與連桿組裝。]), cell([22 分鐘], align: center),
  cell([接線與供電], align: center), cell([說明 Servo 供電、Arduino 連接與共地概念。]), cell([完成基本接線並交叉檢查。]), cell([5 分鐘], align: center),
  cell([組裝檢核], align: center), cell([依檢核表確認平台可正常活動、無明顯卡滯。]), cell([完成小組自評與組間互檢。]), cell([5 分鐘], align: center),
  head([第 4 堂：Arduino 與硬體校正], colspan: 4),
  cell([Arduino IDE], align: center), cell([示範 Board、Port、Verify 與 Upload 基本流程。]), cell([完成 Nano 連線設定。]), cell([7 分鐘], align: center),
  cell([學生參數區], align: center), cell([說明核心程式已由教師完成，指出後續學生主要修改的 Kp、Ki、Kd 區域。]), cell([在程式中找到可調整參數位置。]), cell([5 分鐘], align: center),
  cell([校正程式燒錄], align: center), cell([示範教師提供之 Calibration 程式與 Serial 115200 訊息。]), cell([完成燒錄並確認雙模式校正啟動。]), cell([6 分鐘], align: center),
  cell([Servo 中立與方向], align: center), cell([使用 MODE 1 微調 1° 檢查 X/Y 軸中立角與正負方向。]), cell([觀察平台反應並記錄 PASS/FAIL。]), cell([10 分鐘], align: center),
  cell([安全輸出範圍], align: center), cell([短按 SW 切換 MODE 2；目前中心 90、範圍 ±20，端點推導為 70–110，長按回中心。]), cell([確認平台不碰撞、不超出安全範圍。]), cell([5 分鐘], align: center),
  cell([攝影機校正], align: center), cell([示範球體辨識與平台四角校正。]), cell([完成球體與平台座標校正。]), cell([7 分鐘], align: center),
  cell([X/Y 座標確認], align: center), cell([移動球體並示範座標及控制方向判讀。]), cell([觀察 Serial 資料並確認 X/Y 方向。]), cell([5 分鐘], align: center),
  cell([Calibration PASS], align: center), cell([依校正檢核表確認所有項目。]), cell([完成校正紀錄，通過者進入 PID 調參。]), cell([5 分鐘], align: center),
)

#v(7pt)
#table(
  columns: (1.35fr, 3.8fr, 2.45fr, 1.05fr),
  unit-title([單元三：一次只改一個參數]),
  head([活動簡述]), cell([學生使用已完成組裝並通過 Calibration PASS 的固定平台，以 Kp、Kd、Ki 為主要實驗變因。第 5 堂只聚焦 P-only，找出合理 Kp 範圍；第 6 堂在候選 Kp 上加入 Kd，再視持續誤差加入少量 Ki。每次修改皆須留下「預測 → 參數 → 測試 → 現象 → 數據 → 判斷 → 下一步」，避免無依據的亂調參。], colspan: 3),
  head([時間]), cell([100 分鐘（2 堂，每堂 50 分鐘）], colspan: 3, align: center),
  head([學習表現]), cell([※設 s-V-3 能運用科技工具維修及調校科技產品。\
※設 c-V-1 能運用工程設計流程，規劃、分析並執行專案計畫以解決實務問題。\
運 t-V-2 能使用程式設計實現運算思維的解題方法。\
運 t-V-3 能應用運算思維評估解題方法的優劣。], colspan: 3),
  head([學習內容]), cell([生 N-V-2 工程、科技、科學與數學的統整與應用。\
生 P-V-1 工程設計與實作。\
生 A-V-2 機電整合與控制的設計與應用。], colspan: 3),
  head([學習目標]), cell([1. 能在固定測試條件下依序進行 P → PD → PID 調整。\
2. 能預測 Kp、Kd、Ki 改變後可能造成的控制現象。\
3. 能記錄每次參數修改的理由、測試結果與下一步。\
4. 能比較 P、PD、PID 在反應速度、振盪、殘留誤差與穩定性上的差異。\
5. 能選定一組可進入正式驗證的候選 PID 參數。], colspan: 3),
  head([教學活動\(名稱)]), head([活動內容]), head([學生學習活動]), head([時間]),
  head([第 5 堂：P-only 調參], colspan: 4),
  cell([校正確認], align: center), cell([確認平台已通過校正，說明本堂固定硬體與其他參數。]), cell([完成實驗前檢核。]), cell([5 分鐘], align: center),
  cell([統一測試條件], align: center), cell([固定球體、起始區域、目標位置與測試方式。]), cell([記錄本組固定測試條件。]), cell([5 分鐘], align: center),
  cell([Kp 預測], align: center), cell([回顧 Kp 對修正量的影響。]), cell([預測 Kp 增加後反應速度與振盪的變化。]), cell([5 分鐘], align: center),
  cell([P-only 循序測試], align: center), cell([由較小 Kp 開始，要求其他參數保持不變。]), cell([逐次修改 Kp，至少完成 3～4 組測試並記錄現象。]), cell([20 分鐘], align: center),
  cell([P 結果分析], align: center), cell([引導辨認 Kp 太小、合理與過大的特徵。]), cell([比較數據並圈選合理 Kp 範圍。]), cell([8 分鐘], align: center),
  cell([候選 Kp], align: center), cell([要求各組以證據選出候選值。]), cell([寫下選擇理由並保存紀錄。]), cell([5 分鐘], align: center),
  cell([收尾], align: center), cell([提醒保存程式版本及實驗表。]), cell([整理器材與資料。]), cell([2 分鐘], align: center),
  head([第 6 堂：PD → PID 調參], colspan: 4),
  cell([PD 問題回顧], align: center), cell([從第 5 堂的超越、振盪或快速衝過目標等現象導入 D。]), cell([說明目前 P-only 尚未解決的問題。]), cell([5 分鐘], align: center),
  cell([Kd 預測], align: center), cell([回顧 D 對誤差變化趨勢的反應。]), cell([預測加入 Kd 後的可能結果。]), cell([5 分鐘], align: center),
  cell([PD 循序測試], align: center), cell([固定候選 Kp，逐步增加 Kd。]), cell([完成 2～3 組 PD 測試，記錄振盪、超越與穩定性。]), cell([15 分鐘], align: center),
  cell([PD 分析], align: center), cell([引導選出候選 Kd。]), cell([比較數據並說明選擇理由。]), cell([5 分鐘], align: center),
  cell([I 項導入], align: center), cell([針對持續存在的小誤差介紹積分作用，強調 Ki 並非一定越大越好。]), cell([判斷自己的平台是否需要加入 I。]), cell([5 分鐘], align: center),
  cell([PID 測試], align: center), cell([在 PD 基礎上加入少量 Ki，必要時僅做小幅微調。]), cell([完成 1～2 組 PID 測試並記錄結果。]), cell([10 分鐘], align: center),
  cell([候選參數確認], align: center), cell([要求各組建立正式驗證用參數。]), cell([登錄候選 Kp、Ki、Kd 及其選擇理由。]), cell([5 分鐘], align: center),
)

#v(7pt)
#table(
  columns: (1.35fr, 3.8fr, 2.45fr, 1.05fr),
  unit-title([單元四：哪一組參數真的比較好？]),
  head([活動簡述]), cell([本單元以「如何證明這組 PID 真的比較好」為核心。第 7 堂固定平台、程式、球體、起始區域與候選參數，進行至少 3 次重複測試，利用定位誤差、穩定時間、最大偏移及重複性整理控制表現；第 8 堂進行統一條件下的正式挑戰、組間比較與成果說明，使學生以實驗證據而非主觀感覺說明參數選擇。], colspan: 3),
  head([時間]), cell([100 分鐘（2 堂，每堂 50 分鐘）], colspan: 3, align: center),
  head([學習表現]), cell([※設 c-V-1 能運用工程設計流程，規劃、分析並執行專案計畫以解決實務問題。\
運 t-V-3 能應用運算思維評估解題方法的優劣。\
運 c-V-3 能整合適當的資訊科技與他人合作完成專題製作。], colspan: 3),
  head([學習內容]), cell([生 N-V-2 工程、科技、科學與數學的統整與應用。\
生 P-V-1 工程設計與實作。\
生 A-V-2 機電整合與控制的設計與應用。], colspan: 3),
  head([學習目標]), cell([1. 能說明一次成功不能代表控制系統具穩定性與可靠性。\
2. 能在固定條件下完成至少 3 次重複測試。\
3. 能記錄並比較定位誤差、穩定時間、最大偏移與重複性。\
4. 能依測試現象區分硬體、感測、控制參數與操作問題。\
5. 能以調參紀錄與正式測試數據說明最終 PID 參數的優缺點及改善方向。], colspan: 3),
  head([教學活動\(名稱)]), head([活動內容(含時間分配)]), head([學生學習活動]), head([時間]),
  head([第 7 堂：重複測試與數據分析], colspan: 4),
  cell([驗證概念], align: center), cell([說明一次成功不能代表系統可靠，介紹重複測試的重要性。]), cell([說明為何同一組參數必須重複測試。]), cell([5 分鐘], align: center),
  cell([性能指標], align: center), cell([介紹定位誤差、穩定時間、最大偏移與重複性，說明量測方式。]), cell([完成性能指標紀錄欄位與判讀方式。]), cell([8 分鐘], align: center),
  cell([測試規範], align: center), cell([固定球體、平台、程式、起始區域、目標位置及 PID 參數。]), cell([完成正式測試前檢核。]), cell([5 分鐘], align: center),
  cell([重複測試], align: center), cell([依統一流程進行測試並維持參數不變。]), cell([完成至少 3 次測試，記錄各次定位與時間資料。]), cell([20 分鐘], align: center),
  cell([數據整理], align: center), cell([示範平均值與測試差異的整理方式。]), cell([計算平均結果並比較三次表現。]), cell([7 分鐘], align: center),
  cell([問題診斷], align: center), cell([引導區分機構、感測、PID 參數及操作因素。]), cell([判斷目前系統最大的限制並留下證據。]), cell([5 分鐘], align: center),
  head([第 8 堂：正式挑戰、成果說明與反思], colspan: 4),
  cell([最終參數確認], align: center), cell([確認各組正式挑戰前停止任意修改參數。]), cell([登錄最終 Kp、Ki、Kd 與系統狀態。]), cell([5 分鐘], align: center),
  cell([挑戰規則], align: center), cell([說明目標位置、容許範圍、起始條件、測試次數與安全規則。]), cell([完成設備與測試條件確認。]), cell([5 分鐘], align: center),
  cell([正式挑戰], align: center), cell([各組依統一條件進行正式測試。]), cell([完成正式測試並保存數據。]), cell([15 分鐘], align: center),
  cell([成績整理], align: center), cell([彙整各組定位誤差、穩定時間、最大偏移與重複性。]), cell([整理本組結果並與其他組比較。]), cell([8 分鐘], align: center),
  cell([工程成果說明], align: center), cell([要求回答「做了什麼、證據是什麼、為什麼有效」。]), cell([以 PID 調參紀錄與測試數據說明參數選擇。]), cell([10 分鐘], align: center),
  cell([控制知識統整], align: center), cell([回到 P、I、D 功能，連結第一週理論與實際現象。]), cell([用自己的測試案例說明 P、I、D 的作用。]), cell([4 分鐘], align: center),
  cell([課程反思], align: center), cell([統整「理解 → 組裝校正 → 調參 → 驗證」歷程。]), cell([寫下最有效修改、一次失敗原因及若再次實作會如何改善。]), cell([3 分鐘], align: center),
)

#pagebreak()
#section(5, [附錄])
#text(size: 10pt, weight: "bold")[1. 學習單]

#v(10pt)
#text(size: 10pt, weight: "bold")[2. 評量工具]

#v(4pt)
#table(
  columns: (2.5fr, 1.2fr, 5fr),
  head([評量名稱]), head([占比]), head([主要內容]),
  [控制系統與 PID 基本知識], [20%], [回授、目標值／實際值／誤差、P／I／D 基本作用與控制現象判讀。],
  [Arduino 與系統校正操作], [10%], [Arduino IDE、程式燒錄、Servo 中立與方向、攝影機座標校正及 Calibration PASS。],
  [PID 參數調整紀錄], [30%], [Kp、Kd、Ki 的預測、修改、測試、現象、數據、判斷及下一步；重視調整理由而非最終數值。],
  [正式系統測試], [20%], [固定條件下至少 3 次重複測試；定位誤差、穩定時間、最大偏移與重複性。],
  [數據分析與工程判斷], [15%], [比較 P、PD、PID 表現，區分硬體、感測、控制參數與操作因素，說明最終參數選擇。],
  [成果說明與反思], [5%], [以實驗證據說明成果、失敗原因與後續改善方向。],
)
