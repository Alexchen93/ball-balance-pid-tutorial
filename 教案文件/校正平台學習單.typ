#set page(
  paper: "a4",
  flipped: false,
  margin: (top: 1.05cm, bottom: 1.05cm, left: 1.15cm, right: 1.15cm),
)
#set text(font: ("Noto Sans CJK TC", "Noto Sans"), size: 8.45pt, lang: "zh")
#set par(leading: 0.48em, justify: false)

#let check(label) = [#box(width: 0.32cm, height: 0.32cm, stroke: 0.65pt + black) #label]
#let field(label, width: 3.4cm) = [#text(weight: "medium")[#label：]#line(length: width, stroke: 0.5pt + gray)]
#let section(title) = {
  v(0.38em)
  text(size: 10.6pt, weight: "bold")[#title]
  line(length: 100%, stroke: 0.65pt + black)
  v(0.18em)
}
#let box_note(height: 0.9cm) = block(width: 100%, height: height, stroke: 0.5pt + gray, inset: 3.5pt)
#let ccell(body) = table.cell(align: center)[#body]

#align(center)[#text(size: 15.5pt, weight: "bold")[平衡球 PID 校正紀錄單]]


#v(0.25em)
#grid(
  columns: (1fr, 1fr, 1fr, 1fr, 1fr),
  gutter: 0.65em,
  field([班級], width: 1.5cm),
  field([組別], width: 1.5cm),
  field([組員 1], width: 1.8cm),
  field([組員 2], width: 1.8cm),
  field([組員 3], width: 1.8cm),
)

#section[1. 接線表]
#align(center)[
  #table(
    columns: (2.25cm, 2.55cm, 1fr, 2.0cm),
    inset: 3.2pt,
    stroke: 0.5pt + gray,
    align: (left, left, left, center),
    ccell[項目], ccell[Arduino / 電源], ccell[接線內容], ccell[確認],
    ccell[搖桿 X], [A0], [VRx → A0], [□],
    ccell[搖桿 Y], [A1], [VRy → A1], [□],
    ccell[搖桿按鍵], [D2], [SW → D2], [□],
    ccell[搖桿電源], [5V / GND], [VCC → 5V；GND → GND], [□],
    ccell[伺服 X], [D9], [訊號線 → D9], [□],
    ccell[伺服 Y], [D10], [訊號線 → D10], [□],
    ccell[伺服電源], [獨立 5–6V], [伺服電源不可吃 Nano 5V；伺服 GND 與 Nano GND 共地], [□],
  )
]

#grid(
  columns: (1fr, 1fr),
  gutter: 0.8em,
  check[上電前平台、連桿、伺服臂沒有卡住],
  check[測試時手指離開連桿與伺服臂],
)

#text(size: 8.0pt)[序列監控設 #raw("115200") baud；開機進入 #raw("MODE 1: MICRO STEP (1 degree)")。短按 SW 切換 #raw("MODE 2: ENDPOINT")；目前中心 90、範圍 ±20，端點推導為 70–110。長按 SW 回中心；每次推桿只接受單一主要軸，回中後才可再動。]
#grid(
  columns: (1fr, 1fr),
  gutter: 0.8em,
  check[已看見 MODE 1／MODE 2 訊息與推導後端點 70–110],
  check[MODE 1 每次只微調 1°；MODE 2 直達安全端點],
)

#section[2. 中心軸記錄]
#text(size: 8.2pt)[完成機構置中後，在序列監控送出 #raw("SETC")，再送出 #raw("SHOW")；把本次顯示值填入下表並轉填至韌體。]

#align(center)[
  #table(
    columns: (3.2cm, 2.4cm, 3.0cm, 1fr),
    inset: 3.2pt,
    stroke: 0.5pt + gray,
    ccell[項目], ccell[SHOW 值], ccell[韌體轉填值], ccell[完整常數列],
    ccell[#raw("SERVO_X_CENTER")], [], [], [#raw("constexpr int SERVO_X_CENTER = _____;")],
    ccell[#raw("SERVO_Y_CENTER")], [], [], [#raw("constexpr int SERVO_Y_CENTER = _____;")],
  )
]

#text(weight: "bold")[SHOW 原始輸出抄錄]
#box_note(height: 0.82cm)

#section[3. 最小／最大角度記錄]
#text(size: 8.2pt)[目前韌體以 #raw("SERVO_X_CENTER=90") 與 #raw("SERVO_LIMIT_OFFSET_DEG=20") 推導端點為 70～110。下表填「實測安全值」；若碰撞、拉扯、異音或超出安全範圍，立即回中心並縮小 offset。]

#align(center)[
  #table(
    columns: (1.25cm, 2.1cm, 2.1cm, 2.5cm, 2.55cm, 1fr),
    inset: 3pt,
    stroke: 0.5pt + gray,
    ccell[軸], ccell[目前 MIN], ccell[目前 MAX], ccell[實測安全 MIN], ccell[實測安全 MAX], ccell[觀察／處理],
    ccell[X], [70°], [110°], [], [], [],
    ccell[Y], [70°], [110°], [], [], [],
  )
]

#grid(
  columns: (1fr, 1fr),
  gutter: 0.8em,
  check[端點測試後已回到中心],
  check[安全端點已決定是否需要覆寫韌體常數],
)

#align(center)[
  #table(
    columns: (3.2cm, 2.4cm, 3.0cm, 1fr),
    inset: 3.2pt,
    stroke: 0.5pt + gray,
    ccell[項目], ccell[目前值], ccell[本次實測安全值], ccell[完整常數列],
    ccell[#raw("SERVO_LIMIT_OFFSET_DEG")], [20], [], [#raw("constexpr int SERVO_LIMIT_OFFSET_DEG = _____;")],
    ccell[#raw("推導端點")], [70～110], [], [#raw("SERVO_X_CENTER ± SERVO_LIMIT_OFFSET_DEG")],
  )
]

#section[4. 轉入下一校正內容]
#text(size: 8.2pt)[完成機構校正後，只依課程指示切換至下一階段；下一階段可能是感測器、相機或 PID，本單不臆測細節。]

#align(center)[
  #table(
    columns: (0.8cm, 1fr, 2.15cm),
    inset: 3.2pt,
    stroke: 0.5pt + gray,
    align: (center, left, center),
    ccell[順序], ccell[操作步驟], ccell[完成],
    ccell[1], [確認平台已置中，球盤、連桿與伺服在安全範圍內。], [□],
    ccell[2], [在韌體填入並儲存本單記錄的 #raw("SERVO_X_CENTER")／#raw("SERVO_Y_CENTER") 與 #raw("SERVO_LIMIT_OFFSET_DEG")。], [□],
    ccell[3], [重新燒錄 Arduino Nano。], [□],
    ccell[4], [重新開機，確認平台水平且沒有異音、拉扯或卡住。], [□]
  )
]
