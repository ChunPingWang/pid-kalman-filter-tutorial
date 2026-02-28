# PID Controller & Kalman Filter Tutorial

PID 控制器與卡爾曼濾波器的嵌入式系統教學專案，包含理論基礎、跨平台實作與驗證測試。

---

## 學習路徑

```
Level 1: PID 基礎          -> PoC 1 (溫度控制)
Level 2: Kalman Filter 基礎 -> PoC 2 (感測器濾波)
Level 3: PID + KF 整合      -> PoC 3 (馬達控制)
Level 4: EKF 進階           -> PoC 4 (姿態估測)
Level 5: 進階應用 — 倒單擺  -> 完整指南 (物理建模 → LQR → 實作)
```

---

## 什麼是 PID Controller？

### 生活化比喻

想像你在洗澡調水溫：

- **P（比例）**：水太冷，你把熱水轉大。差距越大，你轉得越多。
- **I（積分）**：水一直微微偏冷，你慢慢地再多轉一點點，直到剛好。
- **D（微分）**：溫度正在快速升高，你預判它會太燙，提前把熱水轉小。

這就是 PID 的三個要素 — 回應當下、修正過去、預測未來。

### 數學公式

```
u(t) = Kp * e(t) + Ki * integral(e) dt + Kd * de(t)/dt

e(t) = 目標值 (setpoint) - 實際測量值 (measurement)
```

| 項目 | 作用 | 太大的後果 | 太小的後果 |
|------|------|-----------|-----------|
| **Kp** (比例增益) | 依誤差大小調整輸出 | 振盪不穩 | 回應遲鈍 |
| **Ki** (積分增益) | 消除長期穩態誤差 | 超調嚴重、反應慢 | 殘留穩態誤差 |
| **Kd** (微分增益) | 抑制變化速度、減少超調 | 對雜訊敏感 | 超調明顯 |

### 離散化實作（嵌入式系統使用）

嵌入式系統以固定取樣週期 `dt` 運行：

```c
e[k] = setpoint - measurement[k]

P_term = Kp * e[k]                          // 比例項
I_term = Ki * sum(e[i] * dt)                 // 積分項（累加近似）
D_term = -Kd * (measurement[k] - measurement[k-1]) / dt  // 微分項（對測量值微分）

output = P_term + I_term + D_term
```

> 注意：微分項使用 **Derivative on Measurement**（對測量值微分），而非對誤差微分，可避免 setpoint 突變時產生的 derivative kick（輸出瞬間跳動）。

### 常見問題與對策

| 問題 | 說明 | 解法 |
|------|------|------|
| **Integral Windup** | 執行器飽和時積分持續累加，解除飽和後嚴重超調 | Anti-Windup Clamping：限制積分項上下限 |
| **Derivative Kick** | setpoint 突然改變，微分項產生巨大脈衝 | 改為對 measurement 微分 |
| **穩態誤差** | 只用 P 控制時，輸出永遠追不到目標 | 加入 I 項消除 |
| **輸出振盪** | Kp 太大導致系統來回擺盪 | 降低 Kp，加入 Kd 增加阻尼 |

---

## 什麼是 Kalman Filter？

### 生活化比喻

想像你開車看 GPS：

- **GPS 顯示**你在 A 位置（但 GPS 有誤差，可能偏 5 公尺）
- **車速表 + 方向盤**告訴你，根據上一秒的位置和速度，你「應該」在 B 位置
- **Kalman Filter** 的工作：結合這兩個資訊，給你一個比任何單一來源都更準確的位置估計

核心思想：**把「不太準的預測」和「不太準的觀測」以最佳權重融合在一起。**

### 五步驟演算法

```
── 預測 (Predict) ──────────────────
Step 1: 狀態預測     x_pred = F * x_est + B * u
Step 2: 協方差預測   P_pred = F * P * F^T + Q

── 更新 (Update) ──────────────────
Step 3: 卡爾曼增益   K = P_pred * H^T / (H * P_pred * H^T + R)
Step 4: 狀態更新     x_est = x_pred + K * (z - H * x_pred)
Step 5: 協方差更新   P = (I - K * H) * P_pred
```

### 一維簡化版（最容易理解）

對於單一變數（例如溫度），所有矩陣退化為純量：

```c
// Predict
x_pred = x_est;                      // 假設狀態不變
p_pred = p_est + q;                   // 不確定性增加

// Update
k = p_pred / (p_pred + r);           // 計算卡爾曼增益
x_est = x_pred + k * (z - x_pred);   // 融合觀測
p_est = (1 - k) * p_pred;            // 更新不確定性
```

### 直覺理解卡爾曼增益 K

```
K = P / (P + R)

K 接近 1 → 信任感測器（R 小，感測器精確）
K 接近 0 → 信任預測模型（R 大，感測器雜訊大）
```

| 情境 | K 值 | 意義 |
|------|------|------|
| 高精度感測器 + 不確定的模型 | K -> 1 | 聽感測器的 |
| 低精度感測器 + 可靠的模型 | K -> 0 | 聽模型的 |
| 兩者差不多 | K ~ 0.5 | 各取一半 |

### Q 和 R 怎麼調？

| 參數 | 增大效果 | 減小效果 | 如何取得 |
|------|---------|---------|---------|
| **Q**（過程雜訊） | 更信任觀測，追蹤快但雜訊多 | 更信任模型，平滑但追蹤慢 | 實驗調整，從小值開始 |
| **R**（觀測雜訊） | 更信任模型，平滑但追蹤慢 | 更信任觀測，追蹤快但雜訊多 | 感測器 datasheet 或靜態量測 |

> 經驗法則：**Q/R 比值**決定濾波器行為。比值大 = 追蹤快；比值小 = 更平滑。

---

## PID + Kalman Filter 為什麼要結合？

在真實世界中，感測器讀數永遠帶有雜訊。如果直接把雜訊訊號送入 PID：

```
沒有 Kalman Filter:
  雜訊感測值 -> PID -> 控制輸出劇烈震盪 -> 執行器壽命縮短、系統不穩定

有 Kalman Filter:
  雜訊感測值 -> Kalman Filter -> 乾淨的估測值 -> PID -> 平滑穩定的控制輸出
```

### 整合架構

```
                    +---------------------+
                    |    System Plant     |
                    | (馬達/加熱器/機器人) |
                    +--------+------------+
                             | 實際狀態 (含雜訊)
                             v
                    +------------------+
                    |   Sensor(s)      |
                    |  z = H*x + v     |
                    +--------+---------+
                             | z[k] (原始觀測)
                             v
                    +------------------+
                    | Kalman Filter    |
                    | predict + update |----> 估測狀態 x_hat (乾淨)
                    +------------------+           |
                                                   v
              setpoint ---> +------------------+
                            | PID Controller   |
                            | e = sp - x_hat   |
                            +--------+---------+
                                     | u[k] (控制輸出)
                                     v
                            +------------------+
                            |   Actuator       |
                            |   (PWM/DAC)      |
                            +--------+---------+
                                     |
                                     +---> 回到 System Plant
```

### 每個控制週期的時序

```
1. 讀取感測器原始值 z[k]
2. Kalman Predict: 用上一步控制輸出 u[k-1] 預測狀態
3. Kalman Update:  用 z[k] 修正預測，得到乾淨的 x_hat[k]
4. PID Compute:    e[k] = setpoint - x_hat[k]，計算 u[k]
5. 輸出 u[k] 到執行器
6. 等待下一個取樣週期
```

---

## 實際應用場景

### 場景 1：智慧恆溫箱（PoC 1）

**情境**：3D 列印機的加熱床需要精確維持 60°C

```
問題：NTC 溫度感測器讀數跳動 +-2°C
     直接用 PID 控制 -> 加熱器頻繁開關 -> 溫度波動大

解法：
  NTC 讀數 -> Kalman Filter (消除跳動) -> PID -> MOSFET 控制加熱器
  結果：溫度穩定在 60 +- 0.3°C
```

**適用場景**：烤箱、恆溫水槽、PCB 回焊爐、孵蛋器

### 場景 2：自走車循線（PoC 2 + PoC 3）

**情境**：機器人沿黑線行駛，紅外線感測器受環境光干擾

```
問題：感測器受日光/燈光干擾，誤判線的位置
     馬達轉速編碼器有量化雜訊

解法：
  IR 感測器 -> Kalman Filter -> 估測線的真實位置
  Encoder   -> Kalman Filter -> 估測真實轉速
  兩個乾淨的訊號 -> 雙迴路 PID -> 左右馬達控制

  外迴路 PID：控制行駛方向（追蹤線的位置）
  內迴路 PID：控制馬達轉速（精確執行方向指令）
```

**適用場景**：AGV 無人搬運車、掃地機器人、倉儲物流機器人

### 場景 3：無人機姿態穩定（PoC 4）

**情境**：四軸飛行器需要維持水平穩定飛行

```
問題：
  加速度計 -> 受震動影響，短期雜訊大，長期準確
  陀螺儀   -> 短期精確，但長期會漂移 (drift)

解法：
  EKF 融合兩個感測器的優點：
  - 短期：信任陀螺儀（快速追蹤姿態變化）
  - 長期：用加速度計修正漂移

  EKF 輸出 (roll, pitch) -> PID -> 四個馬達的 PWM 分配
```

**適用場景**：無人機、自平衡機器人、船舶穩定系統、VR 頭顯追蹤

### 場景 3.5：倒單擺平衡控制（Level 5）

**情境**：將安裝在可移動滑車上的擺桿穩定在鉛直向上的不穩定平衡點

```
問題：
  系統天生不穩定（開迴路極點在右半平面）
  PID 只能處理小角度穩定，無法兼顧位置與角度
  感測器（IMU + 編碼器）各有雜訊與漂移

解法：
  Phase 1：能量法 Swing-Up 控制 — 將擺桿從靜止擺到接近直立
  Phase 2：LQR 全狀態回授 — 在直立附近進行最佳穩定控制
  Phase 3：Kalman Filter 感測器融合 — 提供乾淨的狀態估測

  完整控制架構：
  IMU + Encoder -> Kalman Filter -> [Swing-Up | LQR 自動切換] -> 馬達 PWM
```

**適用場景**：控制理論教學、機器人平衡、火箭著陸姿態控制

> 📖 完整指南請參考 [Inverted_Pendulum_Guide.md](Inverted_Pendulum_Guide.md)

### 場景 4：工業馬達精確控速

**情境**：CNC 加工機的主軸馬達需要穩定在 12000 RPM

```
問題：
  光學編碼器在低速時解析度不足，高速時有計數遺漏
  負載變化（切削力度不同）造成轉速波動

解法：
  Encoder 脈衝 -> 2D Kalman Filter (估測速度+加速度)
  -> PID 控制器 (Kp=1.0, Ki=0.5, Kd=0.05)
  -> PWM 驅動 H-Bridge

  2D Kalman 的優勢：同時估測速度和加速度
  -> 能預測下一步的速度變化
  -> PID 獲得更平滑的回饋訊號
```

**適用場景**：CNC 加工、紡織機、傳送帶、電動車馬達控制

### 場景 5：水位控制系統

**情境**：水塔自動補水，維持水位在 80%

```
系統組成：
  超音波水位感測器 (有回波干擾)
  電動閥門 (PWM 控制開度)

控制流程：
  超音波感測器 -> Kalman Filter -> 真實水位估測
  -> PID (setpoint=80%) -> 閥門開度控制

PID 調參考慮：
  - 水位變化慢 -> 用較小的 Kp，避免閥門頻繁動作
  - 需要精確到位 -> Ki 不可省略
  - 水位不太會突然變化 -> Kd 可以設很小或為 0
```

### 更多應用一覽

| 領域 | 應用 | 使用的技術 |
|------|------|-----------|
| 智慧農業 | 溫室溫濕度控制 | PID + KF 1D |
| 智慧家居 | 冷氣恆溫控制 | PID |
| 醫療設備 | 呼吸器壓力控制 | PID + KF 1D |
| 機器人 | 機械手臂關節控制 | PID + KF 2D |
| 航太 | 衛星姿態控制 | PID + EKF |
| 電力系統 | 太陽能追日系統 | PID + KF 1D |
| 汽車 | 定速巡航 (ACC) | PID + KF 2D |
| 化工 | 反應釜溫度/壓力控制 | 多迴路 PID + KF |
| 控制教學 | 倒單擺平衡控制 | PID + LQR + EKF |

---

## PID 調參實戰指南

### 方法一：手動調參（推薦初學者）

```
步驟 1：先只用 P
  設 Ki=0, Kd=0
  從小的 Kp 開始，慢慢增加
  直到系統開始輕微振盪 -> 記錄此時的 Kp

步驟 2：加入 D
  保持 Kp 不變（或略減 20%）
  慢慢增加 Kd
  直到振盪消失，系統平滑到達目標

步驟 3：加入 I
  慢慢增加 Ki
  觀察穩態誤差是否消除
  如果出現低頻振盪，降低 Ki
```

### 方法二：Ziegler-Nichols 法

Ziegler-Nichols 法是 1942 年由 John G. Ziegler 與 Nathaniel B. Nichols 在 ASME Transactions 發表的經典 PID 調參方法。至今仍是工業界最廣泛使用的 PID 調參起點，其核心思想是透過簡單的實驗量測系統特性，再查表得到 PID 參數的初始值。

#### 變體一：Ultimate Gain Method（臨界增益法 — 閉迴路實驗）

```
步驟 1：僅用 P 控制，增加 Kp 直到系統持續等幅振盪
步驟 2：記錄臨界增益 Ku 和振盪週期 Tu
步驟 3：查表計算：

| 控制器類型 | Kp          | Ki            | Kd            |
|-----------|-------------|---------------|---------------|
| P only    | 0.5 * Ku    | -             | -             |
| PI        | 0.45 * Ku   | 1.2 * Kp / Tu | -             |
| PID       | 0.6 * Ku    | 2 * Kp / Tu   | Kp * Tu / 8   |
```

#### 變體二：Step Response Method（階躍響應法 — 開迴路實驗）

當系統無法安全地進行持續振盪實驗時，可使用開迴路階躍響應：

```
步驟 1：系統在開迴路下施加階躍輸入
步驟 2：記錄 S 型響應曲線的兩個特徵參數：
        - L（延遲時間）：從階躍施加到響應開始上升的時間
        - T（時間常數）：響應從 L 到達 63.2% 穩態值的時間
步驟 3：查表計算：

| 控制器類型 | Kp        | Ti (積分時間) | Td (微分時間) |
|-----------|-----------|--------------|--------------|
| P only    | T / L     | -            | -            |
| PI        | 0.9 * T/L | L / 0.3      | -            |
| PID       | 1.2 * T/L | 2 * L        | 0.5 * L      |

其中 Ki = Kp / Ti，Kd = Kp * Td
```

#### 操作注意事項

- **安全考量**：臨界增益法需要系統持續等幅振盪，必須確保系統能承受（例如馬達不會損壞、溫度不會超過安全範圍）
- **穩定振盪判定**：調整 Ku 時需等待穩定的等幅振盪（至少觀察 5-10 個週期），過早記錄會導致參數不準確
- **數位系統取樣率**：控制迴路的取樣頻率至少應為振盪頻率（1/Tu）的 10 倍，否則離散化效應會使結果失真

#### Z-N 的局限性與改良

Ziegler-Nichols 法設計的目標是 **quarter-decay ratio**（每次振盪幅度衰減 75%），這意味著：
- 典型超調量約 25%，許多精密控制應用無法接受
- 對積分型系統（如水位控制）效果不佳
- 不適用於開迴路不穩定系統（如倒單擺）
- 對大延遲系統（L/T > 1）的效果較差

**Tyreus-Luyben 改良版**（更保守、超調更小）：

```
| 控制器類型 | Kp         | Ti           | Td           |
|-----------|------------|--------------|--------------|
| PI        | Ku / 3.2   | 2.2 * Tu     | -            |
| PID       | Ku / 2.2   | 2.2 * Tu     | Tu / 6.3     |
```

#### 與其他調參方法的比較

| 方法 | 優點 | 缺點 | 適用場景 |
|------|------|------|---------|
| Ziegler-Nichols | 簡單、不需系統模型 | 需容忍振盪、超調約 25% | 一般工業過程 |
| Cohen-Coon | 適合大延遲系統 | 需開迴路階躍測試 | 化工、熱處理 |
| Lambda Tuning | 可指定閉迴路時間常數 | 需系統模型（一階+延遲） | 需要無超調的系統 |
| SIMC (Skogestad) | 現代工業推薦標準、易用 | 稍複雜 | 通用推薦 |
| 手動調參 | 最靈活、適應任何系統 | 依賴經驗、耗時 | 初學者學習、特殊系統 |

### 調參檢查清單

```
[ ] P-only 控制：觀察穩態誤差存在
[ ] PI 控制：穩態誤差消除，但可能超調
[ ] PID 控制：超調減少，響應平滑
[ ] Anti-windup 測試：設極高 setpoint 再降回，觀察恢復時間
[ ] Derivative kick 測試：突然改變 setpoint，觀察輸出是否平滑
[ ] 干擾測試：施加外部干擾，觀察系統恢復能力
```

---

## Kalman Filter 調參指南

### 怎麼決定 R（觀測雜訊）？

```
方法：讓感測器靜止不動，連續讀取 100 筆數據
     計算標準差 std
     R = std * std（變異數）

範例：溫度感測器靜態讀數
  [25.1, 24.8, 25.3, 25.0, 24.9, 25.2, ...]
  平均值 = 25.05
  標準差 std = 0.15
  R = 0.15^2 = 0.0225
```

### 怎麼決定 Q（過程雜訊）？

```
Q 比較難直接量測，通常用實驗法：

1. 從 Q = R / 100 開始（非常信任模型）
2. 觀察濾波結果：
   - 如果追蹤太慢（真實值變了但估測跟不上）-> 增大 Q
   - 如果濾波後仍然雜訊明顯 -> 減小 Q
3. 反覆調整直到滿意

經驗值：
  - 穩定系統（溫度）: Q/R ~ 0.001 ~ 0.01
  - 中等變化（水位）: Q/R ~ 0.01 ~ 0.1
  - 快速變化（馬達轉速）: Q/R ~ 0.1 ~ 1.0
```

---

## Extended Kalman Filter (EKF) 簡介

### 什麼時候需要 EKF？

標準 Kalman Filter 假設系統是**線性**的。但真實世界很多系統是**非線性**的：

| 系統 | 為什麼是非線性？ |
|------|----------------|
| IMU 姿態估測 | 三角函數 (sin, cos, atan2) 轉換 |
| GPS 定位 | 經緯度到距離的轉換 |
| 機械手臂 | 多關節運動學 |
| 電池 SOC 估測 | 電池放電曲線非線性 |

### EKF 的做法

```
標準 KF：用固定的矩陣 F 和 H
EKF：   每一步重新計算 Jacobian 矩陣（一階泰勒展開線性化）

非線性模型：
  x[k] = f(x[k-1], u[k-1]) + w    // 非線性狀態轉移
  z[k] = h(x[k]) + v               // 非線性觀測

線性化：
  F[k] = df/dx |_{x=x_hat}         // 狀態轉移 Jacobian
  H[k] = dh/dx |_{x=x_hat}         // 觀測 Jacobian
```

### 本專案的 EKF 姿態估測

```
狀態向量：[roll, pitch, gyro_bias_x, gyro_bias_y]

預測（使用陀螺儀）：
  roll  += (gyro_x - bias_x) * dt
  pitch += (gyro_y - bias_y) * dt

更新（使用加速度計）：
  accel_roll  = atan2(ay, az)
  accel_pitch = atan2(-ax, sqrt(ay^2 + az^2))
  用加速度計的角度修正預測值
```

優勢：陀螺儀短期準確 + 加速度計長期穩定 = 最佳互補融合

---

## Level 5：倒單擺控制系統（進階應用）

完成 Level 1-4 後，倒單擺是整合所有知識的最佳挑戰。這個系統天生不穩定，必須依靠主動控制才能維持平衡，是驗證控制理論的經典載體。

### 為什麼是 Level 5？

| Level 1-4 基礎 | Level 5 對應應用 |
|----------------|-----------------|
| PID 控制器 (Level 1) | 小角度穩定的初步嘗試 → 理解 SISO 控制的極限 |
| Kalman Filter (Level 2) | IMU + 編碼器感測器融合 → 乾淨的狀態估測 |
| PID + KF 整合 (Level 3) | 感測器融合 → 控制器的完整閉迴路 |
| EKF 進階 (Level 4) | 非線性姿態估測 → 擺桿角度的精確追蹤 |
| **新增** | LQR 全狀態回授、Swing-Up 能量控制、LQG 分離設計 |

### 硬體方案摘要

- **主控**：ESP32（雙核 240MHz，Core 1 專用控制迴路 ≥500Hz）
- **感測器**：MPU6050 (IMU) + 600P/R 旋轉編碼器 + 馬達編碼器
- **驅動**：TB6612FNG + JGA25-371 DC 馬達 + GT2 同步帶
- **預估成本**：約 $70-100 USD

> 📖 完整的物理建模推導、LQR 設計、電路圖與實作指南：[Inverted_Pendulum_Guide.md](Inverted_Pendulum_Guide.md)
> 🔧 電路圖（DrawIO 格式）：[inverted_pendulum_circuit.drawio](inverted_pendulum_circuit.drawio)

---

## 系統架構

```
感測器(含雜訊) -> Kalman Filter(狀態估測) -> PID Controller(控制計算) -> 執行器(輸出)
     ^                                                                      |
     +--------------------------- 回饋迴路 ---------------------------------+
```

## 專案結構

```
pid-kalman-filter-tutorial/
|-- PID_KalmanFilter_Teaching_PoC.md   <- 完整教學文件 (理論 + 實作)
|-- README.md                          <- 本文件
|-- Inverted_Pendulum_Guide.md         <- Level 5: 倒單擺控制完整指南
|-- inverted_pendulum_circuit.drawio   <- Level 5: 電路圖 (DrawIO 格式)
|-- verify_all.sh                      <- 一鍵驗證腳本
|
|-- pid.h / pid.c                      <- PID 控制器 (Anti-Windup, Derivative on Measurement)
|-- kalman.h / kalman.c                <- Kalman Filter (1D + 2D)
|-- ekf_attitude.h / ekf_attitude.c    <- Extended Kalman Filter (4 狀態姿態估測)
|-- hal.h                              <- 硬體抽象層介面
|
|-- tests/
|   |-- test_pid.c                     <- PID 單元測試 (5 項)
|   |-- test_kalman.c                  <- Kalman Filter 單元測試 (4 項)
|   |-- test_integration.c             <- Kalman + PID 整合模擬
|   |-- test_ekf.c                     <- EKF 姿態估測測試 (3 項)
|   |-- benchmark.c                    <- 效能基準測試
|   +-- output/                        <- 測試輸出 CSV
|
|-- arduino/
|   |-- poc1_pid_temperature/          <- PoC 1: PID 溫度控制
|   |-- poc2_kalman_filter/            <- PoC 2: 1D Kalman Filter 濾波
|   |-- poc3_kalman_pid_motor/         <- PoC 3: Kalman + PID 馬達轉速控制
|   +-- poc4_ekf_imu/                  <- PoC 4: EKF + MPU6050 姿態估測
|
|-- rpi/                               <- Raspberry Pi (Python + C)
|-- stm32/                             <- STM32 (PlatformIO)
+-- beaglebone/                        <- BeagleBone Black (Linux sysfs)
```

## 四個 PoC 範例

| PoC | 主題 | 核心模組 | 硬體需求 | 難度 |
|-----|------|---------|---------|------|
| **PoC 1** | 純 PID 溫度控制 | `pid.c` | NTC Thermistor + MOSFET + 加熱線 | 入門 |
| **PoC 2** | 1D Kalman Filter 濾波 | `kalman.c` | 任意感測器（或純模擬） | 入門 |
| **PoC 3** | Kalman + PID 馬達控制 | `pid.c` + `kalman.c` | DC 馬達 + Encoder + H-Bridge | 中級 |
| **PoC 4** | EKF 姿態估測 | `ekf_attitude.c` | MPU6050 (I2C) | 進階 |
| **Level 5** | 倒單擺控制系統 | PID + LQR + EKF | ESP32 + MPU6050 + DC Motor + Encoder | 挑戰 |

## 支援平台

| 平台 | CPU | FPU | 浮點建議 |
|------|-----|-----|---------|
| Arduino UNO/Mega | ATmega328P/2560 16MHz | 無 | Fixed-point 或降低取樣率 |
| Raspberry Pi 4 | BCM2711 1.5GHz | 有 | float/double |
| STM32F4 | Cortex-M4 168MHz | 有 | float |
| BeagleBone Black | AM3358 1GHz | 有 | float/double |

---

## 快速開始

### 1. 執行驗證測試（不需硬體）

```bash
git clone https://github.com/ChunPingWang/pid-kalman-filter-tutorial.git
cd pid-kalman-filter-tutorial
chmod +x verify_all.sh
./verify_all.sh
```

### 2. 執行效能基準

```bash
gcc -Wall -O2 -o build/benchmark tests/benchmark.c pid.c kalman.c ekf_attitude.c -lm
./build/benchmark
```

### 3. 單獨執行測試

```bash
# PID 測試
gcc -Wall -O2 -o test_pid tests/test_pid.c pid.c -lm && ./test_pid

# Kalman Filter 測試
gcc -Wall -O2 -o test_kalman tests/test_kalman.c kalman.c -lm && ./test_kalman

# 整合模擬 (輸出 CSV)
gcc -Wall -O2 -o test_integration tests/test_integration.c pid.c kalman.c -lm
./test_integration > integration_results.csv

# EKF 測試
gcc -Wall -O2 -o test_ekf tests/test_ekf.c ekf_attitude.c -lm && ./test_ekf
```

### 4. Raspberry Pi 視覺化

```bash
cd rpi
make          # 編譯 C 共享函式庫
make run      # 執行 Python 模擬 + matplotlib 視覺化
```

---

## 驗證結果

所有測試皆通過:

```
=== PID & Kalman Filter PoC Verification ===

[1/5] Compiling modules...                    [OK]
[2/5] PID Unit Tests (5/5 passed)             [OK]
[3/5] Kalman Filter Unit Tests (4/4 passed)   [OK]
  - Noise reduction: 91.5% RMSE reduction
  - Kalman Gain convergence: 0.99 -> 0.10
[4/5] Kalman + PID Integration Simulation     [OK]
[5/5] EKF Attitude Tests (3/3 passed)         [OK]
  - Static level: roll=0.00, pitch=0.00
  - Tilted 30 deg: estimate=30.00
  - Gyro bias compensation: roll=0.11 (with bias=0.05 rad/s)

=== All verifications completed ===
```

---

## 故障排除

### PID 問題

| 症狀 | 可能原因 | 解決方案 |
|------|---------|---------|
| 持續振盪 | Kp 過大 | 降低 Kp，增加 Kd |
| 穩態誤差 | 無 I 項或 Ki 太小 | 增加 Ki |
| 超調後長時間回不來 | Integral windup | 啟用 anti-windup clamping |
| 輸出劇烈跳動 | Derivative kick | 改用 derivative on measurement |
| 回應太慢 | Kp 太小 | 逐步增加 Kp |

### Kalman Filter 問題

| 症狀 | 可能原因 | 解決方案 |
|------|---------|---------|
| 濾波後仍有很多雜訊 | Q 太大或 R 太小 | 減小 Q/R 比值 |
| 追蹤太慢（延遲大） | Q 太小或 R 太大 | 增大 Q/R 比值 |
| 估測值發散 | 數值不穩定 | 檢查 P 矩陣正定性，用 Joseph form |
| 初始收斂慢 | P0 太小 | 增大初始 P0 |

### Arduino 特定問題

- `float` 運算慢 -> 考慮 fixed-point (Q16.16 格式)
- RAM 不足 -> 減少 log buffer，用 `PROGMEM` 存常數
- Serial 輸出影響 timing -> 降低 baud rate 或減少輸出頻率

---

## 前置知識

**Level 1-4：**
- C/C++ 基礎程式設計
- 基本線性代數（矩陣運算）
- 基本機率統計（常態分佈、變異數）
- 嵌入式系統基礎（GPIO, ADC, PWM, I2C/SPI）

**Level 5 額外需要：**
- 拉格朗日力學基礎（廣義座標、Euler-Lagrange 方程）
- 狀態空間控制理論（可控性、可觀性、極點配置）
- 線性二次調節器（LQR）概念

## 參考資源

### 書籍

**PID 與控制系統：**
- "Feedback Control of Dynamic Systems" - Franklin, Powell, Emami-Naeini（控制工程經典教材）
- "Modern Control Engineering" - Katsuhiko Ogata（PID 到狀態空間，涵蓋範圍廣）
- "Control Systems Engineering" - Norman Nise（工程導向，範例豐富，適合自學）

**Kalman Filter 與狀態估測：**
- "Optimal State Estimation" - Dan Simon（Kalman Filter 最完整的參考書）
- "Probabilistic Robotics" - Thrun, Burgard, Fox（機器人領域的 KF/EKF 應用）

**進階控制理論（Level 5）：**
- "Applied Nonlinear Control" - Slotine, Li（滑模控制、自適應控制）
- "Optimal Control Theory: An Introduction" - Donald Kirk（LQR 深入推導）
- "Introduction to Autonomous Robots" - Nikolaus Correll et al.（機器人控制入門，免費線上版）

### 線上課程與影片

**基礎入門：**
- Roger Labbe - "Kalman and Bayesian Filters in Python" (GitHub) — 互動式 Jupyter Notebook 教學
- Brett Beauregard - Arduino PID Library — PID 實作的業界參考標準
- MATLAB Tech Talk 系列 — PID Controller / Kalman Filter 專題影片，動畫解說清晰

**控制理論系統課程：**
- MIT OCW 6.003 / 6.302 — 信號與系統 / 回饋系統設計，MIT 開放式課程
- Brian Douglas "Control Systems Lectures" (YouTube) — 強烈推薦，直觀動畫解說，從 Bode Plot 到狀態空間
- Steve Brunton "Control Bootcamp" (YouTube) — 含 MATLAB 程式碼的 LQR / Kalman 教學，特別適合 Level 5

**專題資源：**
- Karl Johan Astrom & Tore Hagglund "PID Control" — PID 調參的權威參考，有線上版本
- Joan Sola - "Quaternion kinematics for the error-state Kalman filter" — EKF 姿態估測的理論推導

### 工具

**開發與模擬：**
- Arduino IDE 內建 Serial Plotter — 即時觀察 PID / KF 輸出波形
- MATLAB/Simulink: PID Tuner, Kalman Filter Designer — 專業級調參與視覺化
- GNU Octave + control toolbox — 免費的 MATLAB 替代方案，支援 LQR/Kalman 設計

**Python 生態系：**
- matplotlib / plotly — 數據視覺化與互動圖表
- python-control library — Python 的控制系統工具箱（transfer function, state space, LQR）

**機器人模擬器：**
- Webots — 開源機器人模擬器，內建倒單擺範例
- Gazebo — ROS 生態系的標準模擬器

### 開源專案參考

- **Arduino PID Library** (Brett Beauregard) — PID 實作的經典參考，本專案 `pid.c` 的設計靈感來源
- **GitHub 搜尋關鍵字**：`inverted pendulum ESP32`、`cart pole LQR`、`kalman filter IMU`
- **MathWorks Inverted Pendulum Example** — MATLAB/Simulink 官方範例，完整建模到控制流程

## License

MIT
