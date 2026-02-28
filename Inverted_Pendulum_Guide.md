# 倒單擺控制系統完整指南

## Inverted Pendulum Control System — 從理論到實作

> **本文件為 [PID Controller & Kalman Filter Tutorial](README.md) 的 Level 5 進階應用。**
> 建議先完成 Level 1-4 的基礎學習後再閱讀本指南。
> 相關電路圖：[inverted_pendulum_circuit.drawio](inverted_pendulum_circuit.drawio)

---

## 目錄

1. [專案概述](#1-專案概述)
2. [物理建模與動力學推導](#2-物理建模與動力學推導)
3. [線性化與狀態空間表示](#3-線性化與狀態空間表示)
4. [控制理論體系](#4-控制理論體系)
5. [非理想因素與補償](#5-非理想因素與補償)
6. [硬體方案與可行性評估](#6-硬體方案與可行性評估)
7. [電路設計](#7-電路設計)
8. [建模驗證流程](#8-建模驗證流程)
9. [學習資料推薦](#9-學習資料推薦)
10. [附錄：典型參數與公式速查](#附錄典型參數與公式速查)

---

## 1. 專案概述

倒單擺（Inverted Pendulum）是控制工程中最經典的實驗平台之一，其核心挑戰在於將一根安裝在可移動滑車上的擺桿，穩定在鉛直向上的不穩定平衡點。這個系統天生不穩定（開迴路極點位於右半平面），必須依靠主動控制才能維持平衡，因此是驗證各種控制理論的理想載體。

本指南涵蓋從物理建模、控制器設計到硬體實作的完整流程，目標是以 ESP32 為主控，搭配 IMU 感測器與直流馬達，實現從擺起（Swing-Up）到穩定平衡的完整控制。

---

## 2. 物理建模與動力學推導

### 2.1 為什麼必須做物理建模？

控制器設計的本質是「根據模型計算最佳控制輸入」。以 LQR 為例，需要求解 Riccati 方程，而這個方程的輸入就是狀態空間矩陣 A 和 B — 這兩個矩陣完全來自物理動力學推導。如果模型不準，控制器的增益 K 就是錯的，系統可能根本穩不住。

即便是 PID 這種不直接依賴模型的方法，理解動力學也能幫助判斷系統的自然頻率、阻尼比，從而縮小調參範圍。

### 2.2 系統定義與座標設定

**系統組成：**

- 滑車（Cart）：質量 M，沿水平軌道運動
- 擺桿（Pendulum）：質量 m，長度 L（質心到轉軸距離 l = L/2）
- 外力 F 作用在滑車上（由馬達提供）

**廣義座標選擇：**

- x：滑車水平位移（向右為正）
- θ：擺桿與鉛直向上的夾角（直立為 θ=0，順時針為正）

這個座標定義很重要 — θ=0 就是我們要穩定的平衡點，後續線性化會圍繞這個點展開。

### 2.3 拉格朗日力學推導

#### 2.3.1 動能（Kinetic Energy）

**滑車動能：**

$$T_{cart} = \frac{1}{2} M \dot{x}^2$$

**擺桿質心位置：**

$$x_p = x + l \sin(\theta)$$

$$y_p = l \cos(\theta)$$

**擺桿質心速度（對時間微分）：**

$$\dot{x}_p = \dot{x} + l \dot{\theta} \cos(\theta)$$

$$\dot{y}_p = -l \dot{\theta} \sin(\theta)$$

**擺桿動能（平移 + 旋轉）：**

$$T_{pend} = \frac{1}{2} m (\dot{x}_p^2 + \dot{y}_p^2) + \frac{1}{2} I \dot{\theta}^2$$

展開 $\dot{x}_p^2 + \dot{y}_p^2$：

$$= \dot{x}^2 + 2l\dot{x}\dot{\theta}\cos(\theta) + l^2\dot{\theta}^2\cos^2(\theta) + l^2\dot{\theta}^2\sin^2(\theta)$$

$$= \dot{x}^2 + 2l\dot{x}\dot{\theta}\cos(\theta) + l^2\dot{\theta}^2$$

**系統總動能：**

$$T = \frac{1}{2}(M+m)\dot{x}^2 + ml\dot{x}\dot{\theta}\cos(\theta) + \frac{1}{2}(I+ml^2)\dot{\theta}^2$$

其中 I 為擺桿繞質心的轉動慣量。均勻細桿繞端點 $I = \frac{1}{3}ml^2$，繞質心 $I = \frac{1}{12}ml^2$。

#### 2.3.2 位能（Potential Energy）

$$V = mgl\cos(\theta)$$

注意：θ=0（直立）時位能最大，這正是不穩定平衡點的特徵。

#### 2.3.3 拉格朗日量

$$\mathcal{L} = T - V = \frac{1}{2}(M+m)\dot{x}^2 + ml\dot{x}\dot{\theta}\cos(\theta) + \frac{1}{2}(I+ml^2)\dot{\theta}^2 - mgl\cos(\theta)$$

#### 2.3.4 Euler-Lagrange 方程推導

**對廣義座標 x：**

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{x}} - \frac{\partial \mathcal{L}}{\partial x} = F$$

計算各項：

$$\frac{\partial \mathcal{L}}{\partial \dot{x}} = (M+m)\dot{x} + ml\dot{\theta}\cos(\theta)$$

$$\frac{d}{dt}[\cdots] = (M+m)\ddot{x} + ml\ddot{\theta}\cos(\theta) - ml\dot{\theta}^2\sin(\theta)$$

$$\frac{\partial \mathcal{L}}{\partial x} = 0$$

得到 **第一運動方程**：

$$\boxed{(M+m)\ddot{x} + ml\ddot{\theta}\cos(\theta) - ml\dot{\theta}^2\sin(\theta) = F}$$

**對廣義座標 θ：**

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{\theta}} - \frac{\partial \mathcal{L}}{\partial \theta} = 0$$

計算各項：

$$\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = ml\dot{x}\cos(\theta) + (I+ml^2)\dot{\theta}$$

$$\frac{d}{dt}[\cdots] = ml\ddot{x}\cos(\theta) - ml\dot{x}\dot{\theta}\sin(\theta) + (I+ml^2)\ddot{\theta}$$

$$\frac{\partial \mathcal{L}}{\partial \theta} = -ml\dot{x}\dot{\theta}\sin(\theta) + mgl\sin(\theta)$$

得到 **第二運動方程**：

$$\boxed{ml\ddot{x}\cos(\theta) + (I+ml^2)\ddot{\theta} - mgl\sin(\theta) = 0}$$

---

## 3. 線性化與狀態空間表示

### 3.1 小角度近似

在直立平衡點 θ ≈ 0 處進行泰勒展開：

$$\sin(\theta) \approx \theta, \quad \cos(\theta) \approx 1, \quad \dot{\theta}^2 \approx 0 \text{ (二階小量忽略)}$$

**線性化後的運動方程：**

$$(M+m)\ddot{x} + ml\ddot{\theta} = F \qquad \cdots(1)$$

$$ml\ddot{x} + (I+ml^2)\ddot{\theta} - mgl\theta = 0 \qquad \cdots(2)$$

### 3.2 求解聯立方程

將方程 (1) 和 (2) 寫成矩陣形式並解耦。令行列式：

$$D = (M+m)(I+ml^2) - (ml)^2$$

需確認 D ≠ 0（系統可解），展開後 $D = MI + Mml^2 + m I$，因為各物理量皆為正，所以 D > 0 恆成立。

解出加速度：

$$\ddot{x} = \frac{(I+ml^2)F + m^2l^2g\theta}{D}$$

$$\ddot{\theta} = \frac{mlF + (M+m)mgl\theta}{D}$$

### 3.3 狀態空間表示

定義狀態向量 $\mathbf{x} = [x, \dot{x}, \theta, \dot{\theta}]^T$，寫成標準形式 $\dot{\mathbf{x}} = A\mathbf{x} + Bu$：

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & \frac{m^2l^2g}{D} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & \frac{(M+m)mgl}{D} & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ \frac{I+ml^2}{D} \\ 0 \\ \frac{ml}{D} \end{bmatrix}$$

加入滑車摩擦係數 b 後，A 矩陣的 [2,2] 和 [4,2] 位置分別加入：

$$A_{22} = \frac{-b(I+ml^2)}{D}, \quad A_{42} = \frac{-bml}{D}$$

### 3.4 可控性與可觀性驗證

**可控性矩陣：**

$$\mathcal{C} = [B \quad AB \quad A^2B \quad A^3B]$$

若 $\text{rank}(\mathcal{C}) = 4$（滿秩），則系統完全可控，可以透過狀態回授任意配置極點。

**可觀性矩陣：**

$$\mathcal{O} = \begin{bmatrix} C \\ CA \\ CA^2 \\ CA^3 \end{bmatrix}$$

若 $\text{rank}(\mathcal{O}) = 4$，則系統完全可觀，可以設計觀測器估計所有狀態。

---

## 4. 控制理論體系

### 4.1 第一階段：PID 控制（入門）

> 💡 PID 基礎理論與調參方法請參考 [README — PID 調參實戰指南](README.md#pid-調參實戰指南)，對應 Level 1 的學習內容。

最直觀的方法，用角度誤差做 P、I、D 三項回授。適合小角度擾動的穩定，但對大擺幅效果有限。

**學習重點：**

- 調參方法：Ziegler-Nichols 經驗法則
- 抗積分飽和（Anti-Windup）
- 微分項的低通濾波處理

**局限性：** PID 本質上是單輸入單輸出（SISO）控制器，而倒單擺有 4 個狀態變數（位置、速度、角度、角速度），PID 無法同時兼顧滑車位置與擺桿角度的控制目標。

### 4.2 第二階段：極點配置（Pole Placement）

透過 Ackermann 公式計算全狀態回授增益 K，將閉迴路極點放到期望位置。

**設計流程：**

1. 根據性能需求（超越量、安定時間）選定期望極點位置
2. 驗證系統可控性
3. 用 Ackermann 公式或 MATLAB `place()` 函數計算 K
4. 閉迴路系統：$\dot{\mathbf{x}} = (A - BK)\mathbf{x}$

**前提：** 系統必須完全可控，且需要全狀態量測或設計觀測器。

### 4.3 第三階段：LQR — 線性二次調節器（核心方法）

倒單擺控制最常用且最成熟的方法。

**代價函數：**

$$J = \int_0^{\infty} (\mathbf{x}^T Q \mathbf{x} + u^T R u) \, dt$$

- Q 矩陣（4×4）：權衡各狀態的重要性，對角元素越大代表對應狀態受到越嚴格的約束
- R 矩陣（1×1）：限制控制能量，R 越大則控制力越保守

**設計步驟：**

1. 選擇 Q 和 R 的初始值（例如 Q = diag([1, 0, 10, 0])，強調角度比位置重要）
2. 求解連續代數 Riccati 方程（CARE）：$A^TP + PA - PBR^{-1}B^TP + Q = 0$
3. 得到最佳增益：$K = R^{-1}B^TP$
4. 閉迴路控制律：$u = -K\mathbf{x}$

**優勢：** 系統化的調參方法、保證穩定性（只要模型準確）、最佳能量效率。

### 4.4 第四階段：卡爾曼濾波器（Kalman Filter）

> 💡 Kalman Filter 基礎理論與 EKF 概念請參考 [README — Kalman Filter 調參指南](README.md#kalman-filter-調參指南) 及 [EKF 簡介](README.md#extended-kalman-filter-ekf-簡介)，對應 Level 2 & 4 的學習內容。

結合 IMU 加速度計與陀螺儀的感測器融合，處理量測雜訊與系統雜訊。

**核心概念：**

- **預測步驟**：根據系統模型預測下一時刻的狀態
- **更新步驟**：根據實際量測值修正預測

**參數設定：**

- Q_kf（過程雜訊協方差）：反映模型不確定性，越大代表越不信任模型
- R_kf（量測雜訊協方差）：反映感測器雜訊水平，可實測得到

**與 LQR 結合 → LQG（Linear Quadratic Gaussian）控制器：** 這是分離定理（Separation Principle）的經典應用 — Kalman Filter 負責狀態估計，LQR 負責最佳控制，兩者可以獨立設計。

### 4.5 第五階段：擺起控制（Swing-Up Control）

當擺桿不在直立附近時，LQR 的線性化假設不成立，需要用能量法將擺桿從靜止擺到接近直立。

**能量法（Energy-based Control）：**

擺桿的能量：$E = \frac{1}{2}I\dot{\theta}^2 + mgl(\cos\theta - 1)$

目標能量（直立靜止）：$E_{ref} = 0$

控制律：$u = k_E(E - E_{ref}) \cdot \text{sign}(\dot{\theta}\cos\theta)$

**切換控制策略：**

- 當 |θ| ≥ 30°（可調閾值）→ 使用 Swing-Up 能量控制
- 當 |θ| < 30° → 切換到 LQR 穩定控制
- 切換點的選擇需要在線性化有效範圍與能量控制收斂速度之間取得平衡

### 4.6 進階延伸方法

| 方法 | 核心思想 | 適用場景 |
|------|---------|---------|
| 滑模控制（SMC） | 設計滑動面，強迫系統狀態在滑動面上運動 | 參數不確定性大、外部干擾頻繁 |
| MPC 模型預測控制 | 在每個時刻求解有限時域最佳化問題 | 需要處理約束（軌道長度限制、馬達電流限制） |
| 模糊控制（Fuzzy） | 用語言變數和模糊規則替代數學模型 | 不需精確模型的替代方案 |
| H∞ 控制 | 最小化最壞情況下的性能指標 | 強健性能要求高 |
| 強化學習（RL） | 透過與環境互動自動學習控制策略 | 探索性研究、模型難以建立的情況 |

---

## 5. 非理想因素與補償

### 5.1 庫倫摩擦（Coulomb Friction）

滑車與軌道之間的靜摩擦/動摩擦會造成 dead-zone 效應：低速時馬達施力不足以克服靜摩擦，導致控制出現 limit cycle。

**處理方法：**

- 加入摩擦補償項（feedforward compensation）
- 使用 dither signal 克服 stiction
- 在模型中建立摩擦模型（如 Stribeck friction model）

### 5.2 馬達動態

DC 馬達本身有電氣時間常數（L/R），高頻控制時不能忽略。完整模型需加入馬達電壓方程：

$$V = K_e \omega + Ri + L\frac{di}{dt}$$

其中馬達力矩 $\tau = K_t \cdot i$。

若馬達電氣時間常數遠小於機械時間常數（通常如此），可以簡化為一階近似。

### 5.3 齒隙（Backlash）

若使用齒輪或皮帶傳動，齒隙會引入非線性死區，對高精度控制影響很大。

**建議：** 使用直驅方案或高預壓同步帶（GT2）減少齒隙。

### 5.4 感測器量化與雜訊

- 編碼器解析度決定角度量化誤差：600P/R → 0.6°/pulse
- IMU 加速度計有 bias drift，陀螺儀有 random walk
- Kalman Filter 是處理這些問題的標準方法

---

## 6. 硬體方案與可行性評估

### 6.1 推薦硬體清單

| 元件 | 推薦型號 | 用途 | 預估價格 (USD) |
|------|---------|------|---------------|
| 微控制器 | ESP32-WROOM-32 | 主控，雙核 240MHz | ~$5 |
| IMU 感測器 | MPU6050 (GY-521) | 量測擺桿角度與角速度 | ~$3 |
| 直流馬達 | JGA25-371（帶編碼器） | 驅動滑車，編碼器回授位置 | ~$15 |
| 馬達驅動 | TB6612FNG 或 BTS7960 | PWM 馬達驅動 | ~$5-10 |
| 旋轉編碼器 | 600P/R 增量式 | 量測擺桿角度（若不用 IMU） | ~$8 |
| 線性滑軌 | MGN12H 400mm | 滑車運動軌道 | ~$15 |
| 同步帶 | GT2 齒形帶 + 皮帶輪 | 馬達驅動滑車的傳動 | ~$8 |
| 電源 | 12V 3A DC | 馬達供電 | ~$8 |
| 穩壓模組 | AMS1117-3.3V | 為 ESP32 提供穩定 3.3V | ~$1 |

**總預估成本：約 $70–100 USD**

### 6.2 ESP32 方案可行性評估

**優勢：**

- 控制迴路頻率可達 1kHz（倒單擺需至少 200Hz，建議 500Hz 以上）
- 雙核架構：Core 0 處理通訊（WiFi 即時數據視覺化），Core 1 專用於控制迴路
- 內建 WiFi 可做即時數據視覺化與參數調整
- 足夠的 GPIO、I2C、SPI、PWM 通道
- 支援 FreeRTOS 做即時排程
- Arduino/ESP-IDF 生態系成熟，開源範例豐富

**風險與注意事項：**

- ESP32 的 ADC 線性度不佳（非單調），建議 IMU 走 I2C 而非 ADC
- MPU6050 的 DMP（Digital Motion Processor）可做硬體角度融合，減輕 CPU 負擔
- 機械結構的剛性和摩擦力是成敗關鍵，建議用鋁型材
- 控制迴路必須固定週期執行，避免被 WiFi 中斷或其他任務搶佔
- WiFi 協議棧在 Core 0 運行時偶爾會佔用較多資源，需用 `xTaskCreatePinnedToCore()` 將控制任務鎖定在 Core 1

---

## 7. 電路設計

### 7.1 系統接線總覽

```
                    ┌─────────────┐
   12V DC ─────────►│ AMS1117-3.3V├──────► 3.3V (ESP32, Sensors)
   Adapter    │     └─────────────┘
              │          │
         C1 100μF    C2 10μF
         (濾波)      (去耦)
              │
              ▼
   ┌─────────────────┐      PWM/DIR        ┌──────────────┐
   │                 │   GPIO 25,26,27      │  TB6612FNG   │
   │                 ├─────────────────────►│  Motor Driver │
   │                 │                      │  VM = 12V    │
   │                 │                      └──────┬───────┘
   │    ESP32        │                             │ AO1, AO2
   │   WROOM-32     │                      ┌──────▼───────┐
   │                 │  Encoder A/B         │  JGA25-371   │
   │                 │◄─── GPIO 32,33 ──────│  DC Motor    │
   │                 │                      │  + Encoder   │
   │                 │                      └──────────────┘
   │                 │
   │                 │  I2C (SDA/SCL)       ┌──────────────┐
   │                 │◄─── GPIO 21,22 ──────│   MPU6050    │
   │                 │  (4.7kΩ pull-up)     │  (GY-521)    │
   │                 │                      └──────────────┘
   │                 │
   │                 │  Pend Enc A/B        ┌──────────────┐
   │                 │◄─── GPIO 34,35 ──────│  600P/R      │
   │                 │                      │  Rotary Enc  │
   │                 │                      └──────────────┘
   │                 │
   │                 │  Limit Switches      ┌──────────────┐
   │                 │◄─── GPIO 16,17 ──────│  微動開關 x2  │
   │                 │  (內部上拉)           │  (軌道兩端)   │
   │                 │                      └──────────────┘
   │                 │
   │            GPIO 2├──── Status LED + 330Ω → GND
   └─────────────────┘
```

### 7.2 ESP32 GPIO 腳位分配

| GPIO | 功能 | 連接目標 | 備註 |
|------|------|---------|------|
| 21 | SDA (I2C) | MPU6050 SDA | 需 4.7kΩ 上拉至 3.3V |
| 22 | SCL (I2C) | MPU6050 SCL | 需 4.7kΩ 上拉至 3.3V |
| 25 | PWM_A | TB6612FNG PWMA | LEDC PWM, 20kHz |
| 26 | IN1 | TB6612FNG AIN1 | 馬達方向控制 |
| 27 | IN2 | TB6612FNG AIN2 | 馬達方向控制 |
| 32 | Encoder A | 馬達編碼器 A 相 | 中斷觸發 |
| 33 | Encoder B | 馬達編碼器 B 相 | 中斷觸發 |
| 34 | Pend Enc A | 擺桿編碼器 A 相 | 僅輸入（無內部上拉） |
| 35 | Pend Enc B | 擺桿編碼器 B 相 | 僅輸入（無內部上拉） |
| 16 | Limit SW Left | 左限位開關 | 內部上拉，常開接 GND |
| 17 | Limit SW Right | 右限位開關 | 內部上拉，常開接 GND |
| 2 | Status LED | LED + 330Ω 電阻 | 狀態指示 |

### 7.3 電源設計要點

1. **馬達電源與邏輯電源分離**：12V 直接供給馬達驅動板的 VM 腳位，經 AMS1117 穩壓後提供 3.3V 給 ESP32 與感測器，避免馬達啟停時的電壓波動影響邏輯電路
2. **12V 輸入端加 100μF 電解電容**（C1）：濾除電源紋波與馬達反電動勢雜訊
3. **3.3V 輸出端加 10μF 去耦電容**（C2）：穩定穩壓器輸出
4. **各 IC 的 VCC 腳位加 0.1μF 陶瓷電容**：就近去耦高頻雜訊
5. **GND 採用星形接地**：避免大電流回路與信號回路共用路徑產生地迴路雜訊

### 7.4 I2C 匯流排注意事項

- MPU6050 的 AD0 腳位接 GND，I2C 地址為 0x68
- SDA 和 SCL 各需一個 4.7kΩ 上拉電阻至 3.3V
- I2C 時脈建議設為 400kHz（Fast Mode），以確保感測器讀取速度
- 若使用 MPU6050 的 DMP 功能，INT 腳位可接至 ESP32 的 GPIO 4（可選）

### 7.5 編碼器信號處理

- 馬達編碼器和擺桿編碼器的 A/B 相信號建議各加 0.1μF 濾波電容
- GPIO 34、35 為僅輸入腳位，無內部上拉功能，需外接 10kΩ 上拉電阻
- 使用 ESP32 的硬體中斷（`attachInterrupt`）處理編碼器脈衝，避免丟失計數

### 7.6 DrawIO 電路圖

完整的 DrawIO 格式電路圖（`.drawio` 檔案）包含：

- **上半部分：硬體電路接線圖** — 電源模組、ESP32 完整 GPIO 分配、MPU6050 I2C 連接、TB6612FNG 馬達驅動、DC 馬達帶編碼器、擺桿旋轉編碼器、安全限位開關
- **下半部分：控制迴路架構圖** — Kalman Filter → LQR 控制器 → Swing-Up 切換邏輯 → 受控體 → 感測器回授的完整閉迴路流程

請用 [draw.io（diagrams.net）](https://app.diagrams.net/) 開啟隨附的 [`inverted_pendulum_circuit.drawio`](inverted_pendulum_circuit.drawio) 查看完整電路圖。

---

## 8. 建模驗證流程

### 8.1 建議的實作路徑

**Step 1：MATLAB/Simulink 模擬**

用本文推導的方程建立非線性模型（ode45 數值積分），驗證開迴路行為是否符合物理直覺。例如：擺桿從 5° 初始偏移自由落下的軌跡、施加脈衝力後滑車的加速度響應。

**Step 2：線性模型比對**

在小角度範圍（例如 ±15°）比較非線性模型與線性化模型的差異，確認線性化的有效範圍。這個範圍將決定 Swing-Up 到 LQR 的切換閾值。

**Step 3：控制器設計與模擬驗證**

基於線性化模型設計 LQR 控制器，在 Simulink 中先以非線性模型驗證閉迴路穩定性。調整 Q、R 矩陣直到性能滿意。

**Step 4：硬體組裝與參數辨識**

實際硬體組裝後，用 step response 或 frequency sweep 辨識真實的 M、m、l、b 等參數。這一步非常關鍵，因為理論值與實際值通常有 10%–30% 的偏差。

**Step 5：控制器部署與調校**

將控制器移植到 ESP32，先做小角度穩定測試，逐步擴大擾動範圍。利用 WiFi 即時傳回狀態數據，觀察控制效果。

**Step 6：Robust 驗證**

對模型參數加入 ±20% 不確定性，驗證控制器的穩定裕度（Gain Margin、Phase Margin）是否足夠。

### 8.2 常見問題排查

| 現象 | 可能原因 | 排查方法 |
|------|---------|---------|
| 擺桿持續振盪不收斂 | LQR 增益不足或模型參數偏差 | 增大 Q 矩陣中角度權重，重新辨識參數 |
| 滑車持續漂移到軌道端點 | 位置回授增益太小 | 增大 Q 矩陣中位置權重 |
| 低速時出現 limit cycle | 庫倫摩擦 dead-zone | 加入摩擦補償或 dither signal |
| 高頻噪音導致馬達抖動 | 微分項放大感測器雜訊 | 加強 Kalman Filter 調校，降低量測雜訊協方差 |
| 控制迴路週期不穩定 | WiFi 任務搶佔 CPU | 將控制任務鎖定在 Core 1，提高任務優先級 |

---

## 9. 學習資料推薦

### 9.1 教科書

- **Modern Control Engineering** — Katsuhiko Ogata（經典入門，涵蓋 PID 到狀態空間）
- **Feedback Control of Dynamic Systems** — Franklin, Powell, Emami-Naeini（工程導向，範例豐富）
- **Optimal Control Theory** — Donald Kirk（LQR 深入推導）
- **Applied Nonlinear Control** — Slotine, Li（滑模控制、自適應控制）

### 9.2 線上課程與影片

- **MIT OCW 6.003 / 6.302** — 控制系統基礎與進階
- **Brian Douglas YouTube "Control Systems Lectures"** — 強烈推薦，直觀易懂，動畫解說出色
- **Steve Brunton YouTube "Control Bootcamp"** — 含 MATLAB 程式碼的 LQR / Kalman 教學，特別適合本專案
- **MATLAB Tech Talk 系列** — 有 Inverted Pendulum 專題影片

### 9.3 實作參考

- **MATLAB/Simulink Inverted Pendulum Example** — MathWorks 官方範例，完整的建模到控制流程
- **GitHub 搜尋關鍵字**：`inverted pendulum ESP32`、`inverted pendulum arduino`、`cart pole LQR`
- **Quanser QUBE-Servo** — 商用倒單擺教學平台的技術文件，設計思路值得參考

---

## 附錄：典型參數與公式速查

### A.1 典型系統參數

| 參數 | 符號 | 典型值 | 單位 |
|------|------|--------|------|
| 滑車質量 | M | 0.5 | kg |
| 擺桿質量 | m | 0.2 | kg |
| 擺桿半長（質心到轉軸） | l | 0.3 | m |
| 擺桿繞質心轉動慣量 | I | 0.006 | kg·m² |
| 重力加速度 | g | 9.81 | m/s² |
| 滑車摩擦係數 | b | 0.1 | N·s/m |
| 行列式 | D | MI + Mml² + mI | kg²·m² |

### A.2 核心公式速查

| 公式 | 用途 |
|------|------|
| $\mathcal{L} = T - V$ | 拉格朗日量 |
| $\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i$ | Euler-Lagrange 方程 |
| $\dot{\mathbf{x}} = A\mathbf{x} + Bu$ | 狀態空間表示 |
| $J = \int_0^{\infty} (\mathbf{x}^TQ\mathbf{x} + u^TRu) \, dt$ | LQR 代價函數 |
| $A^TP + PA - PBR^{-1}B^TP + Q = 0$ | 連續代數 Riccati 方程 |
| $K = R^{-1}B^TP$ | LQR 最佳增益 |
| $u = -K\hat{\mathbf{x}}$ | 狀態回授控制律 |
| $E = \frac{1}{2}I\dot{\theta}^2 + mgl(\cos\theta - 1)$ | 擺桿能量（Swing-Up 用） |

---

*文件版本：v1.0 | 日期：2026-02-28*
