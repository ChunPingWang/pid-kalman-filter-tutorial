# PID Controller & Kalman Filter 教學文件與 PoC 計劃

> **版本**: 1.0  
> **目標平台**: Arduino, Raspberry Pi, STM32, BeagleBone  
> **驗證工具**: Claude Code  
> **日期**: 2026-02-28

---

## 目錄

1. [概述與學習路徑](#1-概述與學習路徑)
2. [PID Controller 理論基礎](#2-pid-controller-理論基礎)
3. [Kalman Filter 理論基礎](#3-kalman-filter-理論基礎)
4. [PID + Kalman Filter 整合架構](#4-pid--kalman-filter-整合架構)
5. [跨平台抽象層設計](#5-跨平台抽象層設計)
6. [PoC 1: 純 PID 溫度控制](#6-poc-1-純-pid-溫度控制)
7. [PoC 2: 一維 Kalman Filter 感測器濾波](#7-poc-2-一維-kalman-filter-感測器濾波)
8. [PoC 3: Kalman + PID 整合馬達控制](#8-poc-3-kalman--pid-整合馬達控制)
9. [PoC 4: 擴展 Kalman Filter (EKF) 姿態估測](#9-poc-4-擴展-kalman-filter-ekf-姿態估測)
10. [各平台部署指南](#10-各平台部署指南)
11. [Claude Code 驗證腳本](#11-claude-code-驗證腳本)
12. [效能基準測試](#12-效能基準測試)
13. [故障排除與常見問題](#13-故障排除與常見問題)
14. [參考資源](#14-參考資源)

---

## 1. 概述與學習路徑

### 1.1 為什麼需要 PID + Kalman Filter？

在嵌入式控制系統中，感測器讀數永遠帶有雜訊。直接將雜訊訊號送入 PID 控制器會導致控制輸出劇烈震盪、執行器壽命縮短、系統不穩定。Kalman Filter 作為最佳線性無偏估計器，能從雜訊中還原真實狀態，為 PID 提供乾淨的回饋訊號。

```
感測器(含雜訊) → Kalman Filter(狀態估測) → PID Controller(控制計算) → 執行器(輸出)
     ↑                                                                      |
     └──────────────────────── 回饋迴路 ────────────────────────────────────┘
```

### 1.2 學習路徑

```
Level 1: PID 基礎          → PoC 1 (溫度控制)
Level 2: Kalman Filter 基礎 → PoC 2 (感測器濾波)
Level 3: PID + KF 整合      → PoC 3 (馬達控制)
Level 4: EKF 進階           → PoC 4 (姿態估測)
Level 5: 進階應用 — 倒單擺  → 完整指南 (物理建模 → LQR → 實作)
```

### 1.3 前置知識

- C/C++ 基礎程式設計
- 基本線性代數 (矩陣運算)
- 基本機率統計 (常態分佈、變異數)
- 嵌入式系統基礎 (GPIO, ADC, PWM, I2C/SPI)

---

## 2. PID Controller 理論基礎

### 2.1 PID 公式

PID 控制器根據誤差 (error) 的三個面向來計算控制輸出：

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```

- **P (Proportional)**: 與當前誤差成正比 → 決定回應速度
- **I (Integral)**: 與累積誤差成正比 → 消除穩態誤差 (steady-state error)
- **D (Derivative)**: 與誤差變化率成正比 → 抑制超調 (overshoot)，增加阻尼

### 2.2 離散化 PID

嵌入式系統以固定取樣週期 `dt` 運行，需要離散化：

```
e[k] = setpoint - measurement[k]

P_term = Kp * e[k]
I_term = Ki * Σ(e[i] * dt)        // 累加近似積分
D_term = Kd * (e[k] - e[k-1]) / dt  // 差分近似微分
u[k]   = P_term + I_term + D_term
```

### 2.3 Anti-Windup (積分飽和防護)

當執行器飽和 (例如 PWM 到達 100%) 時，積分項會持續累加造成 windup，解除飽和後會有嚴重超調：

```
解決策略:
1. Clamping: 限制積分項範圍 → I_term = clamp(I_term, I_min, I_max)
2. Back-calculation: 飽和時反向修正積分 → I_term -= Kb * (u_saturated - u_raw)
3. 條件積分: 誤差大於閾值時停止累加
```

### 2.4 微分項改進：Derivative on Measurement

傳統 `D_term = Kd * (e[k] - e[k-1]) / dt` 在 setpoint 突變時會產生 derivative kick。改為對測量值微分：

```
D_term = -Kd * (measurement[k] - measurement[k-1]) / dt
```

### 2.5 PID 調參方法

| 方法 | 適用場景 | 步驟 |
|------|---------|------|
| Ziegler-Nichols | 能容忍振盪的系統 | 1. 僅用 P 控制，增加 Kp 直到持續振盪 2. 記錄臨界增益 Ku 和振盪週期 Tu 3. 查表計算 Kp/Ki/Kd |
| Cohen-Coon | 有明顯延遲的系統 | 基於開環階躍響應的延遲/時間常數比 |
| 手動調參 | 所有場景 | P 先調到微微振盪 → 加 D 消除振盪 → 加 I 消除穩態誤差 |
| 自動調參 | 產品化系統 | 繼電器回饋法 (Relay feedback) 自動找 Ku 和 Tu |

**Ziegler-Nichols 查表:**

| 控制器 | Kp | Ki | Kd |
|--------|----|----|-----|
| P      | 0.5 * Ku | - | - |
| PI     | 0.45 * Ku | 1.2 * Kp / Tu | - |
| PID    | 0.6 * Ku | 2 * Kp / Tu | Kp * Tu / 8 |

---

## 3. Kalman Filter 理論基礎

### 3.1 核心概念

Kalman Filter 是一種遞迴貝氏估計器，在以下兩個模型假設下為最佳線性無偏估計：

**系統模型 (State Transition):**

```
x[k] = F * x[k-1] + B * u[k-1] + w[k-1]

x: 狀態向量
F: 狀態轉移矩陣
B: 控制輸入矩陣
u: 控制輸入
w: 過程雜訊 ~ N(0, Q)
```

**觀測模型 (Measurement):**

```
z[k] = H * x[k] + v[k]

z: 觀測向量
H: 觀測矩陣
v: 觀測雜訊 ~ N(0, R)
```

### 3.2 Kalman Filter 五步驟

```
── 預測 (Predict) ──────────────────────────
Step 1: 狀態預測     x̂[k|k-1] = F * x̂[k-1|k-1] + B * u[k-1]
Step 2: 協方差預測   P[k|k-1]  = F * P[k-1|k-1] * F^T + Q

── 更新 (Update) ──────────────────────────
Step 3: 卡爾曼增益   K[k] = P[k|k-1] * H^T * (H * P[k|k-1] * H^T + R)^(-1)
Step 4: 狀態更新     x̂[k|k] = x̂[k|k-1] + K[k] * (z[k] - H * x̂[k|k-1])
Step 5: 協方差更新   P[k|k]  = (I - K[k] * H) * P[k|k-1]
```

### 3.3 直覺理解

- **K 接近 1**: 信任觀測 → R 小 (感測器精確) 或 P 大 (狀態不確定)
- **K 接近 0**: 信任預測 → R 大 (感測器雜訊大) 或 P 小 (狀態確定)
- Kalman Filter 本質上是「預測」和「觀測」的加權融合，權重由各自的不確定性決定

### 3.4 一維簡化

對於單一變數 (例如溫度)，矩陣退化為純量：

```
// Predict
x_pred = x_est                    // 假設狀態不變 (F=1)
p_pred = p_est + q                // q: 過程雜訊變異數

// Update
k = p_pred / (p_pred + r)         // r: 觀測雜訊變異數
x_est = x_pred + k * (z - x_pred)
p_est = (1 - k) * p_pred
```

### 3.5 Q 和 R 的調參

| 參數 | 增大效果 | 減小效果 |
|------|---------|---------|
| Q (過程雜訊) | 更信任觀測，追蹤快但雜訊多 | 更信任模型，平滑但追蹤慢 |
| R (觀測雜訊) | 更信任模型，平滑但追蹤慢 | 更信任觀測，追蹤快但雜訊多 |

**實務調參策略:**
1. R 可由感測器 datasheet 或靜態量測統計取得
2. Q 通常由實驗調整，從小值開始逐步增大
3. Q/R 比值決定濾波器行為：比值大 → 追蹤快；比值小 → 平滑好

### 3.6 Extended Kalman Filter (EKF) 概要

當系統或觀測模型為非線性時，EKF 用一階泰勒展開 (Jacobian) 進行線性化：

```
x[k] = f(x[k-1], u[k-1]) + w      // 非線性狀態轉移
z[k] = h(x[k]) + v                  // 非線性觀測

F[k] = ∂f/∂x |_{x=x̂[k-1]}          // 狀態轉移 Jacobian
H[k] = ∂h/∂x |_{x=x̂[k|k-1]}       // 觀測 Jacobian
```

其餘步驟與標準 Kalman Filter 相同，但 F 和 H 每步都要重新計算。

---

## 4. PID + Kalman Filter 整合架構

### 4.1 架構圖

```
                    ┌─────────────────────────────────────┐
                    │         System Plant                 │
                    │   (馬達/加熱器/機器人)                │
                    └──────────┬──────────────────────────┘
                               │ 實際狀態 (含雜訊)
                               ▼
                    ┌──────────────────────┐
                    │    Sensor(s)         │
                    │  z = H*x + v         │
                    └──────────┬───────────┘
                               │ z[k] (原始觀測)
                               ▼
                    ┌──────────────────────┐
                    │   Kalman Filter      │
                    │  x̂ = predict+update  │──→ 估測狀態 x̂[k] (乾淨)
                    └──────────────────────┘         │
                                                     ▼
              setpoint ──→ ┌──────────────────────┐
                           │   PID Controller     │
                           │  e = sp - x̂          │
                           │  u = Kp*e + Ki*∫e    │
                           │      + Kd*de/dt      │
                           └──────────┬───────────┘
                                      │ u[k] (控制輸出)
                                      ▼
                           ┌──────────────────────┐
                           │   Actuator           │
                           │  (PWM/DAC)           │
                           └──────────┬───────────┘
                                      │
                                      └──→ 回到 System Plant
```

### 4.2 時序流程 (每個控制週期)

```
1. 讀取感測器原始值 z[k]
2. Kalman Predict: 用上一步控制輸出 u[k-1] 預測狀態
3. Kalman Update:  用 z[k] 修正預測，得到 x̂[k]
4. PID Compute:    e[k] = setpoint - x̂[k], 計算 u[k]
5. 輸出 u[k] 到執行器
6. 記錄 log (可選)
7. 等待下一個取樣週期
```

---

## 5. 跨平台抽象層設計

### 5.1 平台能力比較

| 特性 | Arduino UNO | Arduino Mega | Raspberry Pi 4 | STM32F4 | BeagleBone Black |
|------|------------|-------------|----------------|---------|-----------------|
| CPU | ATmega328P 16MHz | ATmega2560 16MHz | BCM2711 1.5GHz | Cortex-M4 168MHz | AM3358 1GHz |
| RAM | 2KB | 8KB | 4GB | 192KB | 512MB |
| FPU | 無 | 無 | 有 | 有 | 有 |
| ADC | 10-bit | 10-bit | 無 (需外接) | 12-bit | 12-bit |
| PWM | 6 ch | 15 ch | 軟體/硬體 | 多通道 | 多通道 |
| OS | 裸機 | 裸機 | Linux | 裸機/RTOS | Linux |
| 浮點建議 | 用 fixed-point | 用 fixed-point | float/double | float | float/double |

### 5.2 硬體抽象介面 (HAL)

```cpp
// ═══════════════════════════════════════════════
// hal.h - Hardware Abstraction Layer
// ═══════════════════════════════════════════════
#ifndef HAL_H
#define HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- 時間 ---
uint32_t hal_millis(void);          // 毫秒時間戳
void     hal_delay_ms(uint32_t ms); // 毫秒延遲

// --- ADC ---
void     hal_adc_init(uint8_t channel);
uint16_t hal_adc_read(uint8_t channel);  // 回傳原始值
float    hal_adc_to_voltage(uint16_t raw, float vref, uint8_t bits);

// --- PWM ---
void     hal_pwm_init(uint8_t channel, uint32_t freq_hz);
void     hal_pwm_set_duty(uint8_t channel, float duty_percent); // 0~100

// --- GPIO ---
void     hal_gpio_init(uint8_t pin, uint8_t mode); // 0=INPUT, 1=OUTPUT
void     hal_gpio_write(uint8_t pin, uint8_t value);
uint8_t  hal_gpio_read(uint8_t pin);

// --- Serial/UART ---
void     hal_serial_init(uint32_t baud);
void     hal_serial_print(const char* str);
void     hal_serial_printf(const char* fmt, ...);

// --- I2C (for IMU sensors) ---
void     hal_i2c_init(uint32_t speed);
int      hal_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);
int      hal_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);

#ifdef __cplusplus
}
#endif
#endif // HAL_H
```

### 5.3 平台實作映射

```
hal_arduino.cpp  → analogRead(), analogWrite(), Serial, Wire
hal_rpi.cpp      → pigpio / WiringPi / sysfs GPIO
hal_stm32.cpp    → HAL_ADC_Start(), HAL_TIM_PWM_Start(), HAL_UART
hal_bbb.cpp      → /sys/class/gpio, /sys/devices/.../pwm, sysfs ADC
```

---

## 6. PoC 1: 純 PID 溫度控制

### 6.1 目標

用 PID 控制加熱元件，維持目標溫度。理解 P/I/D 各項效果。

### 6.2 硬體需求

- 溫度感測器: NTC Thermistor 或 DS18B20 或 TMP36
- 加熱器: 電阻加熱線 + MOSFET 驅動
- PWM 輸出: 控制加熱功率

### 6.3 PID 核心實作

```cpp
// ═══════════════════════════════════════════════
// pid.h - PID Controller with Anti-Windup
// ═══════════════════════════════════════════════
#ifndef PID_H
#define PID_H

typedef struct {
    // 參數
    float kp;
    float ki;
    float kd;
    float dt;              // 取樣週期 (秒)

    // 輸出限制
    float out_min;
    float out_max;

    // 積分限制 (anti-windup)
    float integral_min;
    float integral_max;

    // 內部狀態
    float integral;
    float prev_measurement; // derivative on measurement
    float prev_error;
    int   initialized;
} PID_t;

/**
 * 初始化 PID 控制器
 */
void pid_init(PID_t* pid, float kp, float ki, float kd, float dt,
              float out_min, float out_max);

/**
 * 計算 PID 輸出
 * @param pid         PID 結構體指標
 * @param setpoint    目標值
 * @param measurement 實際測量值 (或 Kalman 估測值)
 * @return            控制輸出 (已限制在 out_min ~ out_max)
 */
float pid_compute(PID_t* pid, float setpoint, float measurement);

/**
 * 重置 PID 內部狀態
 */
void pid_reset(PID_t* pid);

#endif // PID_H
```

```cpp
// ═══════════════════════════════════════════════
// pid.c - PID Controller Implementation
// ═══════════════════════════════════════════════
#include "pid.h"

static float clamp(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

void pid_init(PID_t* pid, float kp, float ki, float kd, float dt,
              float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = out_min * 0.8f;  // 預設積分限制為輸出限制的 80%
    pid->integral_max = out_max * 0.8f;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error = 0.0f;
    pid->initialized = 0;
}

float pid_compute(PID_t* pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    // --- P ---
    float p_term = pid->kp * error;

    // --- I with anti-windup (clamping) ---
    pid->integral += error * pid->dt;
    pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);
    float i_term = pid->ki * pid->integral;

    // --- D on measurement (避免 derivative kick) ---
    float d_term = 0.0f;
    if (pid->initialized) {
        float d_measurement = (measurement - pid->prev_measurement) / pid->dt;
        d_term = -pid->kd * d_measurement;
    }
    pid->prev_measurement = measurement;
    pid->prev_error = error;
    pid->initialized = 1;

    // --- 合成輸出 ---
    float output = p_term + i_term + d_term;
    return clamp(output, pid->out_min, pid->out_max);
}

void pid_reset(PID_t* pid) {
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error = 0.0f;
    pid->initialized = 0;
}
```

### 6.4 Arduino 範例 (PoC 1)

```cpp
// ═══════════════════════════════════════════════
// poc1_pid_temperature.ino - Arduino PID Temperature Control
// ═══════════════════════════════════════════════
#include "pid.h"

// --- 硬體設定 ---
#define THERMISTOR_PIN   A0
#define HEATER_PWM_PIN   9
#define SAMPLE_PERIOD_MS 100

// --- Thermistor 參數 (NTC 10K, B=3950) ---
#define SERIES_RESISTOR  10000.0f
#define NOMINAL_RESISTANCE 10000.0f
#define NOMINAL_TEMP     25.0f
#define B_COEFFICIENT    3950.0f

// --- PID 參數 ---
PID_t heater_pid;
float setpoint = 50.0f;  // 目標溫度 °C

float read_temperature() {
    int raw = analogRead(THERMISTOR_PIN);
    float resistance = SERIES_RESISTOR / (1023.0f / raw - 1.0f);

    // Steinhart-Hart 簡化 B-parameter 方程式
    float steinhart = resistance / NOMINAL_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0f / (NOMINAL_TEMP + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;

    return steinhart;
}

void setup() {
    Serial.begin(115200);
    pinMode(HEATER_PWM_PIN, OUTPUT);

    float dt = SAMPLE_PERIOD_MS / 1000.0f;
    pid_init(&heater_pid, 10.0f, 0.5f, 2.0f, dt, 0.0f, 255.0f);

    Serial.println("time_ms,setpoint,temperature,pid_output");
}

void loop() {
    static uint32_t last_time = 0;
    uint32_t now = millis();

    if (now - last_time >= SAMPLE_PERIOD_MS) {
        last_time = now;

        float temperature = read_temperature();
        float output = pid_compute(&heater_pid, setpoint, temperature);

        analogWrite(HEATER_PWM_PIN, (int)output);

        // CSV 格式輸出 (方便繪圖分析)
        Serial.print(now);
        Serial.print(",");
        Serial.print(setpoint);
        Serial.print(",");
        Serial.print(temperature, 2);
        Serial.print(",");
        Serial.println(output, 2);
    }
}
```

### 6.5 驗證要點

```
□ P-only 控制: 設 Ki=0, Kd=0 → 觀察穩態誤差
□ PI 控制: 加入 Ki → 觀察穩態誤差消除，但可能超調
□ PID 控制: 加入 Kd → 觀察超調減少
□ Anti-windup 測試: 設極高 setpoint 再降回 → 觀察恢復時間
□ Derivative kick 測試: 突然改變 setpoint → 觀察輸出是否平滑
```

---

## 7. PoC 2: 一維 Kalman Filter 感測器濾波

### 7.1 目標

用 Kalman Filter 濾除感測器雜訊，比較原始值 vs 濾波後的值。

### 7.2 Kalman Filter 核心實作

```cpp
// ═══════════════════════════════════════════════
// kalman.h - Kalman Filter (Configurable Dimension)
// ═══════════════════════════════════════════════
#ifndef KALMAN_H
#define KALMAN_H

// --- 一維 Kalman Filter ---
typedef struct {
    float x_est;   // 估測狀態
    float p_est;   // 估測誤差協方差
    float q;       // 過程雜訊協方差
    float r;       // 觀測雜訊協方差
    float k;       // 卡爾曼增益 (可讀取用於診斷)
} KalmanFilter1D_t;

/**
 * 初始化一維 Kalman Filter
 * @param kf     Kalman Filter 結構體
 * @param q      過程雜訊變異數 (越大越信任觀測)
 * @param r      觀測雜訊變異數 (越大越信任模型)
 * @param x0     初始狀態估測
 * @param p0     初始誤差協方差
 */
void kf1d_init(KalmanFilter1D_t* kf, float q, float r, float x0, float p0);

/**
 * Kalman Filter 更新 (一維, F=1, H=1, B=0)
 * @param kf     Kalman Filter
 * @param z      觀測值
 * @return       濾波後的估測值
 */
float kf1d_update(KalmanFilter1D_t* kf, float z);

// --- 二維 Kalman Filter (位置+速度) ---
typedef struct {
    float x[2];       // 狀態 [position, velocity]
    float P[2][2];    // 協方差矩陣
    float Q[2][2];    // 過程雜訊
    float R;          // 觀測雜訊 (僅觀測位置)
    float dt;         // 取樣週期
} KalmanFilter2D_t;

void kf2d_init(KalmanFilter2D_t* kf, float q_pos, float q_vel,
               float r, float dt);
void kf2d_predict(KalmanFilter2D_t* kf);
float kf2d_update(KalmanFilter2D_t* kf, float z_position);

#endif // KALMAN_H
```

```cpp
// ═══════════════════════════════════════════════
// kalman.c - Kalman Filter Implementation
// ═══════════════════════════════════════════════
#include "kalman.h"

// ===== 一維 Kalman Filter =====

void kf1d_init(KalmanFilter1D_t* kf, float q, float r, float x0, float p0) {
    kf->q = q;
    kf->r = r;
    kf->x_est = x0;
    kf->p_est = p0;
    kf->k = 0.0f;
}

float kf1d_update(KalmanFilter1D_t* kf, float z) {
    // Predict (F=1, B=0: 假設狀態不變)
    float x_pred = kf->x_est;
    float p_pred = kf->p_est + kf->q;

    // Update
    kf->k = p_pred / (p_pred + kf->r);
    kf->x_est = x_pred + kf->k * (z - x_pred);
    kf->p_est = (1.0f - kf->k) * p_pred;

    return kf->x_est;
}

// ===== 二維 Kalman Filter (位置+速度) =====

void kf2d_init(KalmanFilter2D_t* kf, float q_pos, float q_vel,
               float r, float dt) {
    kf->x[0] = 0.0f;  // position
    kf->x[1] = 0.0f;  // velocity

    kf->P[0][0] = 1.0f;  kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;  kf->P[1][1] = 1.0f;

    kf->Q[0][0] = q_pos; kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;  kf->Q[1][1] = q_vel;

    kf->R = r;
    kf->dt = dt;
}

void kf2d_predict(KalmanFilter2D_t* kf) {
    float dt = kf->dt;

    // x_pred = F * x
    // F = [[1, dt], [0, 1]]
    float x0_new = kf->x[0] + dt * kf->x[1];
    float x1_new = kf->x[1];
    kf->x[0] = x0_new;
    kf->x[1] = x1_new;

    // P_pred = F * P * F^T + Q
    float p00 = kf->P[0][0] + dt * kf->P[1][0] + dt * (kf->P[0][1] + dt * kf->P[1][1]);
    float p01 = kf->P[0][1] + dt * kf->P[1][1];
    float p10 = kf->P[1][0] + dt * kf->P[1][1];
    float p11 = kf->P[1][1];

    kf->P[0][0] = p00 + kf->Q[0][0];
    kf->P[0][1] = p01 + kf->Q[0][1];
    kf->P[1][0] = p10 + kf->Q[1][0];
    kf->P[1][1] = p11 + kf->Q[1][1];
}

float kf2d_update(KalmanFilter2D_t* kf, float z_position) {
    // H = [1, 0] → 只觀測位置
    // S = H * P * H^T + R = P[0][0] + R
    float S = kf->P[0][0] + kf->R;

    // K = P * H^T * S^(-1) = [P[0][0]/S, P[1][0]/S]
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    // Innovation
    float y = z_position - kf->x[0];

    // x = x + K * y
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    // P = (I - K*H) * P
    float p00 = (1.0f - K0) * kf->P[0][0];
    float p01 = (1.0f - K0) * kf->P[0][1];
    float p10 = -K1 * kf->P[0][0] + kf->P[1][0];
    float p11 = -K1 * kf->P[0][1] + kf->P[1][1];

    kf->P[0][0] = p00; kf->P[0][1] = p01;
    kf->P[1][0] = p10; kf->P[1][1] = p11;

    return kf->x[0]; // 回傳估測位置
}
```

### 7.3 Arduino 範例 (PoC 2)

```cpp
// ═══════════════════════════════════════════════
// poc2_kalman_filter.ino - 1D Kalman Filter Demo
// ═══════════════════════════════════════════════
#include "kalman.h"
#include <math.h>

#define SENSOR_PIN       A0
#define SAMPLE_PERIOD_MS 50

KalmanFilter1D_t kf;

// 模擬產生帶雜訊的感測器值 (無實際硬體時用)
float simulate_noisy_sensor(float true_value, float noise_std) {
    // Box-Muller 轉換產生高斯雜訊
    float u1 = (float)random(1, 10000) / 10000.0f;
    float u2 = (float)random(1, 10000) / 10000.0f;
    float noise = sqrt(-2.0f * log(u1)) * cos(2.0f * 3.14159f * u2) * noise_std;
    return true_value + noise;
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(A1));  // 用未連接的 pin 作為隨機種子

    // 初始化 Kalman Filter
    // Q=0.01 (過程雜訊小，假設狀態變化慢)
    // R=1.0  (觀測雜訊 std ≈ 1.0)
    kf1d_init(&kf, 0.01f, 1.0f, 25.0f, 1.0f);

    Serial.println("time_ms,true_value,noisy_measurement,kalman_estimate,kalman_gain");
}

void loop() {
    static uint32_t last_time = 0;
    static float true_temp = 25.0f;
    static float phase = 0.0f;
    uint32_t now = millis();

    if (now - last_time >= SAMPLE_PERIOD_MS) {
        last_time = now;

        // 模擬真實溫度 (緩慢正弦波變化)
        phase += 0.02f;
        true_temp = 25.0f + 5.0f * sin(phase);

        // 加入雜訊
        float noisy_measurement = simulate_noisy_sensor(true_temp, 2.0f);

        // Kalman Filter
        float estimate = kf1d_update(&kf, noisy_measurement);

        // CSV 輸出
        Serial.print(now);
        Serial.print(",");
        Serial.print(true_temp, 3);
        Serial.print(",");
        Serial.print(noisy_measurement, 3);
        Serial.print(",");
        Serial.print(estimate, 3);
        Serial.print(",");
        Serial.println(kf.k, 4);
    }
}
```

### 7.4 驗證要點

```
□ Q/R 比值實驗: Q=0.001/R=1 vs Q=1/R=1 vs Q=1/R=0.01
□ 收斂速度: 從錯誤初始值開始，觀察幾步收斂
□ 追蹤能力: 真實值突然改變時的追蹤延遲
□ 卡爾曼增益 K: 觀察 K 如何收斂到穩態值
□ 與 Moving Average 比較: 相同窗口下的延遲與平滑效果差異
```

---

## 8. PoC 3: Kalman + PID 整合馬達控制

### 8.1 目標

使用 Encoder 量測馬達轉速，Kalman Filter 濾波後送入 PID 控制器，實現精確的轉速控制。

### 8.2 硬體需求

- DC 馬達 + H-Bridge 驅動 (L298N / TB6612)
- 光學/磁性旋轉編碼器
- PWM 輸出控制馬達

### 8.3 整合程式碼

```cpp
// ═══════════════════════════════════════════════
// poc3_kalman_pid_motor.ino - Kalman + PID Motor Speed Control
// ═══════════════════════════════════════════════
#include "pid.h"
#include "kalman.h"

// --- 硬體腳位 ---
#define MOTOR_PWM_PIN    6
#define MOTOR_DIR_PIN    7
#define ENCODER_A_PIN    2   // 需要中斷腳位
#define ENCODER_B_PIN    3

// --- 參數 ---
#define SAMPLE_PERIOD_MS 20
#define ENCODER_PPR      600   // Pulses Per Revolution
#define GEAR_RATIO       1.0f

// --- 全域變數 ---
volatile long encoder_count = 0;
long prev_encoder_count = 0;

PID_t speed_pid;
KalmanFilter2D_t speed_kf;     // 狀態: [speed, acceleration]

float target_rpm = 120.0f;

// --- Encoder 中斷處理 ---
void encoder_isr() {
    if (digitalRead(ENCODER_B_PIN) == HIGH) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

// --- 計算 RPM ---
float calculate_raw_rpm(float dt_sec) {
    long current_count = encoder_count;
    long delta = current_count - prev_encoder_count;
    prev_encoder_count = current_count;

    float rps = (float)delta / (ENCODER_PPR * GEAR_RATIO) / dt_sec;
    return rps * 60.0f;  // RPM
}

void setup() {
    Serial.begin(115200);

    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_isr, RISING);

    float dt = SAMPLE_PERIOD_MS / 1000.0f;

    // PID 初始化: 輸出為 PWM (0~255)
    pid_init(&speed_pid, 1.0f, 0.5f, 0.05f, dt, 0.0f, 255.0f);

    // 2D Kalman Filter: 估測 [speed, acceleration]
    // q_pos=0.1 (速度變化中等), q_vel=1.0 (加速度變化快)
    // R=10.0 (encoder 量測有些雜訊)
    kf2d_init(&speed_kf, 0.1f, 1.0f, 10.0f, dt);

    Serial.println("time_ms,target_rpm,raw_rpm,filtered_rpm,pid_output,kalman_gain");
}

void loop() {
    static uint32_t last_time = 0;
    uint32_t now = millis();

    if (now - last_time >= SAMPLE_PERIOD_MS) {
        float dt_actual = (now - last_time) / 1000.0f;
        last_time = now;

        // 1. 讀取原始 RPM
        float raw_rpm = calculate_raw_rpm(dt_actual);

        // 2. Kalman Filter: Predict + Update
        kf2d_predict(&speed_kf);
        float filtered_rpm = kf2d_update(&speed_kf, raw_rpm);

        // 3. PID: 用濾波後的 RPM 計算控制輸出
        float pid_output = pid_compute(&speed_pid, target_rpm, filtered_rpm);

        // 4. 驅動馬達
        digitalWrite(MOTOR_DIR_PIN, (pid_output >= 0) ? HIGH : LOW);
        analogWrite(MOTOR_PWM_PIN, (int)fabs(pid_output));

        // 5. 記錄
        Serial.print(now); Serial.print(",");
        Serial.print(target_rpm, 1); Serial.print(",");
        Serial.print(raw_rpm, 1); Serial.print(",");
        Serial.print(filtered_rpm, 1); Serial.print(",");
        Serial.println(pid_output, 1);
    }
}
```

### 8.4 比較實驗

```
實驗 A: PID + 原始 encoder 值 (不用 Kalman)
實驗 B: PID + Kalman 濾波後的值
實驗 C: PID + 移動平均濾波

比較指標:
- 穩態誤差 (Steady-State Error)
- 超調量 (Overshoot %)
- 安定時間 (Settling Time)
- 控制輸出平滑度 (PWM 變化標準差)
```

---

## 9. PoC 4: 擴展 Kalman Filter (EKF) 姿態估測

### 9.1 目標

用 MPU6050 (加速度計+陀螺儀) 實現姿態角 (Roll/Pitch) 估測。

### 9.2 EKF 姿態估測核心

```cpp
// ═══════════════════════════════════════════════
// ekf_attitude.h - Extended Kalman Filter for Attitude Estimation
// ═══════════════════════════════════════════════
#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

#include <math.h>

typedef struct {
    // 狀態: [roll, pitch, gyro_bias_x, gyro_bias_y]
    float x[4];

    // 協方差 4x4
    float P[4][4];

    // 過程雜訊
    float Q_angle;     // 角度過程雜訊
    float Q_bias;      // 陀螺儀偏移過程雜訊

    // 觀測雜訊
    float R_accel;     // 加速度計觀測雜訊

    float dt;
} EKF_Attitude_t;

void ekf_attitude_init(EKF_Attitude_t* ekf, float dt,
                        float q_angle, float q_bias, float r_accel);

/**
 * EKF 預測步驟 (使用陀螺儀)
 * @param gx, gy  陀螺儀角速度 (rad/s)
 */
void ekf_attitude_predict(EKF_Attitude_t* ekf, float gx, float gy);

/**
 * EKF 更新步驟 (使用加速度計)
 * @param ax, ay, az  加速度計值 (g)
 */
void ekf_attitude_update(EKF_Attitude_t* ekf, float ax, float ay, float az);

/** 取得估測角度 (度) */
float ekf_get_roll_deg(EKF_Attitude_t* ekf);
float ekf_get_pitch_deg(EKF_Attitude_t* ekf);

#endif
```

```cpp
// ═══════════════════════════════════════════════
// ekf_attitude.c - EKF Implementation
// ═══════════════════════════════════════════════
#include "ekf_attitude.h"

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f

void ekf_attitude_init(EKF_Attitude_t* ekf, float dt,
                        float q_angle, float q_bias, float r_accel) {
    ekf->dt = dt;
    ekf->Q_angle = q_angle;
    ekf->Q_bias = q_bias;
    ekf->R_accel = r_accel;

    // 初始狀態 = 0
    for (int i = 0; i < 4; i++) {
        ekf->x[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void ekf_attitude_predict(EKF_Attitude_t* ekf, float gx, float gy) {
    float dt = ekf->dt;

    // 去除陀螺儀偏移
    float gx_corrected = gx - ekf->x[2];
    float gy_corrected = gy - ekf->x[3];

    // 狀態預測: angle += gyro_corrected * dt
    ekf->x[0] += gx_corrected * dt;  // roll
    ekf->x[1] += gy_corrected * dt;  // pitch
    // bias 保持不變 (random walk model)

    // Jacobian F = I + [[0,0,-dt,0],[0,0,0,-dt],[0,0,0,0],[0,0,0,0]]
    // P = F*P*F^T + Q (簡化計算)
    ekf->P[0][0] += dt * (-ekf->P[2][0] - ekf->P[0][2] + dt * ekf->P[2][2]) + ekf->Q_angle;
    ekf->P[0][1] += dt * (-ekf->P[2][1] - ekf->P[0][3] + dt * ekf->P[2][3]);
    ekf->P[1][0] += dt * (-ekf->P[3][0] - ekf->P[1][2] + dt * ekf->P[3][2]);
    ekf->P[1][1] += dt * (-ekf->P[3][1] - ekf->P[1][3] + dt * ekf->P[3][3]) + ekf->Q_angle;

    ekf->P[2][2] += ekf->Q_bias;
    ekf->P[3][3] += ekf->Q_bias;
}

void ekf_attitude_update(EKF_Attitude_t* ekf, float ax, float ay, float az) {
    // 從加速度計計算觀測角度
    float accel_roll  = atan2(ay, az);
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Innovation (觀測 - 預測)
    float y0 = accel_roll  - ekf->x[0];
    float y1 = accel_pitch - ekf->x[1];

    // H = [[1,0,0,0],[0,1,0,0]] → S = H*P*H^T + R
    float S00 = ekf->P[0][0] + ekf->R_accel;
    float S11 = ekf->P[1][1] + ekf->R_accel;

    // Kalman Gain K = P * H^T * S^(-1) (因 H 簡單，直接計算)
    float K00 = ekf->P[0][0] / S00;
    float K10 = ekf->P[1][0] / S00;
    float K20 = ekf->P[2][0] / S00;
    float K30 = ekf->P[3][0] / S00;

    float K01 = ekf->P[0][1] / S11;
    float K11 = ekf->P[1][1] / S11;
    float K21 = ekf->P[2][1] / S11;
    float K31 = ekf->P[3][1] / S11;

    // 狀態更新: x = x + K * y
    ekf->x[0] += K00 * y0 + K01 * y1;
    ekf->x[1] += K10 * y0 + K11 * y1;
    ekf->x[2] += K20 * y0 + K21 * y1;
    ekf->x[3] += K30 * y0 + K31 * y1;

    // 協方差更新: P = (I - K*H) * P
    float P00_new = (1.0f - K00) * ekf->P[0][0] - K01 * ekf->P[1][0];
    float P11_new = -K10 * ekf->P[0][1] + (1.0f - K11) * ekf->P[1][1];

    ekf->P[0][0] = P00_new;
    ekf->P[1][1] = P11_new;
    ekf->P[2][0] -= K20 * ekf->P[0][0];
    ekf->P[3][1] -= K31 * ekf->P[1][1];
}

float ekf_get_roll_deg(EKF_Attitude_t* ekf) {
    return ekf->x[0] * RAD_TO_DEG;
}

float ekf_get_pitch_deg(EKF_Attitude_t* ekf) {
    return ekf->x[1] * RAD_TO_DEG;
}
```

### 9.3 MPU6050 整合範例

```cpp
// ═══════════════════════════════════════════════
// poc4_ekf_imu.ino - EKF Attitude Estimation with MPU6050
// ═══════════════════════════════════════════════
#include <Wire.h>
#include "ekf_attitude.h"

#define MPU6050_ADDR  0x68
#define SAMPLE_PERIOD_MS 10   // 100 Hz

EKF_Attitude_t ekf;

// MPU6050 原始數據
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

void mpu6050_init() {
    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);  // 喚醒
    Wire.endTransmission();

    // 設定加速度計 ±2g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();

    // 設定陀螺儀 ±250°/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void mpu6050_read() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

    ax_raw = Wire.read() << 8 | Wire.read();
    ay_raw = Wire.read() << 8 | Wire.read();
    az_raw = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // temperature (skip)
    gx_raw = Wire.read() << 8 | Wire.read();
    gy_raw = Wire.read() << 8 | Wire.read();
    gz_raw = Wire.read() << 8 | Wire.read();
}

void setup() {
    Serial.begin(115200);
    mpu6050_init();

    float dt = SAMPLE_PERIOD_MS / 1000.0f;
    ekf_attitude_init(&ekf, dt, 0.001f, 0.003f, 0.03f);

    Serial.println("time_ms,accel_roll,accel_pitch,ekf_roll,ekf_pitch");
}

void loop() {
    static uint32_t last_time = 0;
    uint32_t now = millis();

    if (now - last_time >= SAMPLE_PERIOD_MS) {
        last_time = now;

        mpu6050_read();

        // 轉換為物理單位
        float ax = ax_raw / 16384.0f;  // g (±2g range)
        float ay = ay_raw / 16384.0f;
        float az = az_raw / 16384.0f;
        float gx = gx_raw / 131.0f * 0.01745329f;  // rad/s (±250°/s)
        float gy = gy_raw / 131.0f * 0.01745329f;

        // EKF
        ekf_attitude_predict(&ekf, gx, gy);
        ekf_attitude_update(&ekf, ax, ay, az);

        // 加速度計直接計算 (用於比較)
        float accel_roll  = atan2(ay, az) * 57.2957795f;
        float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2957795f;

        Serial.print(now); Serial.print(",");
        Serial.print(accel_roll, 2); Serial.print(",");
        Serial.print(accel_pitch, 2); Serial.print(",");
        Serial.print(ekf_get_roll_deg(&ekf), 2); Serial.print(",");
        Serial.println(ekf_get_pitch_deg(&ekf), 2);
    }
}
```

---

## 10. 各平台部署指南

### 10.1 Arduino (UNO / Mega / Nano)

```bash
# 專案結構
arduino_poc/
├── pid.h
├── pid.c          → 重命名為 pid.cpp (Arduino IDE 限制)
├── kalman.h
├── kalman.c       → 重命名為 kalman.cpp
├── ekf_attitude.h
├── ekf_attitude.c → 重命名為 ekf_attitude.cpp
├── poc1_pid_temperature/
│   └── poc1_pid_temperature.ino
├── poc2_kalman_filter/
│   └── poc2_kalman_filter.ino
├── poc3_kalman_pid_motor/
│   └── poc3_kalman_pid_motor.ino
└── poc4_ekf_imu/
    └── poc4_ekf_imu.ino
```

**注意事項:**
- Arduino UNO 無 FPU，float 運算慢。考慮用 fixed-point 或降低取樣率
- 將 `.c` 檔改為 `.cpp`
- 使用 `Serial Plotter` 即時觀察波形

### 10.2 Raspberry Pi (Python + C)

```python
#!/usr/bin/env python3
"""
poc_rpi.py - Raspberry Pi PoC 驗證腳本
使用 ctypes 呼叫 C 核心, 搭配 matplotlib 即時視覺化
"""
import ctypes
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 編譯 C 程式庫
# gcc -shared -fPIC -o libcontrol.so pid.c kalman.c ekf_attitude.c -lm

lib = ctypes.CDLL('./libcontrol.so')

# --- PID 結構體映射 ---
class PID_t(ctypes.Structure):
    _fields_ = [
        ('kp', ctypes.c_float),
        ('ki', ctypes.c_float),
        ('kd', ctypes.c_float),
        ('dt', ctypes.c_float),
        ('out_min', ctypes.c_float),
        ('out_max', ctypes.c_float),
        ('integral_min', ctypes.c_float),
        ('integral_max', ctypes.c_float),
        ('integral', ctypes.c_float),
        ('prev_measurement', ctypes.c_float),
        ('prev_error', ctypes.c_float),
        ('initialized', ctypes.c_int),
    ]

class KF1D_t(ctypes.Structure):
    _fields_ = [
        ('x_est', ctypes.c_float),
        ('p_est', ctypes.c_float),
        ('q', ctypes.c_float),
        ('r', ctypes.c_float),
        ('k', ctypes.c_float),
    ]

# --- 函式簽章 ---
lib.pid_init.argtypes = [ctypes.POINTER(PID_t), ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float]
lib.pid_init.restype = None

lib.pid_compute.argtypes = [ctypes.POINTER(PID_t), ctypes.c_float, ctypes.c_float]
lib.pid_compute.restype = ctypes.c_float

lib.kf1d_init.argtypes = [ctypes.POINTER(KF1D_t), ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float]
lib.kf1d_init.restype = None

lib.kf1d_update.argtypes = [ctypes.POINTER(KF1D_t), ctypes.c_float]
lib.kf1d_update.restype = ctypes.c_float


def run_simulation():
    """模擬 Kalman + PID 控制迴路"""
    dt = 0.02  # 50 Hz
    steps = 500

    pid = PID_t()
    lib.pid_init(ctypes.byref(pid), 2.0, 0.5, 0.1, dt, -100.0, 100.0)

    kf = KF1D_t()
    lib.kf1d_init(ctypes.byref(kf), 0.01, 1.0, 0.0, 1.0)

    setpoint = 50.0
    plant_state = 0.0   # 簡單一階系統
    tau = 1.0            # 時間常數

    log = {'time': [], 'setpoint': [], 'true': [],
           'noisy': [], 'filtered': [], 'output': []}

    for i in range(steps):
        t = i * dt

        # 感測器雜訊
        noise = np.random.normal(0, 2.0)
        measurement = plant_state + noise

        # Kalman Filter
        filtered = lib.kf1d_update(ctypes.byref(kf), measurement)

        # PID
        output = lib.pid_compute(ctypes.byref(pid), setpoint, filtered)

        # 模擬一階系統: dx/dt = (-x + K*u) / tau
        plant_state += (-plant_state + output) / tau * dt

        log['time'].append(t)
        log['setpoint'].append(setpoint)
        log['true'].append(plant_state)
        log['noisy'].append(measurement)
        log['filtered'].append(filtered)
        log['output'].append(output)

    return log


def plot_results(log):
    """視覺化結果"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(log['time'], log['setpoint'], 'r--', label='Setpoint')
    axes[0].plot(log['time'], log['noisy'], 'gray', alpha=0.4, label='Noisy Sensor')
    axes[0].plot(log['time'], log['filtered'], 'b-', label='Kalman Estimate')
    axes[0].plot(log['time'], log['true'], 'g-', linewidth=2, label='True State')
    axes[0].set_ylabel('Value')
    axes[0].legend()
    axes[0].set_title('Kalman Filter + PID Control Simulation')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(log['time'], log['output'], 'orange', label='PID Output')
    axes[1].set_ylabel('Control Output')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('kalman_pid_result.png', dpi=150)
    plt.show()


if __name__ == '__main__':
    log = run_simulation()
    plot_results(log)
    print("Simulation complete. Results saved to kalman_pid_result.png")
```

**Raspberry Pi 編譯指令:**
```bash
# 編譯共享函式庫
gcc -shared -fPIC -O2 -o libcontrol.so pid.c kalman.c ekf_attitude.c -lm

# 安裝 Python 依賴
pip3 install numpy matplotlib

# 執行
python3 poc_rpi.py

# 若使用實際 GPIO (需要 pigpio)
sudo apt install pigpio python3-pigpio
sudo pigpiod
```

### 10.3 STM32 (STM32CubeIDE / PlatformIO)

```bash
# PlatformIO 專案結構
stm32_poc/
├── platformio.ini
├── lib/
│   ├── pid/
│   │   ├── pid.h
│   │   └── pid.c
│   ├── kalman/
│   │   ├── kalman.h
│   │   └── kalman.c
│   └── ekf/
│       ├── ekf_attitude.h
│       └── ekf_attitude.c
└── src/
    └── main.c
```

```ini
; platformio.ini
[env:nucleo_f446re]
platform = ststm32
board = nucleo_f446re
framework = stm32cube
build_flags = -DUSE_HAL_DRIVER -DSTM32F446xx
monitor_speed = 115200
```

```c
// src/main.c (STM32 HAL 框架)
#include "main.h"
#include "pid.h"
#include "kalman.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;   // PWM
TIM_HandleTypeDef htim3;   // Encoder
UART_HandleTypeDef huart2;

PID_t pid;
KalmanFilter1D_t kf;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);  // PWM
static void MX_TIM3_Init(void);  // Encoder mode
static void MX_USART2_UART_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    // 啟動 PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // 啟動 Encoder mode
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    float dt = 0.02f;  // 50 Hz
    pid_init(&pid, 2.0f, 0.5f, 0.1f, dt, 0.0f, 999.0f);
    kf1d_init(&kf, 0.01f, 1.0f, 0.0f, 1.0f);

    uint32_t last_tick = HAL_GetTick();
    char buf[128];

    while (1) {
        uint32_t now = HAL_GetTick();
        if (now - last_tick >= 20) {  // 50 Hz
            last_tick = now;

            // 讀取 ADC (溫度)
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 10);
            uint16_t adc_raw = HAL_ADC_GetValue(&hadc1);
            float measurement = (float)adc_raw * 3.3f / 4095.0f * 100.0f;

            // Kalman + PID
            float filtered = kf1d_update(&kf, measurement);
            float output = pid_compute(&pid, 50.0f, filtered);

            // 設定 PWM duty
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)output);

            // UART 輸出
            int len = snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f\r\n",
                               now, measurement, filtered, output);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
        }
    }
}
```

### 10.4 BeagleBone Black (Linux + sysfs)

```c
// ═══════════════════════════════════════════════
// poc_bbb.c - BeagleBone Black PoC
// ═══════════════════════════════════════════════
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include "pid.h"
#include "kalman.h"

// BeagleBone ADC: /sys/bus/iio/devices/iio:device0/in_voltage0_raw
// BeagleBone PWM: /sys/class/pwm/pwmchip0/pwm-0:0/duty_cycle

#define ADC_PATH "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define PWM_PERIOD_PATH "/sys/class/pwm/pwmchip0/pwm-0:0/period"
#define PWM_DUTY_PATH   "/sys/class/pwm/pwmchip0/pwm-0:0/duty_cycle"
#define PWM_ENABLE_PATH "/sys/class/pwm/pwmchip0/pwm-0:0/enable"

float read_adc() {
    FILE* f = fopen(ADC_PATH, "r");
    if (!f) return 0.0f;
    int raw;
    fscanf(f, "%d", &raw);
    fclose(f);
    return (float)raw * 1.8f / 4095.0f;  // BBB ADC: 0~1.8V, 12-bit
}

void set_pwm_duty(float duty_percent) {
    // 假設 period = 1000000 ns (1 kHz)
    int duty_ns = (int)(duty_percent / 100.0f * 1000000.0f);
    if (duty_ns < 0) duty_ns = 0;
    if (duty_ns > 1000000) duty_ns = 1000000;

    FILE* f = fopen(PWM_DUTY_PATH, "w");
    if (f) {
        fprintf(f, "%d", duty_ns);
        fclose(f);
    }
}

uint64_t get_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

int main() {
    PID_t pid;
    KalmanFilter1D_t kf;

    float dt = 0.02f;
    pid_init(&pid, 5.0f, 1.0f, 0.3f, dt, 0.0f, 100.0f);
    kf1d_init(&kf, 0.01f, 0.5f, 25.0f, 1.0f);

    float setpoint = 50.0f;

    printf("time_ms,raw,filtered,output\n");

    uint64_t last_time = get_time_ms();

    while (1) {
        uint64_t now = get_time_ms();
        if (now - last_time >= 20) {
            last_time = now;

            float raw_voltage = read_adc();
            float temperature = raw_voltage * 100.0f;  // 簡化: TMP36 式轉換

            float filtered = kf1d_update(&kf, temperature);
            float output = pid_compute(&pid, setpoint, filtered);

            set_pwm_duty(output);

            printf("%llu,%.2f,%.2f,%.2f\n",
                   (unsigned long long)now, temperature, filtered, output);
            fflush(stdout);
        }
        usleep(1000);  // 1ms sleep 避免 CPU 空轉
    }

    return 0;
}
```

**BeagleBone 編譯與執行:**
```bash
# 編譯
gcc -O2 -o poc_bbb poc_bbb.c pid.c kalman.c -lm -lrt

# 設定 PWM (需要先載入 overlay)
echo 0 > /sys/class/pwm/pwmchip0/export
echo 1000000 > /sys/class/pwm/pwmchip0/pwm-0:0/period
echo 1 > /sys/class/pwm/pwmchip0/pwm-0:0/enable

# 執行 (需要 root 或適當權限)
sudo ./poc_bbb | tee data_log.csv
```

---

## 11. Claude Code 驗證腳本

### 11.1 驗證策略

以下腳本可在 Claude Code 中執行，用純 C 模擬驗證各 PoC 的邏輯正確性，不需實際硬體。

### 11.2 自動化驗證腳本

```bash
#!/bin/bash
# ═══════════════════════════════════════════════
# verify_all.sh - Claude Code 驗證主腳本
# ═══════════════════════════════════════════════
set -e

echo "=== PID & Kalman Filter PoC 驗證 ==="
echo ""

# 建立目錄
mkdir -p build tests/output

# 編譯所有模組
echo "[1/5] 編譯模組..."
gcc -Wall -Wextra -O2 -c pid.c -o build/pid.o -lm
gcc -Wall -Wextra -O2 -c kalman.c -o build/kalman.o -lm
gcc -Wall -Wextra -O2 -c ekf_attitude.c -o build/ekf.o -lm
echo "  ✓ 編譯成功"

# 測試 1: PID 單元測試
echo ""
echo "[2/5] PID 單元測試..."
gcc -Wall -O2 -o build/test_pid tests/test_pid.c pid.c -lm
./build/test_pid
echo "  ✓ PID 測試通過"

# 測試 2: Kalman Filter 單元測試
echo ""
echo "[3/5] Kalman Filter 單元測試..."
gcc -Wall -O2 -o build/test_kalman tests/test_kalman.c kalman.c -lm
./build/test_kalman
echo "  ✓ Kalman Filter 測試通過"

# 測試 3: 整合模擬
echo ""
echo "[4/5] Kalman + PID 整合模擬..."
gcc -Wall -O2 -o build/test_integration tests/test_integration.c pid.c kalman.c -lm
./build/test_integration > tests/output/integration_results.csv
echo "  ✓ 整合模擬完成 → tests/output/integration_results.csv"

# 測試 4: EKF 測試
echo ""
echo "[5/5] EKF 姿態估測測試..."
gcc -Wall -O2 -o build/test_ekf tests/test_ekf.c ekf_attitude.c -lm
./build/test_ekf
echo "  ✓ EKF 測試通過"

echo ""
echo "=== 所有驗證完成 ==="
```

### 11.3 PID 單元測試

```c
// ═══════════════════════════════════════════════
// tests/test_pid.c - PID Unit Tests
// ═══════════════════════════════════════════════
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "../pid.h"

#define ASSERT_NEAR(actual, expected, tol, msg) do {        \
    if (fabs((actual) - (expected)) > (tol)) {              \
        printf("  FAIL: %s (actual=%.4f, expected=%.4f)\n", \
               msg, (float)(actual), (float)(expected));    \
        return 1;                                           \
    }                                                       \
} while(0)

int test_p_only() {
    PID_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.1f, -100.0f, 100.0f);

    float out = pid_compute(&pid, 10.0f, 0.0f);
    ASSERT_NEAR(out, 10.0f, 0.01f, "P-only: error=10 → output=10");

    out = pid_compute(&pid, 10.0f, 10.0f);
    ASSERT_NEAR(out, 0.0f, 0.01f, "P-only: error=0 → output=0");

    printf("  ✓ test_p_only passed\n");
    return 0;
}

int test_pi_steady_state() {
    PID_t pid;
    pid_init(&pid, 1.0f, 10.0f, 0.0f, 0.01f, -100.0f, 100.0f);

    // 持續給定 error=5 的情況下，積分項應持續增加
    float out1 = pid_compute(&pid, 50.0f, 45.0f);
    float out2 = pid_compute(&pid, 50.0f, 45.0f);
    float out3 = pid_compute(&pid, 50.0f, 45.0f);

    assert(out3 > out2 && out2 > out1);
    printf("  ✓ test_pi_steady_state passed (integral accumulates)\n");
    return 0;
}

int test_anti_windup() {
    PID_t pid;
    pid_init(&pid, 1.0f, 100.0f, 0.0f, 0.1f, -10.0f, 10.0f);

    // 持續飽和
    for (int i = 0; i < 1000; i++) {
        pid_compute(&pid, 100.0f, 0.0f);
    }

    // 輸出應被限制
    float out = pid_compute(&pid, 100.0f, 0.0f);
    ASSERT_NEAR(out, 10.0f, 0.01f, "Anti-windup: output clamped at max");

    // 突然達到 setpoint，不應有過度 windup
    out = pid_compute(&pid, 0.0f, 0.0f);
    // 積分應被限制，不會有巨大負值
    assert(out >= -10.0f);
    printf("  ✓ test_anti_windup passed\n");
    return 0;
}

int test_output_clamping() {
    PID_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.1f, 0.0f, 255.0f);

    float out = pid_compute(&pid, 1000.0f, 0.0f);
    ASSERT_NEAR(out, 255.0f, 0.01f, "Output clamped at 255");

    out = pid_compute(&pid, -1000.0f, 0.0f);
    ASSERT_NEAR(out, 0.0f, 0.01f, "Output clamped at 0");

    printf("  ✓ test_output_clamping passed\n");
    return 0;
}

int test_derivative_on_measurement() {
    PID_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.1f, -100.0f, 100.0f);

    // 第一次呼叫 (initialized=0) → D 項應為 0
    float out1 = pid_compute(&pid, 10.0f, 5.0f);
    ASSERT_NEAR(out1, 0.0f, 0.01f, "D first call = 0");

    // 第二次呼叫，measurement 從 5 變到 8
    // D = -Kd * (8 - 5) / 0.1 = -1 * 30 = -30
    float out2 = pid_compute(&pid, 10.0f, 8.0f);
    ASSERT_NEAR(out2, -30.0f, 0.01f, "D on measurement");

    printf("  ✓ test_derivative_on_measurement passed\n");
    return 0;
}

int main() {
    printf("--- PID Controller Unit Tests ---\n");
    int failures = 0;
    failures += test_p_only();
    failures += test_pi_steady_state();
    failures += test_anti_windup();
    failures += test_output_clamping();
    failures += test_derivative_on_measurement();

    if (failures == 0) {
        printf("All PID tests passed!\n");
    } else {
        printf("%d test(s) failed!\n", failures);
    }
    return failures;
}
```

### 11.4 Kalman Filter 單元測試

```c
// ═══════════════════════════════════════════════
// tests/test_kalman.c - Kalman Filter Unit Tests
// ═══════════════════════════════════════════════
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "../kalman.h"

#define ASSERT_NEAR(actual, expected, tol, msg) do {        \
    if (fabs((actual) - (expected)) > (tol)) {              \
        printf("  FAIL: %s (actual=%.6f, expected=%.6f)\n", \
               msg, (float)(actual), (float)(expected));    \
        return 1;                                           \
    }                                                       \
} while(0)

int test_constant_signal() {
    KalmanFilter1D_t kf;
    kf1d_init(&kf, 0.01f, 1.0f, 0.0f, 10.0f);

    // 送入穩定的 50.0 + 小雜訊
    float true_val = 50.0f;
    for (int i = 0; i < 100; i++) {
        float noise = ((float)(rand() % 1000) / 500.0f - 1.0f) * 2.0f;
        kf1d_update(&kf, true_val + noise);
    }

    ASSERT_NEAR(kf.x_est, 50.0f, 1.0f, "Converges to 50.0");
    printf("  ✓ test_constant_signal passed (estimate=%.2f)\n", kf.x_est);
    return 0;
}

int test_kalman_gain_convergence() {
    KalmanFilter1D_t kf;
    kf1d_init(&kf, 0.01f, 1.0f, 0.0f, 100.0f);  // 大 P0 → K 從大開始

    float k_values[20];
    for (int i = 0; i < 20; i++) {
        kf1d_update(&kf, 50.0f);
        k_values[i] = kf.k;
    }

    // K 應該單調遞減 (收斂)
    for (int i = 1; i < 20; i++) {
        if (k_values[i] > k_values[i-1] + 0.001f) {
            printf("  FAIL: K not decreasing at step %d\n", i);
            return 1;
        }
    }

    printf("  ✓ test_kalman_gain_convergence passed (K: %.4f → %.4f)\n",
           k_values[0], k_values[19]);
    return 0;
}

int test_noise_reduction() {
    KalmanFilter1D_t kf;
    kf1d_init(&kf, 0.001f, 4.0f, 25.0f, 1.0f);  // R=4: 雜訊 std≈2

    float true_val = 25.0f;
    float sum_raw_error2 = 0.0f;
    float sum_kf_error2 = 0.0f;

    srand(42);
    for (int i = 0; i < 500; i++) {
        float noise = ((float)(rand() % 10000) / 5000.0f - 1.0f) * 2.0f;
        float measurement = true_val + noise;
        float estimate = kf1d_update(&kf, measurement);

        if (i >= 50) {  // 跳過暖機期
            sum_raw_error2 += (measurement - true_val) * (measurement - true_val);
            sum_kf_error2  += (estimate - true_val) * (estimate - true_val);
        }
    }

    float rmse_raw = sqrt(sum_raw_error2 / 450.0f);
    float rmse_kf  = sqrt(sum_kf_error2 / 450.0f);

    printf("  Raw RMSE: %.4f, KF RMSE: %.4f (reduction: %.1f%%)\n",
           rmse_raw, rmse_kf, (1.0f - rmse_kf/rmse_raw) * 100.0f);

    if (rmse_kf >= rmse_raw) {
        printf("  FAIL: KF should reduce noise\n");
        return 1;
    }

    printf("  ✓ test_noise_reduction passed\n");
    return 0;
}

int test_2d_position_velocity() {
    KalmanFilter2D_t kf;
    kf2d_init(&kf, 0.1f, 1.0f, 1.0f, 0.1f);

    // 模擬等速運動: pos = 2*t, vel = 2
    for (int i = 0; i < 100; i++) {
        float t = i * 0.1f;
        float true_pos = 2.0f * t;
        float noise = ((float)(rand() % 1000) / 500.0f - 1.0f);
        float measured_pos = true_pos + noise;

        kf2d_predict(&kf);
        kf2d_update(&kf, measured_pos);
    }

    // 速度估測應接近 2.0
    ASSERT_NEAR(kf.x[1], 2.0f, 0.5f, "Velocity estimate ≈ 2.0");
    printf("  ✓ test_2d_position_velocity passed (vel=%.2f)\n", kf.x[1]);
    return 0;
}

int main() {
    printf("--- Kalman Filter Unit Tests ---\n");
    int failures = 0;
    failures += test_constant_signal();
    failures += test_kalman_gain_convergence();
    failures += test_noise_reduction();
    failures += test_2d_position_velocity();

    if (failures == 0) {
        printf("All Kalman Filter tests passed!\n");
    } else {
        printf("%d test(s) failed!\n", failures);
    }
    return failures;
}
```

### 11.5 整合模擬測試

```c
// ═══════════════════════════════════════════════
// tests/test_integration.c - Kalman + PID Integration Simulation
// ═══════════════════════════════════════════════
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "../pid.h"
#include "../kalman.h"

/**
 * 模擬一階延遲系統 (first-order lag)
 * G(s) = K / (tau*s + 1)
 *
 * 離散化: x[k+1] = x[k] + (K * u[k] - x[k]) / tau * dt
 */
typedef struct {
    float state;
    float K;      // 系統增益
    float tau;    // 時間常數
} Plant_t;

float plant_step(Plant_t* p, float u, float dt) {
    p->state += (p->K * u - p->state) / p->tau * dt;
    return p->state;
}

int main() {
    float dt = 0.02f;
    int total_steps = 2000;  // 40 秒

    // 系統
    Plant_t plant = { .state = 0.0f, .K = 1.0f, .tau = 2.0f };

    // PID
    PID_t pid;
    pid_init(&pid, 3.0f, 1.0f, 0.5f, dt, 0.0f, 100.0f);

    // Kalman Filter
    KalmanFilter1D_t kf;
    kf1d_init(&kf, 0.01f, 4.0f, 0.0f, 10.0f);

    // 也測試不用 Kalman 的 PID
    PID_t pid_no_kf;
    pid_init(&pid_no_kf, 3.0f, 1.0f, 0.5f, dt, 0.0f, 100.0f);
    Plant_t plant_no_kf = { .state = 0.0f, .K = 1.0f, .tau = 2.0f };

    float setpoint = 50.0f;

    printf("time,setpoint,true_kf,noisy,filtered,pid_kf_out,true_no_kf,pid_no_kf_out\n");

    srand(123);
    for (int i = 0; i < total_steps; i++) {
        float t = i * dt;

        // 在 t=20s 時改變 setpoint
        if (t >= 20.0f) setpoint = 30.0f;

        // --- 方案 A: Kalman + PID ---
        float noise = ((float)(rand() % 10000) / 5000.0f - 1.0f) * 3.0f;
        float measurement = plant.state + noise;
        float filtered = kf1d_update(&kf, measurement);
        float pid_out = pid_compute(&pid, setpoint, filtered);
        plant_step(&plant, pid_out, dt);

        // --- 方案 B: PID only (no Kalman) ---
        float noise2 = ((float)(rand() % 10000) / 5000.0f - 1.0f) * 3.0f;
        float meas_no_kf = plant_no_kf.state + noise2;
        float pid_no_kf_out = pid_compute(&pid_no_kf, setpoint, meas_no_kf);
        plant_step(&plant_no_kf, pid_no_kf_out, dt);

        printf("%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               t, setpoint, plant.state, measurement, filtered,
               pid_out, plant_no_kf.state, pid_no_kf_out);
    }

    // 計算最後 500 步的性能指標
    // (重新跑一次統計)
    fprintf(stderr, "\n--- Performance Summary ---\n");
    fprintf(stderr, "Compare the output smoothness and tracking accuracy\n");
    fprintf(stderr, "in the generated CSV file.\n");

    return 0;
}
```

### 11.6 EKF 驗證測試

```c
// ═══════════════════════════════════════════════
// tests/test_ekf.c - EKF Attitude Estimation Tests
// ═══════════════════════════════════════════════
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "../ekf_attitude.h"

#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f

int test_static_level() {
    EKF_Attitude_t ekf;
    ekf_attitude_init(&ekf, 0.01f, 0.001f, 0.003f, 0.03f);

    // 模擬靜止水平放置: accel = (0, 0, 1g), gyro = (0, 0, 0)
    for (int i = 0; i < 200; i++) {
        ekf_attitude_predict(&ekf, 0.0f, 0.0f);
        ekf_attitude_update(&ekf, 0.0f, 0.0f, 1.0f);
    }

    float roll = ekf_get_roll_deg(&ekf);
    float pitch = ekf_get_pitch_deg(&ekf);

    if (fabs(roll) > 1.0f || fabs(pitch) > 1.0f) {
        printf("  FAIL: Static level should be ~0°, got roll=%.2f pitch=%.2f\n",
               roll, pitch);
        return 1;
    }

    printf("  ✓ test_static_level passed (roll=%.2f°, pitch=%.2f°)\n", roll, pitch);
    return 0;
}

int test_tilted_30deg() {
    EKF_Attitude_t ekf;
    ekf_attitude_init(&ekf, 0.01f, 0.001f, 0.003f, 0.03f);

    // 模擬 roll=30° 傾斜
    float roll_true = 30.0f * DEG_TO_RAD;
    float ax = 0.0f;
    float ay = sin(roll_true);
    float az = cos(roll_true);

    for (int i = 0; i < 500; i++) {
        float noise_gx = ((float)(rand() % 1000) / 500.0f - 1.0f) * 0.01f;
        float noise_gy = ((float)(rand() % 1000) / 500.0f - 1.0f) * 0.01f;
        float noise_ax = ((float)(rand() % 1000) / 500.0f - 1.0f) * 0.05f;
        float noise_ay = ((float)(rand() % 1000) / 500.0f - 1.0f) * 0.05f;
        float noise_az = ((float)(rand() % 1000) / 500.0f - 1.0f) * 0.05f;

        ekf_attitude_predict(&ekf, noise_gx, noise_gy);
        ekf_attitude_update(&ekf, ax + noise_ax, ay + noise_ay, az + noise_az);
    }

    float roll_est = ekf_get_roll_deg(&ekf);
    if (fabs(roll_est - 30.0f) > 3.0f) {
        printf("  FAIL: Expected roll≈30°, got %.2f°\n", roll_est);
        return 1;
    }

    printf("  ✓ test_tilted_30deg passed (roll=%.2f°, expected=30.0°)\n", roll_est);
    return 0;
}

int test_gyro_bias_estimation() {
    EKF_Attitude_t ekf;
    ekf_attitude_init(&ekf, 0.01f, 0.001f, 0.003f, 0.03f);

    // 模擬有偏移的陀螺儀 (bias = 0.05 rad/s)
    float gyro_bias = 0.05f;

    for (int i = 0; i < 2000; i++) {
        ekf_attitude_predict(&ekf, gyro_bias, 0.0f);
        ekf_attitude_update(&ekf, 0.0f, 0.0f, 1.0f);  // 實際水平
    }

    float roll = ekf_get_roll_deg(&ekf);
    float estimated_bias = ekf.x[2];

    printf("  Gyro bias true=%.4f, estimated=%.4f, roll=%.2f°\n",
           gyro_bias, estimated_bias, roll);

    if (fabs(roll) > 5.0f) {
        printf("  FAIL: Roll should be near 0 despite gyro bias\n");
        return 1;
    }

    printf("  ✓ test_gyro_bias_estimation passed\n");
    return 0;
}

int main() {
    printf("--- EKF Attitude Tests ---\n");
    srand(77);
    int failures = 0;
    failures += test_static_level();
    failures += test_tilted_30deg();
    failures += test_gyro_bias_estimation();

    if (failures == 0) {
        printf("All EKF tests passed!\n");
    } else {
        printf("%d test(s) failed!\n", failures);
    }
    return failures;
}
```

---

## 12. 效能基準測試

### 12.1 計算時間基準

```c
// ═══════════════════════════════════════════════
// tests/benchmark.c - Performance Benchmark
// ═══════════════════════════════════════════════
#include <stdio.h>
#include <time.h>
#include "../pid.h"
#include "../kalman.h"
#include "../ekf_attitude.h"

#define ITERATIONS 100000

double benchmark(void (*func)(void), const char* name) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (int i = 0; i < ITERATIONS; i++) {
        func();
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed_us = (end.tv_sec - start.tv_sec) * 1e6 +
                        (end.tv_nsec - start.tv_nsec) / 1e3;
    double per_call = elapsed_us / ITERATIONS;

    printf("  %-30s: %.2f µs/call (%d iterations)\n", name, per_call, ITERATIONS);
    return per_call;
}

// 全域物件 (避免 benchmark 包含初始化)
PID_t g_pid;
KalmanFilter1D_t g_kf1d;
KalmanFilter2D_t g_kf2d;
EKF_Attitude_t g_ekf;

void bench_pid() { pid_compute(&g_pid, 50.0f, 48.5f); }
void bench_kf1d() { kf1d_update(&g_kf1d, 25.3f); }
void bench_kf2d() { kf2d_predict(&g_kf2d); kf2d_update(&g_kf2d, 100.5f); }
void bench_ekf() {
    ekf_attitude_predict(&g_ekf, 0.01f, 0.02f);
    ekf_attitude_update(&g_ekf, 0.1f, 0.2f, 0.98f);
}

int main() {
    pid_init(&g_pid, 2.0f, 0.5f, 0.1f, 0.02f, 0.0f, 100.0f);
    kf1d_init(&g_kf1d, 0.01f, 1.0f, 25.0f, 1.0f);
    kf2d_init(&g_kf2d, 0.1f, 1.0f, 1.0f, 0.02f);
    ekf_attitude_init(&g_ekf, 0.01f, 0.001f, 0.003f, 0.03f);

    printf("--- Performance Benchmark ---\n");
    printf("  (Lower is better)\n\n");

    double t_pid  = benchmark(bench_pid, "PID compute");
    double t_kf1d = benchmark(bench_kf1d, "Kalman 1D update");
    double t_kf2d = benchmark(bench_kf2d, "Kalman 2D predict+update");
    double t_ekf  = benchmark(bench_ekf, "EKF predict+update");

    printf("\n--- Platform Feasibility (at 1 kHz loop) ---\n");
    printf("  Available time per loop: 1000 µs\n");
    printf("  PID + KF1D total: %.2f µs (%.1f%% budget)\n",
           t_pid + t_kf1d, (t_pid + t_kf1d) / 10.0f);
    printf("  PID + KF2D total: %.2f µs (%.1f%% budget)\n",
           t_pid + t_kf2d, (t_pid + t_kf2d) / 10.0f);
    printf("  PID + EKF total:  %.2f µs (%.1f%% budget)\n",
           t_pid + t_ekf, (t_pid + t_ekf) / 10.0f);

    return 0;
}
```

### 12.2 預期基準值

| 運算 | Cortex-M4 (168MHz) | RPi4 (1.5GHz) | ATmega328P (16MHz) |
|------|-------------------|---------------|-------------------|
| PID compute | ~2 µs | ~0.1 µs | ~50 µs |
| KF 1D update | ~1 µs | ~0.05 µs | ~30 µs |
| KF 2D predict+update | ~5 µs | ~0.2 µs | ~150 µs |
| EKF 4-state | ~15 µs | ~0.5 µs | ~500 µs |

---

## 13. 故障排除與常見問題

### 13.1 PID 問題

| 症狀 | 可能原因 | 解決方案 |
|------|---------|---------|
| 持續振盪 | Kp 過大 | 降低 Kp，增加 Kd |
| 穩態誤差 | 無 I 項或 Ki 太小 | 增加 Ki |
| 超調後長時間回不來 | Integral windup | 啟用 anti-windup clamping |
| 輸出劇烈跳動 | Derivative kick | 改用 derivative on measurement |
| 回應太慢 | Kp 太小 | 逐步增加 Kp |

### 13.2 Kalman Filter 問題

| 症狀 | 可能原因 | 解決方案 |
|------|---------|---------|
| 濾波後仍有很多雜訊 | Q 太大或 R 太小 | 減小 Q/R 比值 |
| 追蹤太慢 (延遲大) | Q 太小或 R 太大 | 增大 Q/R 比值 |
| 估測值發散 | 數值不穩定 | 檢查 P 矩陣正定性，用 Joseph form |
| 初始收斂慢 | P0 太小 | 增大初始 P0 |

### 13.3 Arduino 特定問題

- float 運算慢 → 考慮 fixed-point (Q16.16 格式)
- RAM 不足 → 減少 log buffer，用 PROGMEM 存常數
- Serial 輸出影響 timing → 降低 baud rate 或減少輸出頻率

---

## 14. 參考資源

### 書籍
- "Feedback Control of Dynamic Systems" - Franklin, Powell, Emami-Naeini
- "Optimal State Estimation" - Dan Simon
- "Probabilistic Robotics" - Thrun, Burgard, Fox

### 線上教學
- PID: Brett Beauregard 的 Arduino PID Library 文件
- Kalman: Roger Labbe 的 "Kalman and Bayesian Filters in Python" (GitHub)
- EKF: Joan Solà 的 "Quaternion kinematics for the error-state Kalman filter"

### 工具
- Serial Plotter: Arduino IDE 內建
- Python 視覺化: matplotlib, plotly
- MATLAB/Simulink: PID Tuner, Kalman Filter Designer

---

## 附錄 A: Claude Code 快速驗證命令

```bash
# 一鍵驗證 (在專案根目錄執行)
chmod +x verify_all.sh && ./verify_all.sh

# 單獨編譯測試
gcc -Wall -O2 -o test_pid tests/test_pid.c pid.c -lm && ./test_pid
gcc -Wall -O2 -o test_kalman tests/test_kalman.c kalman.c -lm && ./test_kalman
gcc -Wall -O2 -o test_integration tests/test_integration.c pid.c kalman.c -lm && ./test_integration
gcc -Wall -O2 -o test_ekf tests/test_ekf.c ekf_attitude.c -lm && ./test_ekf
gcc -Wall -O2 -o benchmark tests/benchmark.c pid.c kalman.c ekf_attitude.c -lm && ./benchmark
```

## 附錄 B: 專案完整目錄結構

```
pid_kalman_poc/
├── README.md                    ← 專案總覽與教學
├── PID_KalmanFilter_Teaching_PoC.md ← 本文件
├── Inverted_Pendulum_Guide.md   ← Level 5: 倒單擺控制完整指南
├── inverted_pendulum_circuit.drawio ← Level 5: 電路圖 (DrawIO)
├── verify_all.sh                ← Claude Code 驗證主腳本
│
├── pid.h                        ← PID 控制器標頭
├── pid.c                        ← PID 控制器實作
├── kalman.h                     ← Kalman Filter 標頭
├── kalman.c                     ← Kalman Filter 實作
├── ekf_attitude.h               ← EKF 姿態估測標頭
├── ekf_attitude.c               ← EKF 姿態估測實作
├── hal.h                        ← 硬體抽象層介面
│
├── tests/
│   ├── test_pid.c               ← PID 單元測試
│   ├── test_kalman.c            ← Kalman Filter 單元測試
│   ├── test_integration.c       ← 整合模擬測試
│   ├── test_ekf.c               ← EKF 單元測試
│   ├── benchmark.c              ← 效能基準測試
│   └── output/                  ← 測試輸出 CSV
│
├── arduino/
│   ├── poc1_pid_temperature/
│   ├── poc2_kalman_filter/
│   ├── poc3_kalman_pid_motor/
│   └── poc4_ekf_imu/
│
├── rpi/
│   ├── poc_rpi.py
│   ├── Makefile
│   └── libcontrol.so            ← (編譯產物)
│
├── stm32/
│   ├── platformio.ini
│   ├── src/main.c
│   └── lib/
│
└── beaglebone/
    ├── poc_bbb.c
    └── Makefile
```
