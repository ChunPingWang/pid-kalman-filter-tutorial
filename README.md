# PID Controller & Kalman Filter Tutorial

PID 控制器與卡爾曼濾波器的嵌入式系統教學專案，包含理論基礎、跨平台實作與驗證測試。

## 學習路徑

```
Level 1: PID 基礎          -> PoC 1 (溫度控制)
Level 2: Kalman Filter 基礎 -> PoC 2 (感測器濾波)
Level 3: PID + KF 整合      -> PoC 3 (馬達控制)
Level 4: EKF 進階           -> PoC 4 (姿態估測)
```

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

| PoC | 主題 | 核心模組 | 硬體需求 |
|-----|------|---------|---------|
| **PoC 1** | 純 PID 溫度控制 | `pid.c` | NTC Thermistor + MOSFET + 加熱線 |
| **PoC 2** | 1D Kalman Filter 濾波 | `kalman.c` | 任意感測器 (或純模擬) |
| **PoC 3** | Kalman + PID 馬達控制 | `pid.c` + `kalman.c` | DC 馬達 + Encoder + H-Bridge |
| **PoC 4** | EKF 姿態估測 | `ekf_attitude.c` | MPU6050 (I2C) |

## 支援平台

| 平台 | CPU | FPU | 浮點建議 |
|------|-----|-----|---------|
| Arduino UNO/Mega | ATmega328P/2560 16MHz | 無 | Fixed-point 或降低取樣率 |
| Raspberry Pi 4 | BCM2711 1.5GHz | 有 | float/double |
| STM32F4 | Cortex-M4 168MHz | 有 | float |
| BeagleBone Black | AM3358 1GHz | 有 | float/double |

## 快速開始

### 1. 執行驗證測試 (不需硬體)

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

## 核心概念

### PID Controller

```
u(t) = Kp * e(t) + Ki * integral(e) + Kd * de/dt
```

本實作包含:
- **Anti-Windup**: 積分項 clamping 防止飽和
- **Derivative on Measurement**: 避免 setpoint 突變時的 derivative kick
- **Output Clamping**: 限制輸出範圍

### Kalman Filter

```
Predict:  x_pred = F * x_est,       P_pred = F * P * F^T + Q
Update:   K = P_pred / (P_pred + R), x_est = x_pred + K * (z - x_pred)
```

- **Q/R 比值大**: 更信任觀測，追蹤快但雜訊多
- **Q/R 比值小**: 更信任模型，平滑但追蹤慢

### Extended Kalman Filter (EKF)

用於非線性系統，透過 Jacobian 矩陣線性化:
- 狀態: `[roll, pitch, gyro_bias_x, gyro_bias_y]`
- 預測: 陀螺儀積分 (去除偏移)
- 更新: 加速度計觀測角度修正

## PID 調參方法

| 方法 | 步驟 |
|------|------|
| **手動調參** | P 先調到微微振盪 -> 加 D 消除振盪 -> 加 I 消除穩態誤差 |
| **Ziegler-Nichols** | 找臨界增益 Ku 和振盪週期 Tu，查表: Kp=0.6Ku, Ki=2Kp/Tu, Kd=KpTu/8 |

## 故障排除

| 症狀 | 可能原因 | 解決方案 |
|------|---------|---------|
| PID 持續振盪 | Kp 過大 | 降低 Kp，增加 Kd |
| PID 穩態誤差 | 無 I 項 | 增加 Ki |
| KF 濾波後仍有雜訊 | Q 太大 | 減小 Q/R 比值 |
| KF 追蹤太慢 | Q 太小 | 增大 Q/R 比值 |
| EKF 估測值發散 | 數值不穩定 | 檢查 P 矩陣正定性 |

## 前置知識

- C/C++ 基礎程式設計
- 基本線性代數 (矩陣運算)
- 基本機率統計 (常態分佈、變異數)
- 嵌入式系統基礎 (GPIO, ADC, PWM, I2C/SPI)

## 參考資源

- "Feedback Control of Dynamic Systems" - Franklin, Powell, Emami-Naeini
- "Optimal State Estimation" - Dan Simon
- Roger Labbe - "Kalman and Bayesian Filters in Python" (GitHub)
- Brett Beauregard - Arduino PID Library

## License

MIT
