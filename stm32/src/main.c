// ===============================================
// src/main.c - STM32 HAL Framework (Kalman + PID)
// ===============================================
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

    // Start PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // Start Encoder mode
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

            // Read ADC (temperature)
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 10);
            uint16_t adc_raw = HAL_ADC_GetValue(&hadc1);
            float measurement = (float)adc_raw * 3.3f / 4095.0f * 100.0f;

            // Kalman + PID
            float filtered = kf1d_update(&kf, measurement);
            float output = pid_compute(&pid, 50.0f, filtered);

            // Set PWM duty
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)output);

            // UART output
            int len = snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f\r\n",
                               now, measurement, filtered, output);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
        }
    }
}
