// ===============================================
// poc_bbb.c - BeagleBone Black PoC
// ===============================================
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include "../pid.h"
#include "../kalman.h"

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
    // Assume period = 1000000 ns (1 kHz)
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
            float temperature = raw_voltage * 100.0f;  // Simplified: TMP36 style

            float filtered = kf1d_update(&kf, temperature);
            float output = pid_compute(&pid, setpoint, filtered);

            set_pwm_duty(output);

            printf("%llu,%.2f,%.2f,%.2f\n",
                   (unsigned long long)now, temperature, filtered, output);
            fflush(stdout);
        }
        usleep(1000);  // 1ms sleep to avoid CPU busy-loop
    }

    return 0;
}
