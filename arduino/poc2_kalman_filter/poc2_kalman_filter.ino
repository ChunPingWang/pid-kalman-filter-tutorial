// ===============================================
// poc2_kalman_filter.ino - 1D Kalman Filter Demo
// ===============================================
#include "../../kalman.h"
#include <math.h>

#define SENSOR_PIN       A0
#define SAMPLE_PERIOD_MS 50

KalmanFilter1D_t kf;

// Simulate noisy sensor (when no actual hardware)
float simulate_noisy_sensor(float true_value, float noise_std) {
    // Box-Muller transform for Gaussian noise
    float u1 = (float)random(1, 10000) / 10000.0f;
    float u2 = (float)random(1, 10000) / 10000.0f;
    float noise = sqrt(-2.0f * log(u1)) * cos(2.0f * 3.14159f * u2) * noise_std;
    return true_value + noise;
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(A1));  // Use unconnected pin as random seed

    // Initialize Kalman Filter
    // Q=0.01 (small process noise, assume slow state change)
    // R=1.0  (measurement noise std ~ 1.0)
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

        // Simulate real temperature (slow sine wave)
        phase += 0.02f;
        true_temp = 25.0f + 5.0f * sin(phase);

        // Add noise
        float noisy_measurement = simulate_noisy_sensor(true_temp, 2.0f);

        // Kalman Filter
        float estimate = kf1d_update(&kf, noisy_measurement);

        // CSV output
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
