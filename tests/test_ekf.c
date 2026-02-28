// ===============================================
// tests/test_ekf.c - EKF Attitude Estimation Tests
// ===============================================
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "../ekf_attitude.h"

#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f

int test_static_level() {
    EKF_Attitude_t ekf;
    ekf_attitude_init(&ekf, 0.01f, 0.001f, 0.003f, 0.03f);

    // Simulate static level: accel = (0, 0, 1g), gyro = (0, 0, 0)
    for (int i = 0; i < 200; i++) {
        ekf_attitude_predict(&ekf, 0.0f, 0.0f);
        ekf_attitude_update(&ekf, 0.0f, 0.0f, 1.0f);
    }

    float roll = ekf_get_roll_deg(&ekf);
    float pitch = ekf_get_pitch_deg(&ekf);

    if (fabs(roll) > 1.0f || fabs(pitch) > 1.0f) {
        printf("  FAIL: Static level should be ~0 deg, got roll=%.2f pitch=%.2f\n",
               roll, pitch);
        return 1;
    }

    printf("  [PASS] test_static_level (roll=%.2f deg, pitch=%.2f deg)\n", roll, pitch);
    return 0;
}

int test_tilted_30deg() {
    EKF_Attitude_t ekf;
    ekf_attitude_init(&ekf, 0.01f, 0.001f, 0.003f, 0.03f);

    // Simulate roll=30 deg tilt
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
        printf("  FAIL: Expected roll~30 deg, got %.2f deg\n", roll_est);
        return 1;
    }

    printf("  [PASS] test_tilted_30deg (roll=%.2f deg, expected=30.0 deg)\n", roll_est);
    return 0;
}

int test_gyro_bias_estimation() {
    EKF_Attitude_t ekf;
    ekf_attitude_init(&ekf, 0.01f, 0.001f, 0.003f, 0.03f);

    // Simulate gyro with bias = 0.05 rad/s
    float gyro_bias = 0.05f;

    for (int i = 0; i < 2000; i++) {
        ekf_attitude_predict(&ekf, gyro_bias, 0.0f);
        ekf_attitude_update(&ekf, 0.0f, 0.0f, 1.0f);  // Actually level
    }

    float roll = ekf_get_roll_deg(&ekf);
    float estimated_bias = ekf.x[2];

    printf("  Gyro bias true=%.4f, estimated=%.4f, roll=%.2f deg\n",
           gyro_bias, estimated_bias, roll);

    if (fabs(roll) > 5.0f) {
        printf("  FAIL: Roll should be near 0 despite gyro bias\n");
        return 1;
    }

    printf("  [PASS] test_gyro_bias_estimation\n");
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
