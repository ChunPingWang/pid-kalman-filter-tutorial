// ===============================================
// tests/test_kalman.c - Kalman Filter Unit Tests
// ===============================================
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

    // Feed stable 50.0 + small noise
    float true_val = 50.0f;
    for (int i = 0; i < 100; i++) {
        float noise = ((float)(rand() % 1000) / 500.0f - 1.0f) * 2.0f;
        kf1d_update(&kf, true_val + noise);
    }

    ASSERT_NEAR(kf.x_est, 50.0f, 1.0f, "Converges to 50.0");
    printf("  [PASS] test_constant_signal (estimate=%.2f)\n", kf.x_est);
    return 0;
}

int test_kalman_gain_convergence() {
    KalmanFilter1D_t kf;
    kf1d_init(&kf, 0.01f, 1.0f, 0.0f, 100.0f);  // Large P0 -> K starts large

    float k_values[20];
    for (int i = 0; i < 20; i++) {
        kf1d_update(&kf, 50.0f);
        k_values[i] = kf.k;
    }

    // K should monotonically decrease (converge)
    for (int i = 1; i < 20; i++) {
        if (k_values[i] > k_values[i-1] + 0.001f) {
            printf("  FAIL: K not decreasing at step %d\n", i);
            return 1;
        }
    }

    printf("  [PASS] test_kalman_gain_convergence (K: %.4f -> %.4f)\n",
           k_values[0], k_values[19]);
    return 0;
}

int test_noise_reduction() {
    KalmanFilter1D_t kf;
    kf1d_init(&kf, 0.001f, 4.0f, 25.0f, 1.0f);  // R=4: noise std~2

    float true_val = 25.0f;
    float sum_raw_error2 = 0.0f;
    float sum_kf_error2 = 0.0f;

    srand(42);
    for (int i = 0; i < 500; i++) {
        float noise = ((float)(rand() % 10000) / 5000.0f - 1.0f) * 2.0f;
        float measurement = true_val + noise;
        float estimate = kf1d_update(&kf, measurement);

        if (i >= 50) {  // Skip warmup
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

    printf("  [PASS] test_noise_reduction\n");
    return 0;
}

int test_2d_position_velocity() {
    KalmanFilter2D_t kf;
    kf2d_init(&kf, 0.1f, 1.0f, 1.0f, 0.1f);

    // Simulate constant velocity motion: pos = 2*t, vel = 2
    for (int i = 0; i < 100; i++) {
        float t = i * 0.1f;
        float true_pos = 2.0f * t;
        float noise = ((float)(rand() % 1000) / 500.0f - 1.0f);
        float measured_pos = true_pos + noise;

        kf2d_predict(&kf);
        kf2d_update(&kf, measured_pos);
    }

    // Velocity estimate should be close to 2.0 (with noise, allow wider tolerance)
    ASSERT_NEAR(kf.x[1], 2.0f, 1.0f, "Velocity estimate ~ 2.0");
    printf("  [PASS] test_2d_position_velocity (vel=%.2f)\n", kf.x[1]);
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
