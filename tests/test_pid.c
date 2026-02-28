// ===============================================
// tests/test_pid.c - PID Unit Tests
// ===============================================
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
    ASSERT_NEAR(out, 10.0f, 0.01f, "P-only: error=10 -> output=10");

    out = pid_compute(&pid, 10.0f, 10.0f);
    ASSERT_NEAR(out, 0.0f, 0.01f, "P-only: error=0 -> output=0");

    printf("  [PASS] test_p_only\n");
    return 0;
}

int test_pi_steady_state() {
    PID_t pid;
    pid_init(&pid, 1.0f, 10.0f, 0.0f, 0.01f, -100.0f, 100.0f);

    // With constant error=5, integral should accumulate
    float out1 = pid_compute(&pid, 50.0f, 45.0f);
    float out2 = pid_compute(&pid, 50.0f, 45.0f);
    float out3 = pid_compute(&pid, 50.0f, 45.0f);

    assert(out3 > out2 && out2 > out1);
    printf("  [PASS] test_pi_steady_state (integral accumulates)\n");
    return 0;
}

int test_anti_windup() {
    PID_t pid;
    pid_init(&pid, 1.0f, 100.0f, 0.0f, 0.1f, -10.0f, 10.0f);

    // Saturate for a long time
    for (int i = 0; i < 1000; i++) {
        pid_compute(&pid, 100.0f, 0.0f);
    }

    // Output should be clamped
    float out = pid_compute(&pid, 100.0f, 0.0f);
    ASSERT_NEAR(out, 10.0f, 0.01f, "Anti-windup: output clamped at max");

    // Suddenly reach setpoint, should not have excessive windup
    out = pid_compute(&pid, 0.0f, 0.0f);
    assert(out >= -10.0f);
    printf("  [PASS] test_anti_windup\n");
    return 0;
}

int test_output_clamping() {
    PID_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.1f, 0.0f, 255.0f);

    float out = pid_compute(&pid, 1000.0f, 0.0f);
    ASSERT_NEAR(out, 255.0f, 0.01f, "Output clamped at 255");

    out = pid_compute(&pid, -1000.0f, 0.0f);
    ASSERT_NEAR(out, 0.0f, 0.01f, "Output clamped at 0");

    printf("  [PASS] test_output_clamping\n");
    return 0;
}

int test_derivative_on_measurement() {
    PID_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.1f, -100.0f, 100.0f);

    // First call (initialized=0) -> D term should be 0
    float out1 = pid_compute(&pid, 10.0f, 5.0f);
    ASSERT_NEAR(out1, 0.0f, 0.01f, "D first call = 0");

    // Second call, measurement from 5 to 8
    // D = -Kd * (8 - 5) / 0.1 = -1 * 30 = -30
    float out2 = pid_compute(&pid, 10.0f, 8.0f);
    ASSERT_NEAR(out2, -30.0f, 0.01f, "D on measurement");

    printf("  [PASS] test_derivative_on_measurement\n");
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
