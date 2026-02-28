// ===============================================
// tests/benchmark.c - Performance Benchmark
// ===============================================
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

    printf("  %-30s: %.2f us/call (%d iterations)\n", name, per_call, ITERATIONS);
    return per_call;
}

// Global objects (avoid including init in benchmark)
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
    printf("  Available time per loop: 1000 us\n");
    printf("  PID + KF1D total: %.2f us (%.1f%% budget)\n",
           t_pid + t_kf1d, (t_pid + t_kf1d) / 10.0f);
    printf("  PID + KF2D total: %.2f us (%.1f%% budget)\n",
           t_pid + t_kf2d, (t_pid + t_kf2d) / 10.0f);
    printf("  PID + EKF total:  %.2f us (%.1f%% budget)\n",
           t_pid + t_ekf, (t_pid + t_ekf) / 10.0f);

    return 0;
}
