// ===============================================
// tests/test_integration.c - Kalman + PID Integration Simulation
// ===============================================
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "../pid.h"
#include "../kalman.h"

/**
 * Simulate first-order lag system
 * G(s) = K / (tau*s + 1)
 *
 * Discrete: x[k+1] = x[k] + (K * u[k] - x[k]) / tau * dt
 */
typedef struct {
    float state;
    float K;      // System gain
    float tau;    // Time constant
} Plant_t;

float plant_step(Plant_t* p, float u, float dt) {
    p->state += (p->K * u - p->state) / p->tau * dt;
    return p->state;
}

int main() {
    float dt = 0.02f;
    int total_steps = 2000;  // 40 seconds

    // System
    Plant_t plant = { .state = 0.0f, .K = 1.0f, .tau = 2.0f };

    // PID
    PID_t pid;
    pid_init(&pid, 3.0f, 1.0f, 0.5f, dt, 0.0f, 100.0f);

    // Kalman Filter
    KalmanFilter1D_t kf;
    kf1d_init(&kf, 0.01f, 4.0f, 0.0f, 10.0f);

    // Also test PID without Kalman
    PID_t pid_no_kf;
    pid_init(&pid_no_kf, 3.0f, 1.0f, 0.5f, dt, 0.0f, 100.0f);
    Plant_t plant_no_kf = { .state = 0.0f, .K = 1.0f, .tau = 2.0f };

    float setpoint = 50.0f;

    printf("time,setpoint,true_kf,noisy,filtered,pid_kf_out,true_no_kf,pid_no_kf_out\n");

    srand(123);
    for (int i = 0; i < total_steps; i++) {
        float t = i * dt;

        // Change setpoint at t=20s
        if (t >= 20.0f) setpoint = 30.0f;

        // --- Plan A: Kalman + PID ---
        float noise = ((float)(rand() % 10000) / 5000.0f - 1.0f) * 3.0f;
        float measurement = plant.state + noise;
        float filtered = kf1d_update(&kf, measurement);
        float pid_out = pid_compute(&pid, setpoint, filtered);
        plant_step(&plant, pid_out, dt);

        // --- Plan B: PID only (no Kalman) ---
        float noise2 = ((float)(rand() % 10000) / 5000.0f - 1.0f) * 3.0f;
        float meas_no_kf = plant_no_kf.state + noise2;
        float pid_no_kf_out = pid_compute(&pid_no_kf, setpoint, meas_no_kf);
        plant_step(&plant_no_kf, pid_no_kf_out, dt);

        printf("%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               t, setpoint, plant.state, measurement, filtered,
               pid_out, plant_no_kf.state, pid_no_kf_out);
    }

    // Performance summary
    fprintf(stderr, "\n--- Performance Summary ---\n");
    fprintf(stderr, "Compare the output smoothness and tracking accuracy\n");
    fprintf(stderr, "in the generated CSV file.\n");

    return 0;
}
