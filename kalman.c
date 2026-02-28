// ===============================================
// kalman.c - Kalman Filter Implementation
// ===============================================
#include "kalman.h"

// ===== 1D Kalman Filter =====

void kf1d_init(KalmanFilter1D_t* kf, float q, float r, float x0, float p0) {
    kf->q = q;
    kf->r = r;
    kf->x_est = x0;
    kf->p_est = p0;
    kf->k = 0.0f;
}

float kf1d_update(KalmanFilter1D_t* kf, float z) {
    // Predict (F=1, B=0: assume state unchanged)
    float x_pred = kf->x_est;
    float p_pred = kf->p_est + kf->q;

    // Update
    kf->k = p_pred / (p_pred + kf->r);
    kf->x_est = x_pred + kf->k * (z - x_pred);
    kf->p_est = (1.0f - kf->k) * p_pred;

    return kf->x_est;
}

// ===== 2D Kalman Filter (position + velocity) =====

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
    // H = [1, 0] -> observe position only
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

    return kf->x[0]; // Return estimated position
}
