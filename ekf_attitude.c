// ===============================================
// ekf_attitude.c - EKF Implementation
// ===============================================
#include "ekf_attitude.h"

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f

void ekf_attitude_init(EKF_Attitude_t* ekf, float dt,
                        float q_angle, float q_bias, float r_accel) {
    ekf->dt = dt;
    ekf->Q_angle = q_angle;
    ekf->Q_bias = q_bias;
    ekf->R_accel = r_accel;

    // Initial state = 0
    for (int i = 0; i < 4; i++) {
        ekf->x[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void ekf_attitude_predict(EKF_Attitude_t* ekf, float gx, float gy) {
    float dt = ekf->dt;

    // Remove gyroscope bias
    float gx_corrected = gx - ekf->x[2];
    float gy_corrected = gy - ekf->x[3];

    // State prediction: angle += gyro_corrected * dt
    ekf->x[0] += gx_corrected * dt;  // roll
    ekf->x[1] += gy_corrected * dt;  // pitch
    // bias stays unchanged (random walk model)

    // Jacobian F = I + [[0,0,-dt,0],[0,0,0,-dt],[0,0,0,0],[0,0,0,0]]
    // P = F*P*F^T + Q (simplified computation)
    ekf->P[0][0] += dt * (-ekf->P[2][0] - ekf->P[0][2] + dt * ekf->P[2][2]) + ekf->Q_angle;
    ekf->P[0][1] += dt * (-ekf->P[2][1] - ekf->P[0][3] + dt * ekf->P[2][3]);
    ekf->P[1][0] += dt * (-ekf->P[3][0] - ekf->P[1][2] + dt * ekf->P[3][2]);
    ekf->P[1][1] += dt * (-ekf->P[3][1] - ekf->P[1][3] + dt * ekf->P[3][3]) + ekf->Q_angle;

    ekf->P[2][2] += ekf->Q_bias;
    ekf->P[3][3] += ekf->Q_bias;
}

void ekf_attitude_update(EKF_Attitude_t* ekf, float ax, float ay, float az) {
    // Compute observed angles from accelerometer
    float accel_roll  = atan2(ay, az);
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Innovation (measurement - prediction)
    float y0 = accel_roll  - ekf->x[0];
    float y1 = accel_pitch - ekf->x[1];

    // H = [[1,0,0,0],[0,1,0,0]] -> S = H*P*H^T + R
    float S00 = ekf->P[0][0] + ekf->R_accel;
    float S11 = ekf->P[1][1] + ekf->R_accel;

    // Kalman Gain K = P * H^T * S^(-1) (simple due to H structure)
    float K00 = ekf->P[0][0] / S00;
    float K10 = ekf->P[1][0] / S00;
    float K20 = ekf->P[2][0] / S00;
    float K30 = ekf->P[3][0] / S00;

    float K01 = ekf->P[0][1] / S11;
    float K11 = ekf->P[1][1] / S11;
    float K21 = ekf->P[2][1] / S11;
    float K31 = ekf->P[3][1] / S11;

    // State update: x = x + K * y
    ekf->x[0] += K00 * y0 + K01 * y1;
    ekf->x[1] += K10 * y0 + K11 * y1;
    ekf->x[2] += K20 * y0 + K21 * y1;
    ekf->x[3] += K30 * y0 + K31 * y1;

    // Covariance update: P = (I - K*H) * P
    float P00_new = (1.0f - K00) * ekf->P[0][0] - K01 * ekf->P[1][0];
    float P11_new = -K10 * ekf->P[0][1] + (1.0f - K11) * ekf->P[1][1];

    ekf->P[0][0] = P00_new;
    ekf->P[1][1] = P11_new;
    ekf->P[2][0] -= K20 * ekf->P[0][0];
    ekf->P[3][1] -= K31 * ekf->P[1][1];
}

float ekf_get_roll_deg(EKF_Attitude_t* ekf) {
    return ekf->x[0] * RAD_TO_DEG;
}

float ekf_get_pitch_deg(EKF_Attitude_t* ekf) {
    return ekf->x[1] * RAD_TO_DEG;
}
