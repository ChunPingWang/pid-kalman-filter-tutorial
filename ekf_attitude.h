// ===============================================
// ekf_attitude.h - Extended Kalman Filter for Attitude Estimation
// ===============================================
#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

#include <math.h>

typedef struct {
    // State: [roll, pitch, gyro_bias_x, gyro_bias_y]
    float x[4];

    // Covariance 4x4
    float P[4][4];

    // Process noise
    float Q_angle;     // Angle process noise
    float Q_bias;      // Gyroscope bias process noise

    // Measurement noise
    float R_accel;     // Accelerometer measurement noise

    float dt;
} EKF_Attitude_t;

void ekf_attitude_init(EKF_Attitude_t* ekf, float dt,
                        float q_angle, float q_bias, float r_accel);

/**
 * EKF predict step (using gyroscope)
 * @param gx, gy  Gyroscope angular velocity (rad/s)
 */
void ekf_attitude_predict(EKF_Attitude_t* ekf, float gx, float gy);

/**
 * EKF update step (using accelerometer)
 * @param ax, ay, az  Accelerometer values (g)
 */
void ekf_attitude_update(EKF_Attitude_t* ekf, float ax, float ay, float az);

/** Get estimated angles (degrees) */
float ekf_get_roll_deg(EKF_Attitude_t* ekf);
float ekf_get_pitch_deg(EKF_Attitude_t* ekf);

#endif
