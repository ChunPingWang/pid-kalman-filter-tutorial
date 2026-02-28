// ===============================================
// kalman.h - Kalman Filter (Configurable Dimension)
// ===============================================
#ifndef KALMAN_H
#define KALMAN_H

// --- 1D Kalman Filter ---
typedef struct {
    float x_est;   // Estimated state
    float p_est;   // Estimated error covariance
    float q;       // Process noise covariance
    float r;       // Measurement noise covariance
    float k;       // Kalman gain (readable for diagnostics)
} KalmanFilter1D_t;

/**
 * Initialize 1D Kalman Filter
 * @param kf     Kalman Filter struct
 * @param q      Process noise variance (larger = trust measurement more)
 * @param r      Measurement noise variance (larger = trust model more)
 * @param x0     Initial state estimate
 * @param p0     Initial error covariance
 */
void kf1d_init(KalmanFilter1D_t* kf, float q, float r, float x0, float p0);

/**
 * Kalman Filter update (1D, F=1, H=1, B=0)
 * @param kf     Kalman Filter
 * @param z      Measurement
 * @return       Filtered estimate
 */
float kf1d_update(KalmanFilter1D_t* kf, float z);

// --- 2D Kalman Filter (position + velocity) ---
typedef struct {
    float x[2];       // State [position, velocity]
    float P[2][2];    // Covariance matrix
    float Q[2][2];    // Process noise
    float R;          // Measurement noise (observe position only)
    float dt;         // Sampling period
} KalmanFilter2D_t;

void kf2d_init(KalmanFilter2D_t* kf, float q_pos, float q_vel,
               float r, float dt);
void kf2d_predict(KalmanFilter2D_t* kf);
float kf2d_update(KalmanFilter2D_t* kf, float z_position);

#endif // KALMAN_H
