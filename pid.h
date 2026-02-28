// ===============================================
// pid.h - PID Controller with Anti-Windup
// ===============================================
#ifndef PID_H
#define PID_H

typedef struct {
    // Parameters
    float kp;
    float ki;
    float kd;
    float dt;              // Sampling period (seconds)

    // Output limits
    float out_min;
    float out_max;

    // Integral limits (anti-windup)
    float integral_min;
    float integral_max;

    // Internal state
    float integral;
    float prev_measurement; // derivative on measurement
    float prev_error;
    int   initialized;
} PID_t;

/**
 * Initialize PID controller
 */
void pid_init(PID_t* pid, float kp, float ki, float kd, float dt,
              float out_min, float out_max);

/**
 * Compute PID output
 * @param pid         PID struct pointer
 * @param setpoint    Target value
 * @param measurement Actual measured value (or Kalman estimate)
 * @return            Control output (clamped to out_min ~ out_max)
 */
float pid_compute(PID_t* pid, float setpoint, float measurement);

/**
 * Reset PID internal state
 */
void pid_reset(PID_t* pid);

#endif // PID_H
