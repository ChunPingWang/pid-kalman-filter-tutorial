// ===============================================
// pid.c - PID Controller Implementation
// ===============================================
#include "pid.h"

static float clamp(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

void pid_init(PID_t* pid, float kp, float ki, float kd, float dt,
              float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = out_min * 0.8f;  // Default integral limit = 80% of output limit
    pid->integral_max = out_max * 0.8f;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error = 0.0f;
    pid->initialized = 0;
}

float pid_compute(PID_t* pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    // --- P ---
    float p_term = pid->kp * error;

    // --- I with anti-windup (clamping) ---
    pid->integral += error * pid->dt;
    pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);
    float i_term = pid->ki * pid->integral;

    // --- D on measurement (avoid derivative kick) ---
    float d_term = 0.0f;
    if (pid->initialized) {
        float d_measurement = (measurement - pid->prev_measurement) / pid->dt;
        d_term = -pid->kd * d_measurement;
    }
    pid->prev_measurement = measurement;
    pid->prev_error = error;
    pid->initialized = 1;

    // --- Combined output ---
    float output = p_term + i_term + d_term;
    return clamp(output, pid->out_min, pid->out_max);
}

void pid_reset(PID_t* pid) {
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error = 0.0f;
    pid->initialized = 0;
}
