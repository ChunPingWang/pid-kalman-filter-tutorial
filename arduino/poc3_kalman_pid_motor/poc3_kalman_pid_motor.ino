// ===============================================
// poc3_kalman_pid_motor.ino - Kalman + PID Motor Speed Control
// ===============================================
#include "../../pid.h"
#include "../../kalman.h"

// --- Hardware Pins ---
#define MOTOR_PWM_PIN    6
#define MOTOR_DIR_PIN    7
#define ENCODER_A_PIN    2   // Interrupt pin
#define ENCODER_B_PIN    3

// --- Parameters ---
#define SAMPLE_PERIOD_MS 20
#define ENCODER_PPR      600   // Pulses Per Revolution
#define GEAR_RATIO       1.0f

// --- Global Variables ---
volatile long encoder_count = 0;
long prev_encoder_count = 0;

PID_t speed_pid;
KalmanFilter2D_t speed_kf;     // State: [speed, acceleration]

float target_rpm = 120.0f;

// --- Encoder ISR ---
void encoder_isr() {
    if (digitalRead(ENCODER_B_PIN) == HIGH) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

// --- Calculate RPM ---
float calculate_raw_rpm(float dt_sec) {
    long current_count = encoder_count;
    long delta = current_count - prev_encoder_count;
    prev_encoder_count = current_count;

    float rps = (float)delta / (ENCODER_PPR * GEAR_RATIO) / dt_sec;
    return rps * 60.0f;  // RPM
}

void setup() {
    Serial.begin(115200);

    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_isr, RISING);

    float dt = SAMPLE_PERIOD_MS / 1000.0f;

    // PID init: output is PWM (0~255)
    pid_init(&speed_pid, 1.0f, 0.5f, 0.05f, dt, 0.0f, 255.0f);

    // 2D Kalman Filter: estimate [speed, acceleration]
    kf2d_init(&speed_kf, 0.1f, 1.0f, 10.0f, dt);

    Serial.println("time_ms,target_rpm,raw_rpm,filtered_rpm,pid_output");
}

void loop() {
    static uint32_t last_time = 0;
    uint32_t now = millis();

    if (now - last_time >= SAMPLE_PERIOD_MS) {
        float dt_actual = (now - last_time) / 1000.0f;
        last_time = now;

        // 1. Read raw RPM
        float raw_rpm = calculate_raw_rpm(dt_actual);

        // 2. Kalman Filter: Predict + Update
        kf2d_predict(&speed_kf);
        float filtered_rpm = kf2d_update(&speed_kf, raw_rpm);

        // 3. PID: use filtered RPM
        float pid_output = pid_compute(&speed_pid, target_rpm, filtered_rpm);

        // 4. Drive motor
        digitalWrite(MOTOR_DIR_PIN, (pid_output >= 0) ? HIGH : LOW);
        analogWrite(MOTOR_PWM_PIN, (int)fabs(pid_output));

        // 5. Log
        Serial.print(now); Serial.print(",");
        Serial.print(target_rpm, 1); Serial.print(",");
        Serial.print(raw_rpm, 1); Serial.print(",");
        Serial.print(filtered_rpm, 1); Serial.print(",");
        Serial.println(pid_output, 1);
    }
}
