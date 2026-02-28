// ===============================================
// poc4_ekf_imu.ino - EKF Attitude Estimation with MPU6050
// ===============================================
#include <Wire.h>
#include "../../ekf_attitude.h"

#define MPU6050_ADDR  0x68
#define SAMPLE_PERIOD_MS 10   // 100 Hz

EKF_Attitude_t ekf;

// MPU6050 raw data
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

void mpu6050_init() {
    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);  // Wake up
    Wire.endTransmission();

    // Set accelerometer +/-2g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();

    // Set gyroscope +/-250 deg/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void mpu6050_read() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

    ax_raw = Wire.read() << 8 | Wire.read();
    ay_raw = Wire.read() << 8 | Wire.read();
    az_raw = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // temperature (skip)
    gx_raw = Wire.read() << 8 | Wire.read();
    gy_raw = Wire.read() << 8 | Wire.read();
    gz_raw = Wire.read() << 8 | Wire.read();
}

void setup() {
    Serial.begin(115200);
    mpu6050_init();

    float dt = SAMPLE_PERIOD_MS / 1000.0f;
    ekf_attitude_init(&ekf, dt, 0.001f, 0.003f, 0.03f);

    Serial.println("time_ms,accel_roll,accel_pitch,ekf_roll,ekf_pitch");
}

void loop() {
    static uint32_t last_time = 0;
    uint32_t now = millis();

    if (now - last_time >= SAMPLE_PERIOD_MS) {
        last_time = now;

        mpu6050_read();

        // Convert to physical units
        float ax = ax_raw / 16384.0f;  // g (+/-2g range)
        float ay = ay_raw / 16384.0f;
        float az = az_raw / 16384.0f;
        float gx = gx_raw / 131.0f * 0.01745329f;  // rad/s (+/-250 deg/s)
        float gy = gy_raw / 131.0f * 0.01745329f;

        // EKF
        ekf_attitude_predict(&ekf, gx, gy);
        ekf_attitude_update(&ekf, ax, ay, az);

        // Accelerometer direct calculation (for comparison)
        float accel_roll  = atan2(ay, az) * 57.2957795f;
        float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2957795f;

        Serial.print(now); Serial.print(",");
        Serial.print(accel_roll, 2); Serial.print(",");
        Serial.print(accel_pitch, 2); Serial.print(",");
        Serial.print(ekf_get_roll_deg(&ekf), 2); Serial.print(",");
        Serial.println(ekf_get_pitch_deg(&ekf), 2);
    }
}
