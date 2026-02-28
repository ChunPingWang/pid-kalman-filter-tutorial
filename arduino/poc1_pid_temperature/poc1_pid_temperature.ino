// ===============================================
// poc1_pid_temperature.ino - Arduino PID Temperature Control
// ===============================================
#include "../../pid.h"

// --- Hardware Config ---
#define THERMISTOR_PIN   A0
#define HEATER_PWM_PIN   9
#define SAMPLE_PERIOD_MS 100

// --- Thermistor Parameters (NTC 10K, B=3950) ---
#define SERIES_RESISTOR  10000.0f
#define NOMINAL_RESISTANCE 10000.0f
#define NOMINAL_TEMP     25.0f
#define B_COEFFICIENT    3950.0f

// --- PID Parameters ---
PID_t heater_pid;
float setpoint = 50.0f;  // Target temperature (C)

float read_temperature() {
    int raw = analogRead(THERMISTOR_PIN);
    float resistance = SERIES_RESISTOR / (1023.0f / raw - 1.0f);

    // Steinhart-Hart simplified B-parameter equation
    float steinhart = resistance / NOMINAL_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0f / (NOMINAL_TEMP + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;

    return steinhart;
}

void setup() {
    Serial.begin(115200);
    pinMode(HEATER_PWM_PIN, OUTPUT);

    float dt = SAMPLE_PERIOD_MS / 1000.0f;
    pid_init(&heater_pid, 10.0f, 0.5f, 2.0f, dt, 0.0f, 255.0f);

    Serial.println("time_ms,setpoint,temperature,pid_output");
}

void loop() {
    static uint32_t last_time = 0;
    uint32_t now = millis();

    if (now - last_time >= SAMPLE_PERIOD_MS) {
        last_time = now;

        float temperature = read_temperature();
        float output = pid_compute(&heater_pid, setpoint, temperature);

        analogWrite(HEATER_PWM_PIN, (int)output);

        // CSV output (for plotting)
        Serial.print(now);
        Serial.print(",");
        Serial.print(setpoint);
        Serial.print(",");
        Serial.print(temperature, 2);
        Serial.print(",");
        Serial.println(output, 2);
    }
}
