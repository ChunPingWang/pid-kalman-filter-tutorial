// ===============================================
// hal.h - Hardware Abstraction Layer
// ===============================================
#ifndef HAL_H
#define HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Time ---
uint32_t hal_millis(void);          // Millisecond timestamp
void     hal_delay_ms(uint32_t ms); // Millisecond delay

// --- ADC ---
void     hal_adc_init(uint8_t channel);
uint16_t hal_adc_read(uint8_t channel);  // Return raw value
float    hal_adc_to_voltage(uint16_t raw, float vref, uint8_t bits);

// --- PWM ---
void     hal_pwm_init(uint8_t channel, uint32_t freq_hz);
void     hal_pwm_set_duty(uint8_t channel, float duty_percent); // 0~100

// --- GPIO ---
void     hal_gpio_init(uint8_t pin, uint8_t mode); // 0=INPUT, 1=OUTPUT
void     hal_gpio_write(uint8_t pin, uint8_t value);
uint8_t  hal_gpio_read(uint8_t pin);

// --- Serial/UART ---
void     hal_serial_init(uint32_t baud);
void     hal_serial_print(const char* str);
void     hal_serial_printf(const char* fmt, ...);

// --- I2C (for IMU sensors) ---
void     hal_i2c_init(uint32_t speed);
int      hal_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);
int      hal_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);

#ifdef __cplusplus
}
#endif
#endif // HAL_H
