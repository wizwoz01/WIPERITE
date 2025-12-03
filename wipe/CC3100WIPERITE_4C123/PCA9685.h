#ifndef PCA9685_H_
#define PCA9685_H_

#include <stdint.h>

// PCA9685 default 7-bit I2C address (A5..A0 = 0)
#ifndef PCA9685_I2C_ADDR_DEFAULT
#define PCA9685_I2C_ADDR_DEFAULT 0x40
#endif

// Public API
void PCA9685_Init(uint8_t addr);
void PCA9685_Reset(uint8_t addr);
void PCA9685_SetPWMFreq(uint8_t addr, uint16_t freq_hz);
void PCA9685_SetPWM(uint8_t addr, uint8_t channel, uint16_t on_count, uint16_t off_count);
void PCA9685_SetAllPWM(uint8_t addr, uint16_t on_count, uint16_t off_count);

// Servo helpers (typical RC servo)
void PCA9685_SetChannelPulseUs(uint8_t addr, uint8_t channel, uint16_t pulse_us);
void PCA9685_SetServoAngle(uint8_t addr, uint8_t channel, int16_t angle_deg, uint16_t min_us, uint16_t max_us);

// Optional: get/keep current frequency used by helpers
uint16_t PCA9685_GetCurrentFreqHz(void);

#endif // PCA9685_H_
