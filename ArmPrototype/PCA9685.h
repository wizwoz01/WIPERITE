#ifndef PCA9685_H_
#define PCA9685_H_

#include <stdint.h>

// Default I2C address
#ifndef PCA9685_I2C_ADDRESS
#define PCA9685_I2C_ADDRESS 0x40
#endif

#ifndef PCA9685_I2C_ADDR_DEFAULT
#define PCA9685_I2C_ADDR_DEFAULT PCA9685_I2C_ADDRESS
#endif

// Registers
#define PCA9685_MODE1        0x00
#define PCA9685_MODE2        0x01
#define PCA9685_SUBADR1      0x02
#define PCA9685_SUBADR2      0x03
#define PCA9685_SUBADR3      0x04
#define PCA9685_ALLCALLADR   0x05
#define PCA9685_LED0_ON_L    0x06
#define PCA9685_LED0_ON_H    0x07
#define PCA9685_LED0_OFF_L   0x08
#define PCA9685_LED0_OFF_H   0x09
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRESCALE     0xFE

// MODE1 bits
#define MODE1_ALLCALL  0x01
#define MODE1_SUB3     0x02
#define MODE1_SUB2     0x04
#define MODE1_SUB1     0x08
#define MODE1_SLEEP    0x10
#define MODE1_AI       0x20
#define MODE1_EXTCLK   0x40
#define MODE1_RESTART  0x80

// MODE2 bits
#define MODE2_OUTNE_0  0x01
#define MODE2_OUTNE_1  0x02
#define MODE2_OUTDRV   0x04
#define MODE2_OCH      0x08
#define MODE2_INVRT    0x10

#define FREQUENCY_OSCILLATOR 25000000UL

// Public API (Adafruit-like C port)
void PCA9685_Init(uint8_t addr); // initialize
void PCA9685_Reset(uint8_t addr);
void PCA9685_sleep(uint8_t addr);
void PCA9685_wakeup(uint8_t addr);
void PCA9685_setExtClk(uint8_t addr, uint8_t prescale);
void PCA9685_setOutputMode(uint8_t addr, uint8_t totempole);
uint8_t PCA9685_readPrescale(uint8_t addr);
void PCA9685_SetPWMFreq(uint8_t addr, float freq);
uint16_t PCA9685_getPWM(uint8_t addr, uint8_t num, uint8_t off);
uint8_t PCA9685_SetPWM(uint8_t addr, uint8_t num, uint16_t on, uint16_t off);
void PCA9685_SetAllPWM(uint8_t addr, uint16_t on_count, uint16_t off_count);
void PCA9685_SetPin(uint8_t addr, uint8_t num, uint16_t val, uint8_t invert);
uint8_t PCA9685_read8(uint8_t addr, uint8_t reg);
void PCA9685_write8(uint8_t addr, uint8_t reg, uint8_t data);
void PCA9685_writeMicroseconds(uint8_t addr, uint8_t num, uint16_t Microseconds);

// Servo helpers (compatibility)
void PCA9685_SetChannelPulseUs(uint8_t addr, uint8_t channel, uint16_t pulse_us);
void PCA9685_SetServoAngle(uint8_t addr, uint8_t channel, int16_t angle_deg, uint16_t min_us, uint16_t max_us);

// Oscillator helpers
uint32_t PCA9685_getOscillatorFrequency(void);
void PCA9685_setOscillatorFrequency(uint32_t freq);

// Current freq getter
uint16_t PCA9685_GetCurrentFreqHz(void);

#endif // PCA9685_H_
