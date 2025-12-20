#include <stdint.h>
#include "PCA9685.h"
#include "I2C.h"
#include "SysTick.h"

// PCA9685 Register map
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
#define MODE1_RESTART  0x80
#define MODE1_EXTCLK   0x40
#define MODE1_AI       0x20
#define MODE1_SLEEP    0x10
#define MODE1_SUB1     0x08
#define MODE1_SUB2     0x04
#define MODE1_SUB3     0x02
#define MODE1_ALLCALL  0x01

// MODE2 bits
#define MODE2_OUTDRV   0x04
#define MODE2_INVRT    0x10

// Internal state for helper conversions
static volatile uint16_t g_pca9685_freq_hz = 50; // default for servos

// Internal helpers
static inline uint8_t rd8(uint8_t addr, uint8_t reg){
    return I2C1_Receive(addr, reg);
}

static inline void wr8(uint8_t addr, uint8_t reg, uint8_t val){
    (void)I2C1_Transmit(addr, reg, val);
}

static inline void wr4(uint8_t addr, uint8_t start_reg, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3){
    uint8_t buf[4] = {b0, b1, b2, b3};
    (void)I2C1_Burst_Transmit(addr, start_reg, buf, 4);
}

void PCA9685_Reset(uint8_t addr){
    // Software reset is via All-Call or general call (not used here). Minimal reset:
    // Clear MODE1 to defaults with ALLCALL enabled, MODE2 to totem pole
    wr8(addr, PCA9685_MODE1, MODE1_AI | MODE1_ALLCALL); // auto-increment on
    wr8(addr, PCA9685_MODE2, MODE2_OUTDRV);             // totem-pole, non-inverted
    // Wait a bit for oscillator to stabilize
    SysTick_Wait(T1ms); // ~1ms
}

void PCA9685_Init(uint8_t addr){
    // Ensure I2C and SysTick are set up by caller
    PCA9685_Reset(addr);
    // Set a sane default frequency for servos
    PCA9685_SetPWMFreq(addr, 50);
}

uint16_t PCA9685_GetCurrentFreqHz(void){
    return g_pca9685_freq_hz;
}

void PCA9685_SetPWMFreq(uint8_t addr, uint16_t freq_hz){
    if(freq_hz < 24) freq_hz = 24;        // per datasheet min ~24Hz
    if(freq_hz > 1526) freq_hz = 1526;    // max per datasheet

    // Internal oscillator frequency (Hz)
    const uint32_t osc_hz = 25000000UL; // 25 MHz default
    // prescale = round(osc/(4096*freq)) - 1
    uint32_t den = (uint32_t)4096 * (uint32_t)freq_hz;
    uint32_t prescale = (osc_hz + (den/2)) / den; // rounded
    if(prescale == 0) prescale = 1;
    if(prescale > 256) prescale = 256;
    prescale = (prescale > 0) ? (prescale - 1) : 0;

    uint8_t oldmode = rd8(addr, PCA9685_MODE1);
    uint8_t sleepmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // go to sleep to set prescale
    wr8(addr, PCA9685_MODE1, sleepmode);
    wr8(addr, PCA9685_PRESCALE, (uint8_t)prescale);
    wr8(addr, PCA9685_MODE1, oldmode); // restore
    SysTick_Wait(T1ms); // wait >500us for oscillator
    wr8(addr, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);

    g_pca9685_freq_hz = freq_hz;
}

void PCA9685_SetPWM(uint8_t addr, uint8_t channel, uint16_t on_count, uint16_t off_count){
    if(channel > 15) return;
    on_count  &= 0x0FFF;
    off_count &= 0x0FFF;
    uint8_t reg = (uint8_t)(PCA9685_LED0_ON_L + 4 * channel);
    wr4(addr, reg,
        (uint8_t)(on_count & 0xFF),
        (uint8_t)((on_count >> 8) & 0x0F),
        (uint8_t)(off_count & 0xFF),
        (uint8_t)((off_count >> 8) & 0x0F));
}

void PCA9685_SetAllPWM(uint8_t addr, uint16_t on_count, uint16_t off_count){
    on_count  &= 0x0FFF;
    off_count &= 0x0FFF;
    wr4(addr, PCA9685_ALL_LED_ON_L,
        (uint8_t)(on_count & 0xFF),
        (uint8_t)((on_count >> 8) & 0x0F),
        (uint8_t)(off_count & 0xFF),
        (uint8_t)((off_count >> 8) & 0x0F));
}

void PCA9685_SetChannelPulseUs(uint8_t addr, uint8_t channel, uint16_t pulse_us){
    // Convert microseconds to 12-bit count at current frequency
    // counts = pulse_us * freq_hz * 4096 / 1e6
    uint32_t counts = ((uint32_t)pulse_us * (uint32_t)g_pca9685_freq_hz * 4096U + 500000U) / 1000000U;
    if(counts > 4095U) counts = 4095U;
    PCA9685_SetPWM(addr, channel, 0, (uint16_t)counts);
}

void PCA9685_SetServoAngle(uint8_t addr, uint8_t channel, int16_t angle_deg, uint16_t min_us, uint16_t max_us){
    if(angle_deg < 0) angle_deg = 0;
    if(angle_deg > 180) angle_deg = 180;
    if(min_us < 400) min_us = 400;   // clamp sane range
    if(max_us > 3000) max_us = 3000;
    if(max_us <= min_us) max_us = min_us + 1;

    // Map 0..180 to min_us..max_us
    uint32_t span = (uint32_t)(max_us - min_us);
    uint32_t pulse = (uint32_t)min_us + ((uint32_t)angle_deg * span) / 180U;
    PCA9685_SetChannelPulseUs(addr, channel, (uint16_t)pulse);
}
