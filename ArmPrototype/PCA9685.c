#include <stdint.h>
#include <string.h>
#include "PCA9685.h"
#include "I2C.h"
#include "SysTick.h"

// Internal state
static volatile uint16_t g_pca9685_freq_hz = 50;
static uint32_t _oscillator_freq = FREQUENCY_OSCILLATOR;

// Low-level I2C wrappers
uint8_t PCA9685_read8(uint8_t addr, uint8_t reg){
    return I2C1_Receive(addr, reg);
}

void PCA9685_write8(uint8_t addr, uint8_t reg, uint8_t data){
    (void)I2C1_Transmit(addr, reg, data);
}

static void write4(uint8_t addr, uint8_t start, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3){
    // Use single-byte writes to avoid potential issues with burst trasmit implementation
    (void)I2C1_Transmit(addr, start + 0, b0);
    (void)I2C1_Transmit(addr, start + 1, b1);
    (void)I2C1_Transmit(addr, start + 2, b2);
    (void)I2C1_Transmit(addr, start + 3, b3);
}

void PCA9685_Reset(uint8_t addr){
    // Adafruit does write MODE1 = RESTART then delay
    PCA9685_write8(addr, PCA9685_MODE1, MODE1_RESTART);
    SysTick_Wait(10 * T1ms / 1);
}

void PCA9685_Init(uint8_t addr){
    PCA9685_Reset(addr);
    // Configure MODE2 to totem pole and MODE1 to enable auto-increment and all-call
    PCA9685_write8(addr, PCA9685_MODE2, MODE2_OUTDRV);
    PCA9685_write8(addr, PCA9685_MODE1, MODE1_AI | MODE1_ALLCALL);
    PCA9685_setOscillatorFrequency(_oscillator_freq);
    PCA9685_SetPWMFreq(addr, 50.0f);
}

void PCA9685_sleep(uint8_t addr){
    uint8_t awake = PCA9685_read8(addr, PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP;
    PCA9685_write8(addr, PCA9685_MODE1, sleep);
    SysTick_Wait(5 * T1ms / 1);
}

void PCA9685_wakeup(uint8_t addr){
    uint8_t sleep = PCA9685_read8(addr, PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP;
    PCA9685_write8(addr, PCA9685_MODE1, wakeup);
}

void PCA9685_setExtClk(uint8_t addr, uint8_t prescale){
    uint8_t oldmode = PCA9685_read8(addr, PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    PCA9685_write8(addr, PCA9685_MODE1, newmode);
    // set EXTCLK and prescale
    newmode |= MODE1_EXTCLK;
    PCA9685_write8(addr, PCA9685_MODE1, newmode);
    PCA9685_write8(addr, PCA9685_PRESCALE, prescale);
    SysTick_Wait(5 * T1ms / 1);
    PCA9685_write8(addr, PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

void PCA9685_setOutputMode(uint8_t addr, uint8_t totempole){
    uint8_t oldmode = PCA9685_read8(addr, PCA9685_MODE2);
    uint8_t newmode;
    if(totempole) newmode = oldmode | MODE2_OUTDRV;
    else newmode = oldmode & ~MODE2_OUTDRV;
    PCA9685_write8(addr, PCA9685_MODE2, newmode);
}

uint8_t PCA9685_readPrescale(uint8_t addr){
    return PCA9685_read8(addr, PCA9685_PRESCALE);
}

void PCA9685_SetPWMFreq(uint8_t addr, float freq){
    if(freq < 1.0f) freq = 1.0f;
    if(freq > 3500.0f) freq = 3500.0f;

    float prescaleval = ((_oscillator_freq / (freq * 4096.0f)) + 0.5f) - 1.0f;
    if(prescaleval < 3.0f) prescaleval = 3.0f;
    if(prescaleval > 255.0f) prescaleval = 255.0f;
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = PCA9685_read8(addr, PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    PCA9685_write8(addr, PCA9685_MODE1, newmode);
    PCA9685_write8(addr, PCA9685_PRESCALE, prescale);
    PCA9685_write8(addr, PCA9685_MODE1, oldmode);
    SysTick_Wait(5 * T1ms / 1);
    PCA9685_write8(addr, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
    g_pca9685_freq_hz = (uint16_t)freq;
}

uint16_t PCA9685_getPWM(uint8_t addr, uint8_t num, uint8_t off){
    // Read two bytes from LEDn_OFF_L/LEDn_OFF_H if off==1 else LEDn_ON_L/H
    uint8_t reg = PCA9685_LED0_ON_L + 4 * num;
    if(off) reg += 2;
    // Write register then read two bytes
    uint8_t hi = PCA9685_read8(addr, reg + 1);
    uint8_t lo = PCA9685_read8(addr, reg);
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

uint8_t PCA9685_SetPWM(uint8_t addr, uint8_t num, uint16_t on, uint16_t off){
    uint8_t buffer[4];
    on &= 0x0FFF; off &= 0x0FFF;
    buffer[0] = (uint8_t)(on & 0xFF);
    buffer[1] = (uint8_t)((on >> 8) & 0x0F);
    buffer[2] = (uint8_t)(off & 0xFF);
    buffer[3] = (uint8_t)((off >> 8) & 0x0F);
    write4(addr, PCA9685_LED0_ON_L + 4 * num, buffer[0], buffer[1], buffer[2], buffer[3]);
    return 0;
}

void PCA9685_SetAllPWM(uint8_t addr, uint16_t on_count, uint16_t off_count){
    on_count &= 0x0FFF; off_count &= 0x0FFF;
    write4(addr, PCA9685_ALL_LED_ON_L,
            (uint8_t)(on_count & 0xFF), (uint8_t)((on_count >> 8) & 0x0F),
            (uint8_t)(off_count & 0xFF), (uint8_t)((off_count >> 8) & 0x0F));
}

void PCA9685_SetPin(uint8_t addr, uint8_t num, uint16_t val, uint8_t invert){
    // Clamp to 0..4095
    if(val > 4095) val = 4095;
    if(invert){
        if(val == 0){
            PCA9685_SetPWM(addr, num, 4096, 0);
        } else if(val == 4095){
            PCA9685_SetPWM(addr, num, 0, 4096);
        } else {
            PCA9685_SetPWM(addr, num, 0, (uint16_t)(4095 - val));
        }
    } else {
        if(val == 4095){
            PCA9685_SetPWM(addr, num, 4096, 0);
        } else if(val == 0){
            PCA9685_SetPWM(addr, num, 0, 4096);
        } else {
            PCA9685_SetPWM(addr, num, 0, val);
        }
    }
}

void PCA9685_writeMicroseconds(uint8_t addr, uint8_t num, uint16_t Microseconds){
    uint8_t prescale = PCA9685_readPrescale(addr);
    float pulselength = 1000000.0f; // 1,000,000 us per second
    pulselength *= (float)(prescale + 1);
    pulselength /= (float)_oscillator_freq;
    float pulse = (float)Microseconds / pulselength;
    if(pulse < 0.0f) pulse = 0.0f;
    if(pulse > 4095.0f) pulse = 4095.0f;
    PCA9685_SetPWM(addr, num, 0, (uint16_t)(pulse + 0.5f));
}

void PCA9685_setOscillatorFrequency(uint32_t freq){
    _oscillator_freq = freq;
}

uint32_t PCA9685_getOscillatorFrequency(void){
    return _oscillator_freq;
}

void PCA9685_SetChannelPulseUs(uint8_t addr, uint8_t channel, uint16_t pulse_us){
    PCA9685_writeMicroseconds(addr, channel, pulse_us);
}

void PCA9685_SetServoAngle(uint8_t addr, uint8_t channel, int16_t angle_deg, uint16_t min_us, uint16_t max_us){
    if(angle_deg < 0) angle_deg = 0;
    if(angle_deg > 180) angle_deg = 180;
    if(min_us < 400) min_us = 400;
    if(max_us > 3000) max_us = 3000;
    if(max_us <= min_us) max_us = min_us + 1;
    uint32_t span = (uint32_t)(max_us - min_us);
    uint32_t pulse = (uint32_t)min_us + ((uint32_t)angle_deg * span) / 180U;
    PCA9685_SetChannelPulseUs(addr, channel, (uint16_t)pulse);
}

uint16_t PCA9685_GetCurrentFreqHz(void){
    return g_pca9685_freq_hz;
}