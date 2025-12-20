/**
 * PCA9685.c - PCA9685 PWM Servo Driver
 * 
 * Based on Adafruit's PCA9685 library approach:
 * https://github.com/grzesiek2201/Adafruit-Servo-Driver-Library-Pi-Pico
 * 
 * Key initialization sequence from Adafruit:
 *   MODE1 = 0xA0 (RESTART + AI)
 *   MODE2 = 0x04 (OUTDRV totem-pole)
 *   setPWMFreq(50) for servos
 */

#include <stdint.h>
#include "PCA9685.h"
#include "I2C.h"
#include "SysTick.h"
#include "UART0.h"

// =====================================================================
// PCA9685 Register Addresses
// =====================================================================
#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_SUBADR1       0x02
#define PCA9685_SUBADR2       0x03
#define PCA9685_SUBADR3       0x04
#define PCA9685_ALLCALLADR    0x05
#define PCA9685_LED0_ON_L     0x06
#define PCA9685_LED0_ON_H     0x07
#define PCA9685_LED0_OFF_L    0x08
#define PCA9685_LED0_OFF_H    0x09
#define PCA9685_ALLLED_ON_L   0xFA
#define PCA9685_ALLLED_ON_H   0xFB
#define PCA9685_ALLLED_OFF_L  0xFC
#define PCA9685_ALLLED_OFF_H  0xFD
#define PCA9685_PRESCALE      0xFE
#define PCA9685_TESTMODE      0xFF

// =====================================================================
// MODE1 Bits
// =====================================================================
#define MODE1_ALLCAL   0x01  // respond to LED All Call I2C-bus address
#define MODE1_SUB3     0x02  // respond to I2C-bus subaddress 3
#define MODE1_SUB2     0x04  // respond to I2C-bus subaddress 2
#define MODE1_SUB1     0x08  // respond to I2C-bus subaddress 1
#define MODE1_SLEEP    0x10  // Low power mode. Oscillator off
#define MODE1_AI       0x20  // Auto-Increment enabled
#define MODE1_EXTCLK   0x40  // Use EXTCLK pin clock
#define MODE1_RESTART  0x80  // Restart enabled

// =====================================================================
// MODE2 Bits
// =====================================================================
#define MODE2_OUTNE_0  0x01  // Active LOW output enable input
#define MODE2_OUTNE_1  0x02  // Active LOW output enable input - high impedance
#define MODE2_OUTDRV   0x04  // totem pole structure vs open-drain
#define MODE2_OCH      0x08  // Outputs change on ACK vs STOP
#define MODE2_INVRT    0x10  // Output logic state inverted

// =====================================================================
// Constants
// =====================================================================
#define FREQUENCY_OSCILLATOR  25000000UL  // Int. osc. frequency in datasheet
#define PCA9685_PRESCALE_MIN  3
#define PCA9685_PRESCALE_MAX  255

// =====================================================================
// Internal State
// =====================================================================
static uint32_t g_oscillator_freq = FREQUENCY_OSCILLATOR;
static uint16_t g_pca9685_freq_hz = 50;
static uint32_t g_last_i2c_status = 0;

// Forward declaration for recovery helper
static void PCA9685_Begin(uint8_t addr);
static void PCA9685_Wakeup(uint8_t addr);
static void pca9685_ensure_ai_awake(uint8_t addr);

// Check if MODE1 is awake; ignore AI to match Adafruit tolerance
static int pca9685_is_mode1_ok(uint8_t mode1){
    if(mode1 == 0xFF){
        return 0; // read error already captured in g_last_i2c_status
    }
    if((mode1 & MODE1_SLEEP) != 0){
        return 0; // asleep -> no PWM output
    }
    return 1;
}

// =====================================================================
// Low-level I2C helpers
// =====================================================================
static void writeRegister(uint8_t addr, uint8_t reg, uint8_t value){
    uint8_t err = I2C1_Transmit(addr, reg, value);
    if(err){
        g_last_i2c_status = err;
    }
}

static uint8_t readRegister(uint8_t addr, uint8_t reg){
    uint8_t val = I2C1_Receive(addr, reg);
    // I2C1_Receive returns 0xFF on error per implementation
    if(val == 0xFF){
        g_last_i2c_status = 0xFF;
    }
    return val;
}

// Delay helper
static void delay_ms(uint32_t ms){
    while(ms--){
        SysTick_Wait(T1ms);
    }
}

static void delay_us(uint32_t us){
    // T1ms = 16000 for 1ms at 16MHz, so T1us = 16
    uint32_t ticks = (us * 16);
    SysTick_Wait(ticks);
}

// Make sure MODE1 has AI set and is not sleeping before PWM writes
static void pca9685_ensure_ai_awake(uint8_t addr){
    uint8_t mode1 = readRegister(addr, PCA9685_MODE1);
    if(mode1 == 0xFF){
        return;
    }
    uint8_t desired = (uint8_t)((mode1 & ~MODE1_SLEEP) | MODE1_AI);
    if(desired != mode1){
        writeRegister(addr, PCA9685_MODE1, desired);
        delay_ms(1);
    }
}

// Attempt to recover the PCA9685 into a known-good MODE1/2 state
static int pca9685_recover(uint8_t addr){
    // Re-run the Adafruit begin sequence and verify MODE1
    PCA9685_Begin(addr);
    uint8_t mode1 = readRegister(addr, PCA9685_MODE1);
    if(pca9685_is_mode1_ok(mode1)){
        return 0;
    }
    return -1;
}

// Ensure MODE1 is sane; if not, keep trying to reconnect/re-init
static int pca9685_require_ready(uint8_t addr){
    uint8_t mode1 = readRegister(addr, PCA9685_MODE1);
    if(pca9685_is_mode1_ok(mode1)){
        g_last_i2c_status = 0;
        return 0;
    }

    // If asleep, wake it
    if(mode1 != 0xFF && (mode1 & MODE1_SLEEP)){
        PCA9685_Wakeup(addr);
    }

    // Single recovery attempt per call (Adafruit begin sequence)
    I2C1_Init();
    if(pca9685_recover(addr) == 0){
        pca9685_ensure_ai_awake(addr);
        g_last_i2c_status = 0;
        return 0;
    }

    g_last_i2c_status = 0xFF;
    return -1;
}

int PCA9685_EnsureReady(uint8_t addr){
    return pca9685_require_ready(addr);
}

// =====================================================================
// PCA9685_Reset - Software reset of PCA9685 
// =====================================================================
void PCA9685_Reset(uint8_t addr){
    writeRegister(addr, PCA9685_MODE1, MODE1_RESTART);
    delay_ms(10);
}

// =====================================================================
// PCA9685_Sleep - Put board into sleep mode
// =====================================================================
static void PCA9685_Sleep(uint8_t addr){
    uint8_t awake = readRegister(addr, PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP;  // set sleep bit high
    writeRegister(addr, PCA9685_MODE1, sleep);
    delay_ms(5);  // wait until cycle ends for sleep to be active
}

// =====================================================================
// PCA9685_Wakeup - Wake board from sleep
// =====================================================================
static void PCA9685_Wakeup(uint8_t addr){
    uint8_t sleep = readRegister(addr, PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP;  // set sleep bit low
    writeRegister(addr, PCA9685_MODE1, wakeup);
}

// =====================================================================
// PCA9685_Begin - Initialize PCA9685 
// This is the EXACT sequence from the Adafruit library
// =====================================================================
static void PCA9685_Begin(uint8_t addr){
    // Adafruit sequence: MODE2 OUTDRV, MODE1 ALLCALL, wake, then set freq
    writeRegister(addr, PCA9685_MODE2, MODE2_OUTDRV);
    writeRegister(addr, PCA9685_MODE1, MODE1_ALLCAL);
    delay_ms(5);

    // Clear sleep
    uint8_t mode1 = readRegister(addr, PCA9685_MODE1);
    mode1 &= ~MODE1_SLEEP;
    writeRegister(addr, PCA9685_MODE1, mode1);
    delay_ms(5);

    // Set the default frequency for servos (will also set AI|RESTART)
    PCA9685_SetPWMFreq(addr, 50);

    // Ensure AI stays set after frequency write
    pca9685_ensure_ai_awake(addr);
}

// =====================================================================
// PCA9685_Init - Full initialization
// =====================================================================
void PCA9685_Init(uint8_t addr){
    // Ensure time base and I2C bus are ready
    SysTick_Init();
    I2C1_Init();
    
    // Small delay for I2C bus to settle after init
    delay_ms(10);
    
    // Quick probe to detect device presence; capture raw status
    g_last_i2c_status = I2C1_ProbeStatus(addr);
    // If address not acknowledged, warn via UART for visibility
    if(g_last_i2c_status & 0x04){ // ADRACK bit set -> no device ack
        UART_OutString((uint8_t*)"PCA9685 not responding on I2C addr 0x");
        UART_OutUHex(addr);
        UART_OutString((uint8_t*)" (status=0x");
        UART_OutUHex((uint8_t)g_last_i2c_status);
        UART_OutString((uint8_t*)")\r\n");
        UART_OutString((uint8_t*)"Check: power, SDA/SCL wiring, pull-ups (4.7k)\r\n");
    } else {
        UART_OutString((uint8_t*)"PCA9685 detected at 0x");
        UART_OutUHex(addr);
        UART_OutString((uint8_t*)"\r\n");
    }
    
    // Use Adafruit begin sequence
    PCA9685_Begin(addr);
    
    // Verify by reading back MODE1
    uint8_t mode1 = readRegister(addr, PCA9685_MODE1);
    UART_OutString((uint8_t*)"PCA9685 MODE1=0x");
    UART_OutUHex(mode1);
    UART_OutString((uint8_t*)"\r\n");

    // Ensure chip awake before use
    (void)PCA9685_EnsureReady(addr);

    // Set a safe default PWM on first two channels to verify output (1.5ms pulse at 50Hz)
    uint32_t mid_counts = ((uint32_t)1500U * (uint32_t)g_pca9685_freq_hz * 4096UL + 500000UL) / 1000000UL;
    if(mid_counts > 4095UL) mid_counts = 3072U;
    PCA9685_SetPWM(addr, 0, 0, (uint16_t)mid_counts);
    PCA9685_SetPWM(addr, 1, 0, (uint16_t)mid_counts);
}

// =====================================================================
// PCA9685_GetCurrentFreqHz - Get current PWM frequency
// =====================================================================
uint16_t PCA9685_GetCurrentFreqHz(void){
    return g_pca9685_freq_hz;
}

// =====================================================================
// PCA9685_SetPWMFreq - Set PWM frequency 
// This is the EXACT algorithm from the Adafruit library
// =====================================================================
void PCA9685_SetPWMFreq(uint8_t addr, uint16_t freq_hz){
    float freq = (float)freq_hz;
    
    // Range output modulation frequency is dependent on oscillator
    if(freq < 1.0f) freq = 1.0f;
    if(freq > 3500.0f) freq = 3500.0f;  // Datasheet limit is 3052=50MHz/(4*4096)
    
    float prescaleval = (((float)g_oscillator_freq / (freq * 4096.0f)) + 0.5f) - 1.0f;
    if(prescaleval < (float)PCA9685_PRESCALE_MIN) prescaleval = (float)PCA9685_PRESCALE_MIN;
    if(prescaleval > (float)PCA9685_PRESCALE_MAX) prescaleval = (float)PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;
    
    // Adafruit sequence: sleep, set prescale, wake, restart+AI
    uint8_t oldmode = readRegister(addr, PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | MODE1_SLEEP;  // go to sleep
    writeRegister(addr, PCA9685_MODE1, newmode);
    writeRegister(addr, PCA9685_PRESCALE, prescale);
    writeRegister(addr, PCA9685_MODE1, oldmode);
    delay_ms(5);
    writeRegister(addr, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
    
    // Update internal tracking
    g_pca9685_freq_hz = freq_hz;
}

// =====================================================================
// PCA9685_SetPWM - Set PWM on/off values for a channel 
// Includes channel offset for staggered turn-on (reduces current spikes)
// =====================================================================
void PCA9685_SetPWM(uint8_t addr, uint8_t channel, uint16_t on_count, uint16_t off_count){
    if(channel > 15) return;
    // Ensure awake before write
    (void)pca9685_require_ready(addr);
    pca9685_ensure_ai_awake(addr);
    
    uint16_t channelOn = on_count & 0x0FFF;
    uint16_t channelOff = off_count & 0x0FFF;
    
    uint8_t buf[4];
    buf[0] = (uint8_t)(channelOn & 0xFF);
    buf[1] = (uint8_t)((channelOn >> 8) & 0x0F);
    buf[2] = (uint8_t)(channelOff & 0xFF);
    buf[3] = (uint8_t)((channelOff >> 8) & 0x0F);
    
    uint8_t reg = PCA9685_LED0_ON_L + (4 * channel);
    uint8_t err = I2C1_Burst_Transmit(addr, reg, buf, 4);
    if(err){
        g_last_i2c_status = err;
        UART_OutString((uint8_t*)"PCA9685 SetPWM I2C err=0x");
        UART_OutUHex(err);
        UART_OutString((uint8_t*)"\r\n");
    }
}

// =====================================================================
// PCA9685_SetAllPWM - Set PWM on all channels
// =====================================================================
void PCA9685_SetAllPWM(uint8_t addr, uint16_t on_count, uint16_t off_count){
    (void)pca9685_require_ready(addr);
    pca9685_ensure_ai_awake(addr);
    on_count &= 0x0FFF;
    off_count &= 0x0FFF;
    
    uint8_t buf[4];
    buf[0] = (uint8_t)(on_count & 0xFF);
    buf[1] = (uint8_t)((on_count >> 8) & 0x0F);
    buf[2] = (uint8_t)(off_count & 0xFF);
    buf[3] = (uint8_t)((off_count >> 8) & 0x0F);
    
    uint8_t err = I2C1_Burst_Transmit(addr, PCA9685_ALLLED_ON_L, buf, 4);
    if(err){
        g_last_i2c_status = err;
        UART_OutString((uint8_t*)"PCA9685 SetAllPWM I2C err=0x");
        UART_OutUHex(err);
        UART_OutString((uint8_t*)"\r\n");
    }
}

// =====================================================================
// PCA9685_SetChannelPulseUs - Set servo pulse width in microseconds
// =====================================================================
void PCA9685_SetChannelPulseUs(uint8_t addr, uint8_t channel, uint16_t pulse_us){
    // Convert microseconds to 12-bit count at current frequency
    // counts = pulse_us * freq_hz * 4096 / 1e6
    uint32_t counts = ((uint32_t)pulse_us * (uint32_t)g_pca9685_freq_hz * 4096UL + 500000UL) / 1000000UL;
    if(counts > 4095UL) counts = 4095UL;
    
    // on=0, off=counts (no channel offset here, SetPWM will add it)
    PCA9685_SetPWM(addr, channel, 0, (uint16_t)counts);
}

// =====================================================================
// PCA9685_SetServoAngle - Set servo angle (0-180 degrees)
// =====================================================================
void PCA9685_SetServoAngle(uint8_t addr, uint8_t channel, int16_t angle_deg, uint16_t min_us, uint16_t max_us){
    // Clamp angle
    if(angle_deg < 0) angle_deg = 0;
    if(angle_deg > 180) angle_deg = 180;
    
    // Clamp pulse ranges to sane values
    if(min_us < 400) min_us = 400;
    if(max_us > 3000) max_us = 3000;
    if(max_us <= min_us) max_us = min_us + 1;
    
    // Map 0..180 to min_us..max_us
    uint32_t span = (uint32_t)(max_us - min_us);
    uint32_t pulse = (uint32_t)min_us + ((uint32_t)angle_deg * span) / 180UL;
    
    PCA9685_SetChannelPulseUs(addr, channel, (uint16_t)pulse);
}

// =====================================================================
// Diagnostics helper
// =====================================================================
uint32_t PCA9685_GetLastI2CStatus(void){
    return g_last_i2c_status;
}
