#include <stdint.h>
#include "I2C.h"
#include "SysTick.h"
#include "PCA9685.h"
#include "writing_arm.h"

// Simple writing arm controller using PCA9685 at 50Hz
// 4 channels total: base, arm, pen (linear), eraser (linear)
#ifndef WRITING_ARM_PCA_ADDR
#define WRITING_ARM_PCA_ADDR PCA9685_I2C_ADDR_DEFAULT
#endif

#define SERVO_BASE_CH      0
#define SERVO_ARM_CH       1
#define SERVO_PEN_CH       2  // linear pen
#define SERVO_ERASER_CH    3  // linear eraser

// Per-servo min/max pulse width (microseconds). Tune as needed
static uint16_t s_min_us[4] = {500, 500, 600, 600};
static uint16_t s_max_us[4] = {2500, 2500, 2400, 2400};

static uint8_t s_addr = WRITING_ARM_PCA_ADDR;

static void delay_ms(uint32_t ms){
    while(ms--){
        SysTick_Wait(T1ms);
    }
}

void WritingArm_Init(uint8_t pca_addr){
    s_addr = pca_addr;
    SysTick_Init();
    I2C1_Init();
    PCA9685_Init(s_addr);
    PCA9685_SetPWMFreq(s_addr, 50);
}

void WritingArm_SetServoUs(uint8_t channel, uint16_t pulse_us){
    PCA9685_SetChannelPulseUs(s_addr, channel, pulse_us);
}

void WritingArm_SetServoAngle(uint8_t channel, int16_t angle_deg){
    uint8_t idx = (channel < 4) ? channel : 0;
    uint16_t min_us = s_min_us[idx];
    uint16_t max_us = s_max_us[idx];
    PCA9685_SetServoAngle(s_addr, channel, angle_deg, min_us, max_us);
}

void WritingArm_PenUp(void){
    // Usually smaller pulse raises the pen; adjust if inverted
    WritingArm_SetServoAngle(SERVO_PEN_CH, 0);
    delay_ms(300);
}

void WritingArm_PenDown(void){
    WritingArm_SetServoAngle(SERVO_PEN_CH, 90);
    delay_ms(300);
}

void WritingArm_EraserUp(void){
    WritingArm_SetServoAngle(SERVO_ERASER_CH, 0);
    delay_ms(300);
}

void WritingArm_EraserDown(void){
    WritingArm_SetServoAngle(SERVO_ERASER_CH, 90);
    delay_ms(300);
}

void WritingArm_Home(void){
    WritingArm_PenUp();
    WritingArm_EraserUp();
    WritingArm_SetServoAngle(SERVO_BASE_CH, 90);
    WritingArm_SetServoAngle(SERVO_ARM_CH, 90);
    delay_ms(500);
}

void WritingArm_MoveToAngles(int16_t base, int16_t arm){
    WritingArm_SetServoAngle(SERVO_BASE_CH, base);
    WritingArm_SetServoAngle(SERVO_ARM_CH, arm);
}

// Simple demonstration strokes for quick validation
void WritingArm_DemoStrokes(void){
    WritingArm_Home();

    // Draw a short stroke
    WritingArm_PenDown();
    WritingArm_MoveToAngles(90, 100);
    delay_ms(300);
    WritingArm_MoveToAngles(90, 110);
    delay_ms(300);
    WritingArm_PenUp();

    // Move right and tap
    WritingArm_MoveToAngles(110, 95);
    delay_ms(200);
    WritingArm_PenDown();
    delay_ms(200);
    WritingArm_PenUp();

    // Try eraser down/up
    WritingArm_MoveToAngles(80, 90);
    delay_ms(200);
    WritingArm_EraserDown();
    delay_ms(300);
    WritingArm_EraserUp();
}

// Optional quick self-test for all 4 channels
void WritingArm_ServoSweepTest(void){
    for(uint8_t ch = 0; ch < 4; ++ch){
        for(int a=0; a<=180; a+=30){
            PCA9685_SetServoAngle(s_addr, ch, a, s_min_us[ch], s_max_us[ch]);
            delay_ms(150);
        }
        for(int a=180; a>=0; a-=30){
            PCA9685_SetServoAngle(s_addr, ch, a, s_min_us[ch], s_max_us[ch]);
            delay_ms(150);
        }
    }
}
