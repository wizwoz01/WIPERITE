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
    WritingArm_SetServoAngle(SERVO_PEN_CH, 90);
    delay_ms(300);
}

void WritingArm_PenDown(void){
    WritingArm_SetServoAngle(SERVO_PEN_CH, 0);
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
    WritingArm_SetServoAngle(SERVO_ARM_CH, 110);
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

// --- Basic 5x7 font drawing implementation ---------------------------------
// Each character is 5 columns wide, 7 rows high. Columns are L-to-R, bits
// low-to-high represent top-to-bottom rows (bit0 = top row).

static const uint8_t font5x7[26][5] = {
    {0x0E,0x11,0x1F,0x11,0x11}, // A
    {0x1E,0x11,0x1E,0x11,0x1E}, // B
    {0x0E,0x11,0x10,0x11,0x0E}, // C
    {0x1E,0x11,0x11,0x11,0x1E}, // D
    {0x1F,0x10,0x1E,0x10,0x1F}, // E
    {0x1F,0x10,0x1E,0x10,0x10}, // F
    {0x0E,0x10,0x17,0x11,0x0E}, // G
    {0x11,0x11,0x1F,0x11,0x11}, // H
    {0x0E,0x04,0x04,0x04,0x0E}, // I
    {0x07,0x02,0x02,0x12,0x0C}, // J
    {0x11,0x12,0x1C,0x12,0x11}, // K
    {0x10,0x10,0x10,0x10,0x1F}, // L
    {0x11,0x1B,0x15,0x11,0x11}, // M
    {0x11,0x19,0x15,0x13,0x11}, // N
    {0x0E,0x11,0x11,0x11,0x0E}, // O
    {0x1E,0x11,0x1E,0x10,0x10}, // P
    {0x0E,0x11,0x11,0x0E,0x03}, // Q
    {0x1E,0x11,0x1E,0x12,0x11}, // R
    {0x0F,0x10,0x0E,0x01,0x1E}, // S
    {0x1F,0x04,0x04,0x04,0x04}, // T
    {0x11,0x11,0x11,0x11,0x0E}, // U
    {0x11,0x11,0x0A,0x0A,0x04}, // V
    {0x11,0x11,0x15,0x1B,0x11}, // W
    {0x11,0x0A,0x04,0x0A,0x11}, // X
    {0x11,0x0A,0x04,0x04,0x04}, // Y
    {0x1F,0x02,0x04,0x08,0x1F}  // Z
};

// Grid mapping: X columns and Y rows
#define GRID_COLS_PER_CHAR 6 // 5 cols + 1 spacing
#define GRID_MAX_X 60
#define GRID_MAX_Y 6

// Tune these ranges to fit your workspace; they map grid coordinates to servo
// angles. base: left/right, arm: up/down.
#define GRID_BASE_MIN 60
#define GRID_BASE_MAX 120
#define GRID_ARM_MIN 80
#define GRID_ARM_MAX 120

static void moveToGrid(int gx, int gy){
    if(gx < 0) gx = 0;
    if(gy < 0) gy = 0;
    if(gy > GRID_MAX_Y) gy = GRID_MAX_Y;

    // clamp gx to a reasonable span
    if(gx > GRID_MAX_X) gx = GRID_MAX_X;

    int base = GRID_BASE_MIN + (gx * (GRID_BASE_MAX - GRID_BASE_MIN)) / GRID_MAX_X;
    int arm  = GRID_ARM_MIN + (gy * (GRID_ARM_MAX - GRID_ARM_MIN)) / GRID_MAX_Y;
    WritingArm_MoveToAngles(base, arm);
    delay_ms(80);
}

void WritingArm_DrawChar(char c, int x_origin){
    if(c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    if(c < 'A' || c > 'Z'){
        // treat as space: advance pen without drawing
        // move baseline to end of char width
        moveToGrid(x_origin + GRID_COLS_PER_CHAR - 1, GRID_MAX_Y/2);
        return;
    }

    const uint8_t *glyph = font5x7[c - 'A'];

    // For each column in glyph
    for(int col = 0; col < 5; ++col){
        uint8_t colbits = glyph[col];
        int gx = x_origin + col;
        // For each row (0 = top)
        for(int row = 0; row <= GRID_MAX_Y; ++row){
            if(colbits & (1 << row)){
                // move above the dot, then pen down, tap, pen up
                moveToGrid(gx, row);
                WritingArm_PenDown();
                delay_ms(80);
                WritingArm_PenUp();
                delay_ms(30);
            }
        }
    }

    // small gap after char
    moveToGrid(x_origin + GRID_COLS_PER_CHAR, GRID_MAX_Y/2);
}

void WritingArm_DrawString(const char *s, int x_origin){
    int x = x_origin;
    while(s && *s){
        WritingArm_DrawChar(*s, x);
        x += GRID_COLS_PER_CHAR;
        s++;
    }
}

