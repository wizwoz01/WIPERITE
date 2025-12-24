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

// IMPORTANT: these are *servo angles* (0..180).
// Tune using your UART stepping commands in main.c:
//   W/S = BASE_CH (0), A/D = ARM_CH (1), J/K = PEN_CH (2)

#define BASE_LEFT_ANGLE     60
#define BASE_RIGHT_ANGLE    120
#define ARM_TOP_ANGLE       85
#define ARM_BOTTOM_ANGLE    135

#define PEN_UP_ANGLE        180
#define PEN_DOWN_ANGLE      50

// Flip these if letters are mirrored/upside-down:
#define BASE_INVERT         0
#define ARM_INVERT          0


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

// void WritingArm_PenUp(void){
//     WritingArm_SetServoAngle(SERVO_PEN_CH, 180);
//     delay_ms(300);
// }

// void WritingArm_PenDown(void){
//     WritingArm_SetServoAngle(SERVO_PEN_CH, 0);
//     delay_ms(300);
// }
void WritingArm_PenUp(void){
    WritingArm_SetServoAngle(SERVO_PEN_CH, PEN_UP_ANGLE);
    delay_ms(250);
}

void WritingArm_PenDown(void){
    WritingArm_SetServoAngle(SERVO_PEN_CH, PEN_DOWN_ANGLE);
    delay_ms(250);
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

// --- Basic 5x5 font drawing implementation ---------------------------------
// Each character is 5 columns wide, 5 rows high. Columns are L-to-R, bits
// low-to-high represent top-to-bottom rows (bit0 = top row).

static const uint8_t font5x5[26][5] = {
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

#define FONT_W 5
#define FONT_H 5
#define GRID_COLS_PER_CHAR (FONT_W + 1)
#define GRID_ROWS_PER_LINE (FONT_H + 2)

// allow longer text / multiple lines
#define GRID_MAX_X 90
#define GRID_MAX_Y 35

#define GRID_BASE_MIN BASE_LEFT_ANGLE
#define GRID_BASE_MAX BASE_RIGHT_ANGLE
#define GRID_ARM_MIN  ARM_TOP_ANGLE
#define GRID_ARM_MAX  ARM_BOTTOM_ANGLE

static int lerp_int(int a, int b, int t, int tmax){
    if(t < 0) t = 0;
    if(t > tmax) t = tmax;
    return a + (t * (b - a)) / tmax;
}

static void moveToGrid(int gx, int gy){
    if(gx < 0) gx = 0;
    if(gy < 0) gy = 0;
    if(gx > GRID_MAX_X) gx = GRID_MAX_X;
    if(gy > GRID_MAX_Y) gy = GRID_MAX_Y;

    int base = lerp_int(GRID_BASE_MIN, GRID_BASE_MAX, gx, GRID_MAX_X);
    int arm  = lerp_int(GRID_ARM_MIN,  GRID_ARM_MAX,  gy, GRID_MAX_Y);

    if(BASE_INVERT) base = GRID_BASE_MIN + GRID_BASE_MAX - base;
    if(ARM_INVERT)  arm  = GRID_ARM_MIN  + GRID_ARM_MAX  - arm;

    WritingArm_MoveToAngles(base, arm);
    delay_ms(120);
}

static void WritingArm_DrawCharAt(char c, int x0, int y0){
    WritingArm_PenUp();

    if(c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    if(c < 'A' || c > 'Z'){
        // space / unsupported char
        moveToGrid(x0 + GRID_COLS_PER_CHAR, y0);
        return;
    }

    const uint8_t *rows = font5x5[c - 'A'];

    // Draw horizontal “runs” (cleaner than tapping dots)
    for(int r = 0; r < FONT_H; r++){
        uint8_t bits = rows[r];
        int col = 0;

        while(col < FONT_W){
            while(col < FONT_W && (bits & (1U << (FONT_W-1-col))) == 0) col++;
            if(col >= FONT_W) break;

            int start = col;
            while(col < FONT_W && (bits & (1U << (FONT_W-1-col))) != 0) col++;
            int end = col - 1;

            moveToGrid(x0 + start, y0 + r);
            WritingArm_PenDown();
            moveToGrid(x0 + end,   y0 + r);
            WritingArm_PenUp();
        }
    }
}

void WritingArm_DrawChar(char c, int x_origin){
    WritingArm_DrawCharAt(c, x_origin, 0);
    moveToGrid(x_origin + GRID_COLS_PER_CHAR, 0);
}

void WritingArm_DrawString(const char *s, int x_origin){
    int x = x_origin;
    int y = 0;

    while(s && *s){
        char c = *s++;

        if(c == '\n' || c == '\r'){
            x = x_origin;
            y += GRID_ROWS_PER_LINE;
            continue;
        }

        if(x + GRID_COLS_PER_CHAR > GRID_MAX_X){
            x = x_origin;
            y += GRID_ROWS_PER_LINE;
        }
        if(y + FONT_H > GRID_MAX_Y) break;

        WritingArm_DrawCharAt(c, x, y);
        x += GRID_COLS_PER_CHAR;
    }
}
// --- End of font drawing implementation ------------------------------------