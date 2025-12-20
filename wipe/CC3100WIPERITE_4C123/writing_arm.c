#include <stdint.h>
#include <string.h>
#include "I2C.h"
#include "SysTick.h"
#include "PCA9685.h"
#include "writing_arm.h"

// Forward declare SimpleLink cooperative task so long delays keep WiFi alive
extern void _SlNonOsMainLoopTask(void);

// =====================================================================
// Writing Arm Controller for WIPERITE
// 4 channels: base rotation, arm extension, pen actuator, eraser actuator
// =====================================================================

#ifndef WRITING_ARM_PCA_ADDR
#define WRITING_ARM_PCA_ADDR PCA9685_I2C_ADDR_DEFAULT
#endif

// Channel assignments
#define SERVO_BASE_CH      WARM_CH_BASE
#define SERVO_ARM_CH       WARM_CH_ARM
#define SERVO_PEN_CH       WARM_CH_PEN
#define SERVO_ERASER_CH    WARM_CH_ERASER

// Number of servo channels
#define NUM_SERVOS 4

// Per-servo min/max pulse width (microseconds)
// These define the physical limits of each servo
static uint16_t s_min_us[NUM_SERVOS] = {500, 500, 600, 600};
static uint16_t s_max_us[NUM_SERVOS] = {2500, 2500, 2400, 2400};

// Pen/Eraser up/down angles (adjust based on your linear actuator behavior)
static int16_t s_pen_up_angle = 0;
static int16_t s_pen_down_angle = 60;
static int16_t s_eraser_up_angle = 0;
static int16_t s_eraser_down_angle = 60;

// Home position for base and arm
static int16_t s_home_base = 90;
static int16_t s_home_arm = 90;

// Device address
static uint8_t s_addr = WRITING_ARM_PCA_ADDR;

// Current state tracking
static WritingArmState_t s_state = {90, 90, 0, 0, TOOL_NONE};

// =====================================================================
// Internal Helpers
// =====================================================================
// Cooperative delay that services WiFi to prevent timeout and disconnection
static void delay_ms(uint32_t ms){
    while(ms--){
        SysTick_Wait(T1ms);
        _SlNonOsMainLoopTask();  // Keep WiFi alive during delays
    }
}

// Absolute value helper
static int16_t abs16(int16_t x){
    return (x < 0) ? -x : x;
}

// Clamp value to range
static int16_t clamp16(int16_t val, int16_t min_v, int16_t max_v){
    if(val < min_v) return min_v;
    if(val > max_v) return max_v;
    return val;
}

// =====================================================================
// Core Initialization
// =====================================================================
void WritingArm_Init(uint8_t pca_addr){
    s_addr = pca_addr;
    SysTick_Init();
    I2C1_Init();
    
    // Initialize PCA9685 with robust sequence
    PCA9685_Init(s_addr);
    PCA9685_SetPWMFreq(s_addr, 50);  // 50Hz for servos
    
    // Small delay for PCA9685 to stabilize
    delay_ms(100);
    
    // Initialize state
    s_state.base_angle = s_home_base;
    s_state.arm_angle = s_home_arm;
    s_state.pen_down = 0;
    s_state.eraser_down = 0;
    s_state.active_tool = TOOL_NONE;
    
    // Move to home position
    WritingArm_Home();
}

void WritingArm_SetServoLimits(uint8_t channel, uint16_t min_us, uint16_t max_us){
    if(channel < NUM_SERVOS){
        s_min_us[channel] = min_us;
        s_max_us[channel] = max_us;
    }
}

WritingArmState_t WritingArm_GetState(void){
    return s_state;
}

// =====================================================================
// Low-Level Servo Control
// =====================================================================
void WritingArm_SetServoUs(uint8_t channel, uint16_t pulse_us){
    PCA9685_SetChannelPulseUs(s_addr, channel, pulse_us);
}

void WritingArm_SetServoAngle(uint8_t channel, int16_t angle_deg){
    uint8_t idx = (channel < NUM_SERVOS) ? channel : 0;
    uint16_t min_us = s_min_us[idx];
    uint16_t max_us = s_max_us[idx];
    
    // Clamp angle to valid range
    angle_deg = clamp16(angle_deg, 0, 180);
    
    PCA9685_SetServoAngle(s_addr, channel, angle_deg, min_us, max_us);
    
    // Update state tracking
    if(channel == SERVO_BASE_CH){
        s_state.base_angle = angle_deg;
    } else if(channel == SERVO_ARM_CH){
        s_state.arm_angle = angle_deg;
    }
}

// =====================================================================
// Tool Control (Pen & Eraser)
// =====================================================================
void WritingArm_PenUp(void){
    WritingArm_SetServoAngle(SERVO_PEN_CH, s_pen_up_angle);
    s_state.pen_down = 0;
    delay_ms(200);  // Allow time for actuator to move
}

void WritingArm_PenDown(void){
    WritingArm_SetServoAngle(SERVO_PEN_CH, s_pen_down_angle);
    s_state.pen_down = 1;
    delay_ms(200);
}

void WritingArm_EraserUp(void){
    WritingArm_SetServoAngle(SERVO_ERASER_CH, s_eraser_up_angle);
    s_state.eraser_down = 0;
    delay_ms(200);
}

void WritingArm_EraserDown(void){
    WritingArm_SetServoAngle(SERVO_ERASER_CH, s_eraser_down_angle);
    s_state.eraser_down = 1;
    delay_ms(200);
}

void WritingArm_AllToolsUp(void){
    WritingArm_PenUp();
    WritingArm_EraserUp();
}

void WritingArm_SelectTool(WritingTool_t tool){
    // First raise all tools
    WritingArm_AllToolsUp();
    s_state.active_tool = tool;
}

// Lower the currently selected tool
static void tool_down(void){
    switch(s_state.active_tool){
        case TOOL_PEN:
            WritingArm_PenDown();
            break;
        case TOOL_ERASER:
            WritingArm_EraserDown();
            break;
        default:
            break;
    }
}

// Raise the currently selected tool
static void tool_up(void){
    switch(s_state.active_tool){
        case TOOL_PEN:
            WritingArm_PenUp();
            break;
        case TOOL_ERASER:
            WritingArm_EraserUp();
            break;
        default:
            break;
    }
}

// =====================================================================
// Movement Commands
// =====================================================================
void WritingArm_Home(void){
    WritingArm_AllToolsUp();
    WritingArm_SetServoAngle(SERVO_BASE_CH, s_home_base);
    WritingArm_SetServoAngle(SERVO_ARM_CH, s_home_arm);
    delay_ms(500);
}

void WritingArm_MoveToAngles(int16_t base, int16_t arm){
    base = clamp16(base, 0, 180);
    arm = clamp16(arm, 0, 180);
    WritingArm_SetServoAngle(SERVO_BASE_CH, base);
    WritingArm_SetServoAngle(SERVO_ARM_CH, arm);
}

void WritingArm_MoveToAnglesSlow(int16_t base, int16_t arm, uint16_t step_delay_ms){
    base = clamp16(base, 0, 180);
    arm = clamp16(arm, 0, 180);
    
    int16_t curr_base = s_state.base_angle;
    int16_t curr_arm = s_state.arm_angle;
    
    int16_t db = base - curr_base;
    int16_t da = arm - curr_arm;
    
    // Calculate number of steps based on larger delta
    int16_t steps = abs16(db);
    if(abs16(da) > steps) steps = abs16(da);
    if(steps == 0) return;
    
    // Move in increments
    for(int16_t i = 1; i <= steps; i++){
        int16_t new_base = curr_base + (db * i) / steps;
        int16_t new_arm = curr_arm + (da * i) / steps;
        WritingArm_SetServoAngle(SERVO_BASE_CH, new_base);
        WritingArm_SetServoAngle(SERVO_ARM_CH, new_arm);
        delay_ms(step_delay_ms);
    }
}

void WritingArm_MoveTo(ArmPoint_t pt){
    WritingArm_MoveToAngles(pt.base, pt.arm);
}

void WritingArm_MoveToSlow(ArmPoint_t pt, uint16_t step_delay_ms){
    WritingArm_MoveToAnglesSlow(pt.base, pt.arm, step_delay_ms);
}

// =====================================================================
// Drawing Primitives
// =====================================================================
void WritingArm_DrawLine(ArmPoint_t from, ArmPoint_t to, uint16_t step_delay_ms){
    // Move to start with tool up
    tool_up();
    WritingArm_MoveToSlow(from, step_delay_ms);
    delay_ms(100);
    
    // Lower tool
    tool_down();
    delay_ms(100);
    
    // Draw line to end point
    WritingArm_MoveToSlow(to, step_delay_ms);
    
    // Raise tool
    tool_up();
}

void WritingArm_DrawArc(ArmPoint_t center, int16_t radius_deg, int16_t start_angle, 
                        int16_t end_angle, uint16_t step_delay_ms){
    // Simple arc approximation using line segments
    // This is in "angle space" not Cartesian - adjust for your kinematics
    
    int16_t num_segments = abs16(end_angle - start_angle) / 10;
    if(num_segments < 1) num_segments = 1;
    if(num_segments > 36) num_segments = 36;
    
    // Calculate first point and move there
    // Simple approximation: base varies with angle, arm = center arm + radius
    ArmPoint_t first;
    first.base = center.base + (radius_deg * start_angle) / 90;
    first.arm = center.arm + radius_deg;
    
    tool_up();
    WritingArm_MoveTo(first);
    delay_ms(100);
    tool_down();
    
    int16_t angle_step = (end_angle - start_angle) / num_segments;
    for(int16_t i = 1; i <= num_segments; i++){
        int16_t a = start_angle + angle_step * i;
        ArmPoint_t pt;
        pt.base = center.base + (radius_deg * a) / 90;
        pt.arm = center.arm + radius_deg;
        WritingArm_MoveToSlow(pt, step_delay_ms);
    }
    
    tool_up();
}

void WritingArm_Dot(ArmPoint_t pt, uint16_t dwell_ms){
    tool_up();
    WritingArm_MoveTo(pt);
    delay_ms(100);
    tool_down();
    delay_ms(dwell_ms);
    tool_up();
}

// =====================================================================
// Letter Writing - Stroke-based letter definitions
// Letters are defined as sequences of strokes in normalized coordinates
// Origin is bottom-left, size scales the letter
// =====================================================================

// Helper to draw a stroke (line segment) relative to an origin with scaling
static void draw_stroke(ArmPoint_t origin, int16_t size,
                        int16_t x1, int16_t y1, int16_t x2, int16_t y2,
                        uint16_t step_delay_ms){
    ArmPoint_t from, to;
    // Map normalized coords (0-10) to arm angles relative to origin
    // x maps to base angle change, y maps to arm angle change
    from.base = origin.base + (x1 * size) / 10;
    from.arm = origin.arm + (y1 * size) / 10;
    to.base = origin.base + (x2 * size) / 10;
    to.arm = origin.arm + (y2 * size) / 10;
    
    WritingArm_DrawLine(from, to, step_delay_ms);
}

// Write letter A
static void write_A(ArmPoint_t origin, int16_t size){
    // Left diagonal: (0,0) -> (5,10)
    draw_stroke(origin, size, 0, 0, 5, 10, 20);
    // Right diagonal: (5,10) -> (10,0)
    draw_stroke(origin, size, 5, 10, 10, 0, 20);
    // Horizontal bar: (2,5) -> (8,5)
    draw_stroke(origin, size, 2, 5, 8, 5, 20);
}

// Write letter B
static void write_B(ArmPoint_t origin, int16_t size){
    // Vertical: (0,0) -> (0,10)
    draw_stroke(origin, size, 0, 0, 0, 10, 20);
    // Top horizontal: (0,10) -> (7,10)
    draw_stroke(origin, size, 0, 10, 7, 10, 20);
    // Top curve approximation: (7,10) -> (7,6)
    draw_stroke(origin, size, 7, 10, 7, 6, 20);
    // Middle horizontal: (0,5) -> (7,5)
    draw_stroke(origin, size, 0, 5, 7, 5, 20);
    // Bottom curve: (7,5) -> (7,0)
    draw_stroke(origin, size, 7, 5, 7, 0, 20);
    // Bottom horizontal: (0,0) -> (7,0)
    draw_stroke(origin, size, 0, 0, 7, 0, 20);
}

// Write letter C
static void write_C(ArmPoint_t origin, int16_t size){
    // Top horizontal: (10,10) -> (2,10)
    draw_stroke(origin, size, 10, 10, 2, 10, 20);
    // Left vertical: (2,10) -> (2,0)
    draw_stroke(origin, size, 2, 10, 2, 0, 20);
    // Bottom horizontal: (2,0) -> (10,0)
    draw_stroke(origin, size, 2, 0, 10, 0, 20);
}

// Write letter E
static void write_E(ArmPoint_t origin, int16_t size){
    // Vertical: (0,0) -> (0,10)
    draw_stroke(origin, size, 0, 0, 0, 10, 20);
    // Top horizontal: (0,10) -> (8,10)
    draw_stroke(origin, size, 0, 10, 8, 10, 20);
    // Middle horizontal: (0,5) -> (6,5)
    draw_stroke(origin, size, 0, 5, 6, 5, 20);
    // Bottom horizontal: (0,0) -> (8,0)
    draw_stroke(origin, size, 0, 0, 8, 0, 20);
}

// Write letter H
static void write_H(ArmPoint_t origin, int16_t size){
    // Left vertical: (0,0) -> (0,10)
    draw_stroke(origin, size, 0, 0, 0, 10, 20);
    // Right vertical: (8,0) -> (8,10)
    draw_stroke(origin, size, 8, 0, 8, 10, 20);
    // Middle horizontal: (0,5) -> (8,5)
    draw_stroke(origin, size, 0, 5, 8, 5, 20);
}

// Write letter I
static void write_I(ArmPoint_t origin, int16_t size){
    // Vertical: (5,0) -> (5,10)
    draw_stroke(origin, size, 5, 0, 5, 10, 20);
    // Top serif: (2,10) -> (8,10)
    draw_stroke(origin, size, 2, 10, 8, 10, 20);
    // Bottom serif: (2,0) -> (8,0)
    draw_stroke(origin, size, 2, 0, 8, 0, 20);
}

// Write letter L
static void write_L(ArmPoint_t origin, int16_t size){
    // Vertical: (0,10) -> (0,0)
    draw_stroke(origin, size, 0, 10, 0, 0, 20);
    // Bottom horizontal: (0,0) -> (8,0)
    draw_stroke(origin, size, 0, 0, 8, 0, 20);
}

// Write letter O
static void write_O(ArmPoint_t origin, int16_t size){
    // Approximate as rectangle
    // Left: (0,0) -> (0,10)
    draw_stroke(origin, size, 0, 0, 0, 10, 20);
    // Top: (0,10) -> (8,10)
    draw_stroke(origin, size, 0, 10, 8, 10, 20);
    // Right: (8,10) -> (8,0)
    draw_stroke(origin, size, 8, 10, 8, 0, 20);
    // Bottom: (8,0) -> (0,0)
    draw_stroke(origin, size, 8, 0, 0, 0, 20);
}

// Write letter P
static void write_P(ArmPoint_t origin, int16_t size){
    // Vertical: (0,0) -> (0,10)
    draw_stroke(origin, size, 0, 0, 0, 10, 20);
    // Top horizontal: (0,10) -> (7,10)
    draw_stroke(origin, size, 0, 10, 7, 10, 20);
    // Right vertical top: (7,10) -> (7,5)
    draw_stroke(origin, size, 7, 10, 7, 5, 20);
    // Middle horizontal: (7,5) -> (0,5)
    draw_stroke(origin, size, 7, 5, 0, 5, 20);
}

// Write letter R
static void write_R(ArmPoint_t origin, int16_t size){
    // Same as P plus diagonal leg
    write_P(origin, size);
    // Diagonal leg: (4,5) -> (8,0)
    draw_stroke(origin, size, 4, 5, 8, 0, 20);
}

// Write letter T
static void write_T(ArmPoint_t origin, int16_t size){
    // Top horizontal: (0,10) -> (10,10)
    draw_stroke(origin, size, 0, 10, 10, 10, 20);
    // Vertical: (5,10) -> (5,0)
    draw_stroke(origin, size, 5, 10, 5, 0, 20);
}

// Write letter W
static void write_W(ArmPoint_t origin, int16_t size){
    // Left down: (0,10) -> (2,0)
    draw_stroke(origin, size, 0, 10, 2, 0, 20);
    // Left up: (2,0) -> (5,6)
    draw_stroke(origin, size, 2, 0, 5, 6, 20);
    // Right down: (5,6) -> (8,0)
    draw_stroke(origin, size, 5, 6, 8, 0, 20);
    // Right up: (8,0) -> (10,10)
    draw_stroke(origin, size, 8, 0, 10, 10, 20);
}

// Write number 1
static void write_1(ArmPoint_t origin, int16_t size){
    // Vertical: (5,0) -> (5,10)
    draw_stroke(origin, size, 5, 0, 5, 10, 20);
    // Top serif: (3,8) -> (5,10)
    draw_stroke(origin, size, 3, 8, 5, 10, 20);
}

// Write number 0
static void write_0(ArmPoint_t origin, int16_t size){
    write_O(origin, size);  // Same as letter O
}

void WritingArm_WriteChar(char c, ArmPoint_t origin, int16_t size){
    // Convert to uppercase
    if(c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    
    switch(c){
        case 'A': write_A(origin, size); break;
        case 'B': write_B(origin, size); break;
        case 'C': write_C(origin, size); break;
        case 'E': write_E(origin, size); break;
        case 'H': write_H(origin, size); break;
        case 'I': write_I(origin, size); break;
        case 'L': write_L(origin, size); break;
        case 'O': write_O(origin, size); break;
        case 'P': write_P(origin, size); break;
        case 'R': write_R(origin, size); break;
        case 'T': write_T(origin, size); break;
        case 'W': write_W(origin, size); break;
        case '0': write_0(origin, size); break;
        case '1': write_1(origin, size); break;
        case ' ': break;  // Space - just move
        default:
            // For unsupported chars, draw a dot
            WritingArm_Dot(origin, 100);
            break;
    }
}

void WritingArm_WriteString(const char* str, ArmPoint_t origin, int16_t size, int16_t spacing){
    ArmPoint_t pos = origin;
    
    while(*str){
        WritingArm_WriteChar(*str, pos, size);
        // Move to next character position (adjust base angle)
        pos.base += spacing;
        str++;
    }
}

// =====================================================================
// Calibration & Testing
// =====================================================================
void WritingArm_Calibrate(void){
    // Test each channel individually
    // Base Servo (CH0)
    WritingArm_SetServoAngle(SERVO_BASE_CH, 0);
    delay_ms(1000);
    WritingArm_SetServoAngle(SERVO_BASE_CH, 90);
    delay_ms(1000);
    WritingArm_SetServoAngle(SERVO_BASE_CH, 180);
    delay_ms(1000);
    WritingArm_SetServoAngle(SERVO_BASE_CH, 90);
    delay_ms(500);
    
    // Arm Servo (CH1)
    WritingArm_SetServoAngle(SERVO_ARM_CH, 0);
    delay_ms(1000);
    WritingArm_SetServoAngle(SERVO_ARM_CH, 90);
    delay_ms(1000);
    WritingArm_SetServoAngle(SERVO_ARM_CH, 180);
    delay_ms(1000);
    WritingArm_SetServoAngle(SERVO_ARM_CH, 90);
    delay_ms(500);
    
    // Pen Actuator (CH2)
    WritingArm_PenDown();
    delay_ms(500);
    WritingArm_PenUp();
    delay_ms(500);
    
    // Eraser Actuator (CH3)
    WritingArm_EraserDown();
    delay_ms(500);
    WritingArm_EraserUp();
    delay_ms(500);
}

void WritingArm_ServoSweepTest(void){
    for(uint8_t ch = 0; ch < NUM_SERVOS; ++ch){
        // Sweep 0 to 180
        for(int a = 0; a <= 180; a += 30){
            PCA9685_SetServoAngle(s_addr, ch, a, s_min_us[ch], s_max_us[ch]);
            delay_ms(200);
        }
        // Sweep 180 to 0
        for(int a = 180; a >= 0; a -= 30){
            PCA9685_SetServoAngle(s_addr, ch, a, s_min_us[ch], s_max_us[ch]);
            delay_ms(200);
        }
    }
}

// Sweep a single channel from 0->180->0 using stored limits
void WritingArm_SweepChannel(uint8_t channel){
    if(channel >= NUM_SERVOS) return;
    for(int a = 0; a <= 180; a += 15){
        PCA9685_SetServoAngle(s_addr, channel, a, s_min_us[channel], s_max_us[channel]);
        delay_ms(120);
    }
    for(int a = 180; a >= 0; a -= 15){
        PCA9685_SetServoAngle(s_addr, channel, a, s_min_us[channel], s_max_us[channel]);
        delay_ms(120);
    }
}

void WritingArm_DemoStrokes(void){
    WritingArm_SelectTool(TOOL_PEN);
    WritingArm_Home();

    // Draw a short vertical stroke
    ArmPoint_t p1 = {90, 80};
    ArmPoint_t p2 = {90, 100};
    WritingArm_DrawLine(p1, p2, 20);
    delay_ms(500);

    // Draw a horizontal stroke
    p1.base = 70; p1.arm = 90;
    p2.base = 110; p2.arm = 90;
    WritingArm_DrawLine(p1, p2, 20);
    delay_ms(500);

    // Draw a diagonal stroke
    p1.base = 70; p1.arm = 70;
    p2.base = 110; p2.arm = 110;
    WritingArm_DrawLine(p1, p2, 20);
    delay_ms(500);

    // Make a dot
    p1.base = 90; p1.arm = 120;
    WritingArm_Dot(p1, 300);

    // Try eraser
    WritingArm_SelectTool(TOOL_ERASER);
    p1.base = 80; p1.arm = 90;
    p2.base = 100; p2.arm = 90;
    WritingArm_DrawLine(p1, p2, 30);
    
    WritingArm_SelectTool(TOOL_NONE);
    WritingArm_Home();
}

void WritingArm_DemoSquare(void){
    WritingArm_SelectTool(TOOL_PEN);
    WritingArm_Home();
    
    // Define square corners (in angle space)
    ArmPoint_t corners[4] = {
        {70, 70},    // Bottom-left
        {110, 70},   // Bottom-right
        {110, 110},  // Top-right
        {70, 110}    // Top-left
    };
    
    // Draw square
    for(int i = 0; i < 4; i++){
        int next = (i + 1) % 4;
        WritingArm_DrawLine(corners[i], corners[next], 20);
    }
    
    WritingArm_SelectTool(TOOL_NONE);
    WritingArm_Home();
}

void WritingArm_DemoLetters(void){
    WritingArm_SelectTool(TOOL_PEN);
    WritingArm_Home();
    
    // Write "HI" at starting position
    ArmPoint_t origin = {60, 70};
    int16_t size = 20;  // Letter size in degrees
    int16_t spacing = 25;  // Space between letters
    
    WritingArm_WriteString("HI", origin, size, spacing);
    
    delay_ms(1000);
    
    // Write "WIPE" below
    origin.base = 50;
    origin.arm = 100;
    WritingArm_WriteString("WIPE", origin, size, spacing);
    
    WritingArm_SelectTool(TOOL_NONE);
    WritingArm_Home();
}

// WritingArm_PrintState - No-op (debug output removed)
void WritingArm_PrintState(void){
    // State info available via WritingArm_GetState()
}
