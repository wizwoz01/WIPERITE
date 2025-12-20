// motor_drawing.c
// Servo-based drawing controller for WIPERITE
// Uses PCA9685 servo controller via writing_arm module to draw letters, numbers, and shapes
// by Min He and WIPERITE team

#include <stdint.h>
#include <string.h>
#include "motor_drawing.h"
#include "writing_arm.h"
#include "SysTick.h"
#include "UART0.h"
#include "motor.h"
#include "PWM.h"

// Forward declare SimpleLink cooperative task so long delays keep WiFi alive
extern void _SlNonOsMainLoopTask(void);

// =====================================================================
// Internal State
// =====================================================================

static MotorDrawMode_t s_mode = DRAW_MODE_OFF;
static uint8_t s_scale = 5;     // 1-10, default 5 (maps to letter size in degrees)
static uint8_t s_speed = 5;     // 1-10, default 5 (maps to step delay)
static volatile uint8_t s_busy = 0;
static volatile uint8_t s_stop_requested = 0;
// Debug jog state for PCA9685 channels
static int16_t s_dbg_ch1_angle = 90;
static int16_t s_dbg_ch2_angle = 90;
#define DBG_JOG_STEP_DEG 2
#define DBG_CH1 WARM_CH_BASE
#define DBG_CH2 WARM_CH_ARM

// Drawing origin position (can be adjusted)
static ArmPoint_t s_origin = {60, 70};
static int16_t s_cursor_offset = 0;  // Current X offset for string drawing

// =====================================================================
// Helper Functions
// =====================================================================

// Cooperative delay that services WiFi and checks for stop
static void draw_delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms && !s_stop_requested; i++) {
        SysTick_Wait(T1ms);
        _SlNonOsMainLoopTask();
    }
}

// Debug jog helper for PCA9685 manual sweeps
static void dbg_jog_channel(uint8_t channel, int16_t* angle_ptr, int16_t delta){
    int16_t a = *angle_ptr + delta;
    if(a < 0) a = 0;
    if(a > 180) a = 180;
    *angle_ptr = a;
    WritingArm_SetServoAngle(channel, (uint16_t)a);
}

// Double-press detector for 'k' sweep trigger
static uint8_t s_k_arm = 0;

// Get letter size based on scale (degrees)
static int16_t get_letter_size(void) {
    // Scale 1 = 10 degrees, Scale 5 = 20 degrees, Scale 10 = 35 degrees
    return (int16_t)(10 + (s_scale - 1) * 3);
}

// Get step delay based on speed (ms)
static uint16_t get_step_delay(void) {
    // Speed 1 = 40ms (slow), Speed 5 = 20ms, Speed 10 = 5ms (fast)
    if (s_speed <= 2) return 40;
    if (s_speed <= 4) return 30;
    if (s_speed <= 6) return 20;
    if (s_speed <= 8) return 10;
    return 5;
}

// Get letter spacing based on scale
static int16_t get_letter_spacing(void) {
    return get_letter_size() + 5;
}

// =====================================================================
// Core API Implementation
// =====================================================================

void MotorDraw_Init(void) {
    s_mode = DRAW_MODE_OFF;
    s_scale = 5;
    s_speed = 5;
    s_busy = 0;
    s_stop_requested = 0;
    s_cursor_offset = 0;
    s_origin.base = 60;
    s_origin.arm = 70;
    // Prime debug jog angles with current state
    WritingArmState_t st = WritingArm_GetState();
    s_dbg_ch1_angle = st.base_angle;
    s_dbg_ch2_angle = st.arm_angle;
}

void MotorDraw_EnterMode(void) {
    if (s_mode == DRAW_MODE_OFF) {
        s_mode = DRAW_MODE_READY;
        s_stop_requested = 0;
        s_cursor_offset = 0;
        // Select pen tool and go home
        WritingArm_SelectTool(TOOL_PEN);
        WritingArm_Home();
        // Sync jog angles with current servo positions
        WritingArmState_t st = WritingArm_GetState();
        s_dbg_ch1_angle = st.base_angle;
        s_dbg_ch2_angle = st.arm_angle;
        UART_OutString((uint8_t*)"Draw Mode ON (Servo)\r\n");
    }
}

void MotorDraw_ExitMode(void) {
    MotorDraw_Stop();
    WritingArm_AllToolsUp();
    WritingArm_Home();
    s_mode = DRAW_MODE_OFF;
    s_cursor_offset = 0;
    UART_OutString((uint8_t*)"Draw Mode OFF\r\n");
    // Re-enable DC motor PWM outputs when leaving draw mode
    PWM_PinsToPWM();
    MotorPWM_Enable();
    MotorControl_SetEnableAllowed(1);
}

MotorDrawMode_t MotorDraw_GetMode(void) {
    return s_mode;
}

void MotorDraw_SetScale(uint8_t scale) {
    if (scale < 1) scale = 1;
    if (scale > 10) scale = 10;
    s_scale = scale;
}

uint8_t MotorDraw_GetScale(void) {
    return s_scale;
}

void MotorDraw_SetSpeed(uint8_t speed) {
    if (speed < 1) speed = 1;
    if (speed > 10) speed = 10;
    s_speed = speed;
}

uint8_t MotorDraw_GetSpeed(void) {
    return s_speed;
}

int MotorDraw_IsBusy(void) {
    return s_busy;
}

void MotorDraw_Stop(void) {
    s_stop_requested = 1;
    WritingArm_AllToolsUp();
    s_busy = 0;
    if (s_mode == DRAW_MODE_EXECUTING) {
        s_mode = DRAW_MODE_READY;
    }
}

// =====================================================================
// Letter Drawing - Uses WritingArm_WriteChar
// =====================================================================

void MotorDraw_Letter(char c) {
    if (s_mode == DRAW_MODE_OFF) return;
    
    s_busy = 1;
    s_mode = DRAW_MODE_EXECUTING;
    s_stop_requested = 0;
    
    // Calculate current position with cursor offset
    ArmPoint_t pos;
    pos.base = s_origin.base + s_cursor_offset;
    pos.arm = s_origin.arm;
    
    int16_t size = get_letter_size();
    
    // Use the writing arm to draw the character
    WritingArm_WriteChar(c, pos, size);
    
    // Advance cursor for next character
    s_cursor_offset += get_letter_spacing();
    
    // Reset cursor if it goes too far
    if (s_cursor_offset > 100) {
        s_cursor_offset = 0;
        s_origin.arm += size + 10;  // Move to next line
    }
    
    s_busy = 0;
    if (s_mode == DRAW_MODE_EXECUTING) {
        s_mode = DRAW_MODE_READY;
    }
}

void MotorDraw_Digit(char c) {
    // Digits use the same drawing system as letters
    MotorDraw_Letter(c);
}

void MotorDraw_String(const char* str) {
    if (s_mode == DRAW_MODE_OFF || str == 0) return;
    
    ArmPoint_t pos;
    pos.base = s_origin.base;
    pos.arm = s_origin.arm;
    
    int16_t size = get_letter_size();
    int16_t spacing = get_letter_spacing();
    
    WritingArm_WriteString(str, pos, size, spacing);
}

// =====================================================================
// Shape Drawing - Uses WritingArm primitives
// =====================================================================

void MotorDraw_Square(void) {
    if (s_mode == DRAW_MODE_OFF) {
        // Allow shape drawing even when not in draw mode for backward compat
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // Define square corners relative to origin
    ArmPoint_t corners[4] = {
        {s_origin.base, s_origin.arm},
        {s_origin.base + size, s_origin.arm},
        {s_origin.base + size, s_origin.arm + size},
        {s_origin.base, s_origin.arm + size}
    };
    
    // Draw square
    for (int i = 0; i < 4 && !s_stop_requested; i++) {
        int next = (i + 1) % 4;
        WritingArm_DrawLine(corners[i], corners[next], delay);
    }
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_Triangle(void) {
    if (s_mode == DRAW_MODE_OFF) {
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // Define triangle corners
    ArmPoint_t corners[3] = {
        {s_origin.base, s_origin.arm},
        {s_origin.base + size, s_origin.arm},
        {s_origin.base + size/2, s_origin.arm + size}
    };
    
    // Draw triangle
    for (int i = 0; i < 3 && !s_stop_requested; i++) {
        int next = (i + 1) % 3;
        WritingArm_DrawLine(corners[i], corners[next], delay);
    }
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_Circle(void) {
    if (s_mode == DRAW_MODE_OFF) {
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // Draw circle as arc from 0 to 360
    ArmPoint_t center;
    center.base = s_origin.base + size/2;
    center.arm = s_origin.arm + size/2;
    
    WritingArm_DrawArc(center, size/2, 0, 360, delay);
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_Star(void) {
    if (s_mode == DRAW_MODE_OFF) {
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // 5-pointed star vertices (every other point)
    // Points at angles: 90, 162, 234, 306, 18 degrees
    ArmPoint_t center;
    center.base = s_origin.base + size/2;
    center.arm = s_origin.arm + size/2;
    
    // Simple star approximation with lines
    ArmPoint_t points[5];
    points[0].base = center.base; points[0].arm = center.arm + size/2;  // top
    points[1].base = center.base + size/2; points[1].arm = center.arm - size/3;  // bottom right
    points[2].base = center.base - size/3; points[2].arm = center.arm + size/6;  // left
    points[3].base = center.base + size/3; points[3].arm = center.arm + size/6;  // right
    points[4].base = center.base - size/2; points[4].arm = center.arm - size/3;  // bottom left
    
    // Draw star by connecting every other point
    for (int i = 0; i < 5 && !s_stop_requested; i++) {
        int next = (i + 2) % 5;
        WritingArm_DrawLine(points[i], points[next], delay);
    }
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_ZigZag(void) {
    if (s_mode == DRAW_MODE_OFF) {
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // ZigZag pattern
    ArmPoint_t points[5];
    points[0].base = s_origin.base; points[0].arm = s_origin.arm;
    points[1].base = s_origin.base + size/3; points[1].arm = s_origin.arm + size;
    points[2].base = s_origin.base + size*2/3; points[2].arm = s_origin.arm;
    points[3].base = s_origin.base + size; points[3].arm = s_origin.arm + size;
    points[4].base = s_origin.base + size + size/3; points[4].arm = s_origin.arm;
    
    for (int i = 0; i < 4 && !s_stop_requested; i++) {
        WritingArm_DrawLine(points[i], points[i+1], delay);
    }
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_FigureEight(void) {
    if (s_mode == DRAW_MODE_OFF) {
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // Two circles stacked
    ArmPoint_t center1, center2;
    center1.base = s_origin.base + size/2;
    center1.arm = s_origin.arm + size/4;
    center2.base = s_origin.base + size/2;
    center2.arm = s_origin.arm + size*3/4;
    
    WritingArm_DrawArc(center1, size/4, 0, 360, delay);
    WritingArm_DrawArc(center2, size/4, 0, 360, delay);
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_Heart(void) {
    if (s_mode == DRAW_MODE_OFF) {
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // Heart shape: two arcs at top + V at bottom
    ArmPoint_t bottom, left_top, right_top;
    bottom.base = s_origin.base + size/2;
    bottom.arm = s_origin.arm;
    left_top.base = s_origin.base + size/4;
    left_top.arm = s_origin.arm + size*2/3;
    right_top.base = s_origin.base + size*3/4;
    right_top.arm = s_origin.arm + size*2/3;
    
    // Draw left curve
    WritingArm_DrawLine(bottom, left_top, delay);
    // Draw right curve  
    ArmPoint_t mid_top;
    mid_top.base = s_origin.base + size/2;
    mid_top.arm = s_origin.arm + size;
    WritingArm_DrawLine(left_top, mid_top, delay);
    WritingArm_DrawLine(mid_top, right_top, delay);
    WritingArm_DrawLine(right_top, bottom, delay);
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_Diamond(void) {
    if (s_mode == DRAW_MODE_OFF) {
        WritingArm_SelectTool(TOOL_PEN);
    }
    
    s_busy = 1;
    s_stop_requested = 0;
    
    int16_t size = get_letter_size();
    uint16_t delay = get_step_delay();
    
    // Diamond shape (rotated square)
    ArmPoint_t corners[4];
    corners[0].base = s_origin.base + size/2; corners[0].arm = s_origin.arm;  // bottom
    corners[1].base = s_origin.base + size; corners[1].arm = s_origin.arm + size/2;  // right
    corners[2].base = s_origin.base + size/2; corners[2].arm = s_origin.arm + size;  // top
    corners[3].base = s_origin.base; corners[3].arm = s_origin.arm + size/2;  // left
    
    for (int i = 0; i < 4 && !s_stop_requested; i++) {
        int next = (i + 1) % 4;
        WritingArm_DrawLine(corners[i], corners[next], delay);
    }
    
    WritingArm_AllToolsUp();
    s_busy = 0;
}

void MotorDraw_Shape(MotorShape_t shape) {
    switch (shape) {
        case SHAPE_SQUARE:       MotorDraw_Square(); break;
        case SHAPE_TRIANGLE:     MotorDraw_Triangle(); break;
        case SHAPE_CIRCLE:       MotorDraw_Circle(); break;
        case SHAPE_STAR:         MotorDraw_Star(); break;
        case SHAPE_ZIGZAG:       MotorDraw_ZigZag(); break;
        case SHAPE_FIGURE_EIGHT: MotorDraw_FigureEight(); break;
        case SHAPE_HEART:        MotorDraw_Heart(); break;
        case SHAPE_DIAMOND:      MotorDraw_Diamond(); break;
        default: break;
    }
}

// =====================================================================
// Custom Pattern Execution
// =====================================================================

void MotorDraw_ExecuteSteps(const DrawStep_t* steps, uint8_t numSteps) {
    // This function is less relevant for servo drawing but kept for API compatibility
    // Could be extended to support custom servo sequences
    (void)steps;
    (void)numSteps;
}

// =====================================================================
// Command Processing
// =====================================================================

// In draw mode, commands are interpreted as:
// Letters A-Z: draw that letter
// Digits 0-9: draw that digit
// !: Square    @: Triangle   #: Circle
// $: Star      %: ZigZag     ^: Figure-8
// &: Heart     *: Diamond
// +: increase scale   -: decrease scale
// >: increase speed   <: decrease speed
// ESC or Q: exit draw mode
// Space: stop current drawing
// P: select pen tool
// E: select eraser tool
// H: home position

int MotorDraw_ProcessCommand(unsigned char cmd) {
    if (s_mode == DRAW_MODE_OFF) {
        // Check for draw mode entry command
        if (cmd == '~' || cmd == '`') {
            MotorDraw_EnterMode();
            return 1;
        }
        s_k_arm = 0;
        return 0;  // Not handled, pass to normal motor control
    }
    
    // In draw mode
    if (s_busy) {
        // Only stop command works while executing
        if (cmd == ' ' || cmd == 'H' || cmd == 'h' || cmd == 27) {  // ESC = 27
            MotorDraw_Stop();
            return 1;
        }
        return 1;  // Ignore other commands while busy
    }

    // Handle double-'k' sweep trigger (case-insensitive)
    if (cmd == 'k' || cmd == 'K') {
        if (s_k_arm) {
            s_k_arm = 0;
            WritingArm_SweepChannel(WARM_CH_BASE); // channel 0
            UART_OutString((uint8_t*)"Sweep CH0 (k,k)\r\n");
        } else {
            s_k_arm = 1;
        }
        return 1;
    } else {
        s_k_arm = 0;
    }

    // Debug jog commands (arrow keys mapped in main when in draw mode)
    switch(cmd){
        case MTRDRAW_CMD_CH1_DEC:
            dbg_jog_channel(DBG_CH1, &s_dbg_ch1_angle, -DBG_JOG_STEP_DEG);
            UART_OutString((uint8_t*)"CH1 angle: ");
            UART_OutUDec((uint32_t)s_dbg_ch1_angle);
            UART_OutString((uint8_t*)"\r\n");
            return 1;
        case MTRDRAW_CMD_CH1_INC:
            dbg_jog_channel(DBG_CH1, &s_dbg_ch1_angle, DBG_JOG_STEP_DEG);
            UART_OutString((uint8_t*)"CH1 angle: ");
            UART_OutUDec((uint32_t)s_dbg_ch1_angle);
            UART_OutString((uint8_t*)"\r\n");
            return 1;
        case MTRDRAW_CMD_CH2_INC:
            dbg_jog_channel(DBG_CH2, &s_dbg_ch2_angle, DBG_JOG_STEP_DEG);
            UART_OutString((uint8_t*)"CH2 angle: ");
            UART_OutUDec((uint32_t)s_dbg_ch2_angle);
            UART_OutString((uint8_t*)"\r\n");
            return 1;
        case MTRDRAW_CMD_CH2_DEC:
            dbg_jog_channel(DBG_CH2, &s_dbg_ch2_angle, -DBG_JOG_STEP_DEG);
            UART_OutString((uint8_t*)"CH2 angle: ");
            UART_OutUDec((uint32_t)s_dbg_ch2_angle);
            UART_OutString((uint8_t*)"\r\n");
            return 1;
        default:
            break;
    }
    
    // Process commands in ready state
    switch (cmd) {
        // Exit draw mode
        case 27:  // ESC
            MotorDraw_ExitMode();
            return 1;
        
        // Q in draw mode exits draw mode (doesn't quit app)
        case 'Q':
        case 'q':
            MotorDraw_ExitMode();
            return 1;
        
        // Tool selection
        case 'P':
        case 'p':
            WritingArm_SelectTool(TOOL_PEN);
            UART_OutString((uint8_t*)"Pen selected\r\n");
            return 1;
        
        case 'E':
        case 'e':
            WritingArm_SelectTool(TOOL_ERASER);
            UART_OutString((uint8_t*)"Eraser selected\r\n");
            return 1;
        
        // Home position
        case 'H':
        case 'h':
            WritingArm_Home();
            s_cursor_offset = 0;
            return 1;
        
        // Letters A-Z (except reserved: P, E, H, Q)
        case 'A': case 'B': case 'C': case 'D': case 'F':
        case 'G': case 'I': case 'J': case 'K': case 'L':
        case 'M': case 'N': case 'O': case 'R': case 'S':
        case 'T': case 'U': case 'V': case 'W': case 'X':
        case 'Y': case 'Z':
        case 'a': case 'b': case 'c': case 'd': case 'f':
        case 'g': case 'i': case 'j': case 'k': case 'l':
        case 'm': case 'n': case 'o': case 'r': case 's':
        case 't': case 'u': case 'v': case 'w': case 'x':
        case 'y': case 'z':
            MotorDraw_Letter(cmd);
            return 1;
        
        // Digits
        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
            MotorDraw_Digit(cmd);
            return 1;
        
        // Shapes
        case '!':
            MotorDraw_Square();
            return 1;
        case '@':
            MotorDraw_Triangle();
            return 1;
        case '#':
            MotorDraw_Circle();
            return 1;
        case '$':
            MotorDraw_Star();
            return 1;
        case '%':
            MotorDraw_ZigZag();
            return 1;
        case '^':
            MotorDraw_FigureEight();
            return 1;
        case '&':
            MotorDraw_Heart();
            return 1;
        case '*':
            MotorDraw_Diamond();
            return 1;
        
        // Scale adjustment
        case '+':
        case '=':
            if (s_scale < 10) {
                s_scale++;
                UART_OutString((uint8_t*)"Scale: ");
                UART_OutUDec(s_scale);
                UART_OutString((uint8_t*)"\r\n");
            }
            return 1;
        case '-':
        case '_':
            if (s_scale > 1) {
                s_scale--;
                UART_OutString((uint8_t*)"Scale: ");
                UART_OutUDec(s_scale);
                UART_OutString((uint8_t*)"\r\n");
            }
            return 1;
        
        // Speed adjustment
        case '>':
        case '.':
            if (s_speed < 10) {
                s_speed++;
                UART_OutString((uint8_t*)"Speed: ");
                UART_OutUDec(s_speed);
                UART_OutString((uint8_t*)"\r\n");
            }
            return 1;
        case '<':
        case ',':
            if (s_speed > 1) {
                s_speed--;
                UART_OutString((uint8_t*)"Speed: ");
                UART_OutUDec(s_speed);
                UART_OutString((uint8_t*)"\r\n");
            }
            return 1;
        
        // Stop / lift tools
        case ' ':
            WritingArm_AllToolsUp();
            return 1;
        
        // New line (reset cursor, move arm down)
        case '\r':
        case '\n':
            s_cursor_offset = 0;
            s_origin.arm += get_letter_size() + 10;
            return 1;
        
        default:
            // Unknown command in draw mode - ignore
            return 1;
    }
}
