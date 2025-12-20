#ifndef MOTOR_DRAWING_H_
#define MOTOR_DRAWING_H_

#include <stdint.h>

// =====================================================================
// Motor Drawing Controller for WIPERITE
// Allows the car to drive in patterns forming letters, numbers, and shapes
// Similar concept to writing_arm.c but uses DC motor movement
// =====================================================================

// Drawing mode states
typedef enum {
    DRAW_MODE_OFF = 0,      // Normal motor control mode
    DRAW_MODE_READY,        // Drawing mode active, waiting for command
    DRAW_MODE_EXECUTING     // Currently executing a drawing pattern
} MotorDrawMode_t;

// Shape types for drawing
typedef enum {
    SHAPE_NONE = 0,
    SHAPE_SQUARE,
    SHAPE_TRIANGLE,
    SHAPE_CIRCLE,
    SHAPE_STAR,
    SHAPE_ZIGZAG,
    SHAPE_FIGURE_EIGHT,
    SHAPE_HEART,
    SHAPE_DIAMOND
} MotorShape_t;

// Movement primitives for building patterns
typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_PIVOT_LEFT,
    MOVE_PIVOT_RIGHT,
    MOVE_TURN_LEFT,
    MOVE_TURN_RIGHT,
    MOVE_STOP,
    MOVE_DELAY
} MovePrimitive_t;

// A single step in a drawing sequence
typedef struct {
    MovePrimitive_t action;
    uint16_t duration_ms;   // How long to perform this action
    uint8_t speed_percent;  // Speed 0-100
} DrawStep_t;

// Maximum steps in a single pattern
#define MAX_DRAW_STEPS 32

// =====================================================================
// Core Initialization & Mode Control
// =====================================================================

// Initialize motor drawing module
void MotorDraw_Init(void);

// Enter/exit drawing mode
void MotorDraw_EnterMode(void);
void MotorDraw_ExitMode(void);
MotorDrawMode_t MotorDraw_GetMode(void);

// Set drawing scale (affects size of letters/shapes)
// scale: 1-10 where 1 is smallest, 10 is largest (default 5)
void MotorDraw_SetScale(uint8_t scale);
uint8_t MotorDraw_GetScale(void);

// Set drawing speed (affects movement speed)
// speed: 1-10 where 1 is slowest, 10 is fastest (default 5)
void MotorDraw_SetSpeed(uint8_t speed);
uint8_t MotorDraw_GetSpeed(void);

// =====================================================================
// Command Processing
// =====================================================================

// Debug jog commands (arrow keys while in draw mode)
#define MTRDRAW_CMD_CH1_DEC 0x91  // Left arrow
#define MTRDRAW_CMD_CH1_INC 0x92  // Right arrow
#define MTRDRAW_CMD_CH2_INC 0x93  // Up arrow
#define MTRDRAW_CMD_CH2_DEC 0x94  // Down arrow

// Process a command while in drawing mode
// Returns 1 if command was handled, 0 if should pass to normal motor control
int MotorDraw_ProcessCommand(unsigned char cmd);

// Check if currently executing a drawing pattern
int MotorDraw_IsBusy(void);

// Stop current drawing pattern immediately
void MotorDraw_Stop(void);

// =====================================================================
// Letter Drawing
// =====================================================================

// Draw a single letter (A-Z, case insensitive)
void MotorDraw_Letter(char c);

// Draw a single digit (0-9)
void MotorDraw_Digit(char c);

// Draw a string of characters
void MotorDraw_String(const char* str);

// =====================================================================
// Shape Drawing
// =====================================================================

// Draw a shape
void MotorDraw_Shape(MotorShape_t shape);

// Shape drawing convenience functions
void MotorDraw_Square(void);
void MotorDraw_Triangle(void);
void MotorDraw_Circle(void);
void MotorDraw_Star(void);
void MotorDraw_ZigZag(void);
void MotorDraw_FigureEight(void);
void MotorDraw_Heart(void);
void MotorDraw_Diamond(void);

// =====================================================================
// Custom Pattern Execution
// =====================================================================

// Execute a custom sequence of steps
void MotorDraw_ExecuteSteps(const DrawStep_t* steps, uint8_t numSteps);

#endif // MOTOR_DRAWING_H_
