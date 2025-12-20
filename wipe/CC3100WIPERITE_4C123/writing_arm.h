#ifndef WRITING_ARM_H_
#define WRITING_ARM_H_

#include <stdint.h>

// =====================================================================
// Writing Arm Controller for WIPERITE
// Controls: Base servo (rotation), Arm servo (reach), 
//           Pen linear actuator, Eraser linear actuator
// =====================================================================

// Channel assignments (configurable in writing_arm.c)
#define WARM_CH_BASE      0   // Base rotation servo (0-180 degrees)
#define WARM_CH_ARM       1   // Arm extension servo (0-180 degrees)
#define WARM_CH_PEN       2   // Pen up/down linear actuator
#define WARM_CH_ERASER    3   // Eraser up/down linear actuator

// Tool selection
typedef enum {
    TOOL_NONE = 0,
    TOOL_PEN,
    TOOL_ERASER
} WritingTool_t;

// Writing state
typedef struct {
    int16_t base_angle;      // Current base angle (degrees)
    int16_t arm_angle;       // Current arm angle (degrees)
    uint8_t pen_down;        // 1 if pen is down
    uint8_t eraser_down;     // 1 if eraser is down
    WritingTool_t active_tool;
} WritingArmState_t;

// Point in arm coordinate space (angles)
typedef struct {
    int16_t base;   // Base angle (0-180)
    int16_t arm;    // Arm angle (0-180)
} ArmPoint_t;

// =====================================================================
// Core Initialization & Configuration
// =====================================================================
void WritingArm_Init(uint8_t pca_addr);
void WritingArm_SetServoLimits(uint8_t channel, uint16_t min_us, uint16_t max_us);
WritingArmState_t WritingArm_GetState(void);

// =====================================================================
// Low-Level Servo Control
// =====================================================================
void WritingArm_SetServoUs(uint8_t channel, uint16_t pulse_us);
void WritingArm_SetServoAngle(uint8_t channel, int16_t angle_deg);

// =====================================================================
// Tool Control (Pen & Eraser)
// =====================================================================
void WritingArm_PenUp(void);
void WritingArm_PenDown(void);
void WritingArm_EraserUp(void);
void WritingArm_EraserDown(void);
void WritingArm_AllToolsUp(void);
void WritingArm_SelectTool(WritingTool_t tool);

// =====================================================================
// Movement Commands
// =====================================================================
void WritingArm_Home(void);
void WritingArm_MoveToAngles(int16_t base, int16_t arm);
void WritingArm_MoveToAnglesSlow(int16_t base, int16_t arm, uint16_t step_delay_ms);
void WritingArm_MoveTo(ArmPoint_t pt);
void WritingArm_MoveToSlow(ArmPoint_t pt, uint16_t step_delay_ms);

// =====================================================================
// Drawing Primitives
// =====================================================================
void WritingArm_DrawLine(ArmPoint_t from, ArmPoint_t to, uint16_t step_delay_ms);
void WritingArm_DrawArc(ArmPoint_t center, int16_t radius_deg, int16_t start_angle, 
                        int16_t end_angle, uint16_t step_delay_ms);
void WritingArm_Dot(ArmPoint_t pt, uint16_t dwell_ms);

// =====================================================================
// Letter Writing (simplified stroke-based letters)
// =====================================================================
void WritingArm_WriteChar(char c, ArmPoint_t origin, int16_t size);
void WritingArm_WriteString(const char* str, ArmPoint_t origin, int16_t size, int16_t spacing);

// =====================================================================
// Calibration & Testing
// =====================================================================
void WritingArm_Calibrate(void);
void WritingArm_ServoSweepTest(void);
void WritingArm_DemoStrokes(void);
void WritingArm_DemoSquare(void);
void WritingArm_DemoLetters(void);

// Single-channel sweep (0..180..0) using stored limits
void WritingArm_SweepChannel(uint8_t channel);

// =====================================================================
// Debug/Diagnostic
// =====================================================================
void WritingArm_PrintState(void);

#endif // WRITING_ARM_H_
