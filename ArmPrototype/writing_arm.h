#ifndef WRITING_ARM_H_
#define WRITING_ARM_H_

#include <stdint.h>


void WritingArm_Init(uint8_t pca_addr);
void WritingArm_Home(void);

void WritingArm_PenUp(void);
void WritingArm_PenDown(void);
void WritingArm_EraserUp(void);
void WritingArm_EraserDown(void);

void WritingArm_MoveToAngles(int16_t base, int16_t arm);
void WritingArm_SetServoUs(uint8_t channel, uint16_t pulse_us);
void WritingArm_SetServoAngle(uint8_t channel, int16_t angle_deg);

void WritingArm_ServoSweepTest(void);
//void WritingArm_ServoSweepTest(uint8_t channel, int16_t start_deg, int16_t end_deg, int16_t step_deg, uint32_t delay_each_ms);

// Draw a single character at the current baseline X offset (grid columns).
// Uses a simple 5x7 dot font mapped into base/arm angles.
void WritingArm_DrawChar(char c, int x_origin);

// Internal helper: draw a character at a specific grid (x,y) origin. Exposed
// for testing and diagnostic purposes.
void WritingArm_DrawCharAt(char c, int x0, int y0);

// Draw a NUL-terminated string starting at baseline X origin.
void WritingArm_DrawString(const char *s, int x_origin);

// Text buffer API: queue/clear/flush. Queue will append strings to an
// internal buffer that WritingArm_WriteBuffer will lay out starting at the
// paper top-left (0,0) and write left->right, top->bottom.
void WritingArm_ClearTextBuffer(void);
int  WritingArm_QueueText(const char *s);
void WritingArm_WriteBuffer(void);

// Diagnostic: print mapping samples (corners + center) over UART
void WritingArm_DumpMappingSamples(void);

#endif // WRITING_ARM_H_
