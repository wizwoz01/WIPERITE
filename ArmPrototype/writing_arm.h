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

void WritingArm_DemoStrokes(void);
void WritingArm_ServoSweepTest(void);

#endif // WRITING_ARM_H_
