// motors.h
// Runs on TM4C123
// PWM on PD0 (M1PWM0) and PD1 (M1PWM1) to drive motor speed.
// Direction control uses three pins on Port E (PE3-PE1) and one pin on Port D (PD2).
// Note: PE0 intentionally unused to avoid conflict with CC3100 (SPI CS).
// by Min He, 3/17/2024

// This header exposes motor control helpers aligned with HW_PWM_Car.
// Motors use PWM1 on PD0/PD1 and direction pins on PE3-PE1 plus PD2.

#include <stdint.h>

// Single initialization entry point
void MotorControl_Init(void);

// Enable PWM outputs after WiFi is connected; keeps PD0/PD1 quiet during join
void MotorPWM_Enable(void);
// Allow or prevent PWM enabling inside Car_ProcessCommand (runtime lock)
void MotorControl_SetEnableAllowed(uint8_t allowed);

void move_forward(void);
void move_backward(void);
void stop_the_car(void);
void forward_left(void);
void forward_right(void);
void backward_left(void);
void backward_right(void);
void pivot_left(void);
void pivot_right(void);

void PortB_Init(void);
// Use PortF_Init from GPIO.c for PF LEDs and switches

// High-level command processor (formerly in HW_PWM_Car)
void Car_ProcessCommand(unsigned char control_symbol);

// Modes removed: direct-drive and patterns are always available via commands.