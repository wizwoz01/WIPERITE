// motors.h
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// PE3-0 controls the direction of a Romi Chassis car.
// by Min He, 3/17/2024

// This header exposes motor control helpers aligned with HW_PWM_Car.
// Motors use PWM0 on PB6/PB7 and direction pins on PB5..PB2.

#include <stdint.h>

// Single initialization entry point
void MotorControl_Init(void);

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

// Optional mode flag exposed for UI/ISR (1=patterns, 2=direct drive)
extern volatile unsigned char g_mode;