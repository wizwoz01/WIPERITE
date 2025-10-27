/////////////////////////////////////////////////////////////////////////////
// Course Number: CECS 347
// Assignment: Example project for Hardware PWM controlled Car
// Description: 
/////////////////////////////////////////////////////////////////////////////
#include <stdint.h>

// Constant definitions based on the following hardware interface:
// System clock is 16MHz.
#define PERIOD 16000 //  16MHz/1000=16000
#define START_SPEED 16000*0.9
#define SPEED_50  5000
#define SPEED_35	3500
#define SPEED_20  2000
#define SPEED_10 	1000

// PB5432 are used for direction control on L298.
// Motor 1 is connected to the left wheel, Motor 2 is connected to the right wheel.

//////////////////////1. Declarations Section////////////////////////////////
////////// Function Prototypes //////////
// Dependency: None
// Inputs: None
// Outputs: None
// Description: 
//	Initializes the PWM module 1 signals tied to PF321 on the Tiva Launchpad 
//		to allow for changing brightness of LEDs based on vehicle speed.
//	Initializes the PWM module 0 signals tied to PE5 and PB5 to be used with the 
//		L298N motor driver allowing for a variable speed of robot car.
void PWM_Init(void);

// Dependency: PWM_Init()
// Inputs: 
//	duty_L is the value corresponding to the duty cycle of the left wheel (PB5)
//	duty_R is the value corresponding to the duty cycle of the right wheel (PE5)
// Outputs: None 
// Description: Changes the duty cycles of the PWM signals by changing the CMP registers
void PWM_Duty(uint16_t duty_L, uint16_t duty_R);
void PWM_PE5_Duty(uint16_t duty_R);
void PWM_PB5_Duty(uint16_t duty_L);
/////////////////////////////////////////////////////////////////////////////
