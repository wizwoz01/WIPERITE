/////////////////////////////////////////////////////////////////////////////
// Course Number: CECS 347
// Assignment: Example project for Hardware PWM controlled Car
// Description: 
/////////////////////////////////////////////////////////////////////////////

//////////////////////1. Pre-processor Directives Section////////////////////
#include "tm4c123gh6pm.h"
#include "PWM.h"
#include <stdint.h>

////////// Local Global Variables //////////
/////////////////////////////////////////////////////////////////////////////

//////////////////////3. Subroutines Section/////////////////////////////////
// Dependency: None
// Inputs: None
// Outputs: None
// Description: 
// Initializes the PWM module 0 generators to be used with the 
// L298N motor driver allowing for a variable speed of robot car.
void PWM_Init(void){
	// Enable clocks for Port B, Port E, and PWM0
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
	}
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOE)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;	// Activate E clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOE)==0){};
	}
	SYSCTL_RCGCPWM_R |= 0x01;	// activate PWM0
	
	// Configure PE5 for M0PWM5
	GPIO_PORTE_AFSEL_R |= 0x20;	// enable alt funct on PE5
  GPIO_PORTE_PCTL_R &= ~0x00F00000; // PWM to be used
  GPIO_PORTE_PCTL_R |= 0x00400000; // PWM to be used
  GPIO_PORTE_DEN_R |= 0x20;	// enable digital I/O 
	
	// Configure PB5 for M0PWM3
	GPIO_PORTB_AFSEL_R |= 0x20;	// enable alt funct on PB5
	GPIO_PORTB_PCTL_R &= ~0x00F00000; // PWM to be used
	GPIO_PORTB_PCTL_R |= 0x00400000; // PWM to be used
	GPIO_PORTB_DEN_R |= 0x20;	// enable digital I/O
	
	SYSCTL_RCC_R &= ~0x001E0000; // Clear any previous PWM divider values
	
	// PWM0_2 output B (PE5) Initialization
	PWM0_2_CTL_R = 0;	// re-loading down-counting mode
	PWM0_2_GENB_R |= 0xC08;// low on LOAD, high on CMPB down
	PWM0_2_LOAD_R = PERIOD - 1;	// cycles needed to count down to 0
	PWM0_2_CMPB_R = 0;	// count value when output rises
	
	// PWM0_1 output B (PB5) Initialization
	PWM0_1_CTL_R = 0; // re-loading down-counting mode
	PWM0_1_GENB_R |= 0xC08; // low on LOAD, high on CMPB down
	PWM0_1_LOAD_R = PERIOD - 1; // cycles needed to count down to 0
	PWM0_1_CMPB_R = 0; // count value when output rises
	
	PWM0_2_CTL_R |= 0x00000001;	// Enable PWM0 Generator 2 in Countdown mode
	PWM0_1_CTL_R |= 0x00000001; // Enable PWM0 Generator 1 in Countdown mode
	PWM0_ENABLE_R &= ~0x00000028;	// Disable PE5(M0PWM5) and PB5(M0PWM3) on initialization
}


// Dependency: PWM_Init()
// Inputs: PWM timer is 16 bits wide.
//	duty_L is the value corresponding to the duty cycle of the left wheel (PB5)
//	duty_R is the value corresponding to the duty cycle of the right wheel (PE5)
// Outputs: None 
// Description: Changes the duty cycles of the PWM signals by changing the CMP registers
void PWM_Duty(uint16_t duty_L, uint16_t duty_R){
	PWM0_1_CMPB_R = duty_L - 1;	// PB5 count value when output rises
  PWM0_2_CMPB_R = duty_R - 1;	// PE5 count value when output rises
}

void PWM_PE5_Duty(uint16_t duty_R){
	PWM0_2_CMPB_R = duty_R -1; //PE5 count value when output rises
}

void PWM_PB5_Duty(uint16_t duty_L){
	PWM0_1_CMPB_R = duty_L -1; //PB5 count value when output rises
}
/////////////////////////////////////////////////////////////////////////////
