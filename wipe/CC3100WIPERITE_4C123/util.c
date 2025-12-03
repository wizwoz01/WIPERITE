/*
 * util.c
 *
 * Description: Main implementation of basic utility function such as
 * delay and value mapping
 * Authors: Jonny, Michelle, Ricardo, Ryan, 11-05-2025
 * Group 1
 *
 */
 
#include "util.h"
#include "../inc/tm4c123gh6pm.h"

/* Local Macros */
#define TIMER_32_MAX_RELOAD		(0)	
 
/* The reason why Wide Timer is used instead of regular time is because
	 of the prescaler option */
void WTIMER0_Init(void){
	SYSCTL_RCGCWTIMER_R |= EN_WTIMER0_CLOCK;      // Enable clock for WTIMER0
	SYSCTL_RCGCGPIO_R |= 0x20;            // Enable clock for Port F (optional)
	while((SYSCTL_PRWTIMER_R & EN_WTIMER0_CLOCK) == 0);

	WTIMER0_CTL_R &= ~WTIMER0_TAEN_BIT;           // Disable WTIMER0A during setup
	WTIMER0_CFG_R = WTIMER0_32_BIT_CFG;           // 32-bit timer mode
	WTIMER0_TAMR_R = WTIMER0_PERIOD_MODE;         // Periodic mode, down-count
	WTIMER0_TAILR_R = 16000 - 1;                  // 1 ms for 16 MHz clock
	WTIMER0_TAPR_R = PRESCALER_VALUE; 						// set reload value
	WTIMER0_ICR_R = 0x1;                          // Clear timeout flag
	WTIMER0_CTL_R |= WTIMER0_TAEN_BIT;            // Enable WTIMER0A
}

void DELAY_1MS(uint32_t delay){
	uint32_t i;
	for (i = 0; i < delay; i++) {
			while((WTIMER0_RIS_R & EN_WTIMER0_CLOCK) == 0); // Wait for timeout
			WTIMER0_ICR_R = EN_WTIMER0_CLOCK;               // Acknowledge timeout
	}
}

