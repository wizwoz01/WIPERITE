// SysTick.c
// Runs on TM4C123
// By Dr. Min He
// December 10th, 2018

#include <stdint.h>
#include "SysTick.h"
#include "tm4c123gh6pm.h"

void SysTick_Wait(uint32_t delay)
{
    NVIC_ST_RELOAD_R = delay - 1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
  
  while(!(NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT)){};
    
    NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
}

void SysTick_Init(void){
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC;
}