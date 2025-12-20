// SysTick.h
// Runs on TM4C123
// By Dr. Min He
// December 10th, 2018
 
#include <stdint.h>

#define T1ms                             50000U   // Systick reload value to generate 1ms delay, assumes using 50 MHz Clock.
#define ONE_SEC                      100*T1ms     //systick reload value for 0.5s

// Time delay using busy wait.
void SysTick_Wait(uint32_t delay);
void SysTick_Init(void);