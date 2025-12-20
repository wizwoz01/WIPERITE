/*
 * util.h
 *
 * Description: Provides basic utility function such as delay and 
 * value mapping
 * Authors: Jonny, Michelle, Ricardo, Ryan, 11-05-2025
 * Group 1
 *
 */
 

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

#define CONSTANT_FILL	(50)     // a place holder for all constants needs to be defined by students
#define CODE_FILL	(0)     // a place holder for code needs to be defined by students

/* List of Fill In Macros */
#define EN_WTIMER0_CLOCK			(0x01)
#define WTIMER0_TAEN_BIT			(0x00000001)
#define WTIMER0_32_BIT_CFG		(0x00000004)
#define WTIMER0_PERIOD_MODE		(0x00000002)
#define PRESCALER_VALUE				(0)

void WTIMER0_Init(void);
void DELAY_1MS(uint32_t);
//int16_t map(int16_t, int16_t, int16_t, int16_t, int16_t);

#endif