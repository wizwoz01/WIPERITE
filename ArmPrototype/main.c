/*
 * main.c
 *
 * Description: Main implementation of the Writing Arm using the PCA9685 I2C Servo Driver
 * Authors: Ricardo C. 12-17-2025
 * Team 9
 */

#include <stdint.h>
#include <stdbool.h>
#include "UART0.h"
#include "SysTick.h"
#include "writing_arm.h"
#include "I2C.h"
#include "PCA9685.h"
#include "tm4c123gh6pm.h"

// Local channel mapping 
#define BASE_CH   0
#define ARM_CH    1
#define PEN_CH    2
#define ERASER_CH 3

static void delay_ms(uint32_t ms){
	while(ms--) SysTick_Wait(T1ms);
}

static void show_menu(void){
	UART0_OutString((uint8_t *)"Writing Arm Menu:\r\n");
	UART0_OutString((uint8_t *)"A/a : Sweep base channel forward\r\n");
	UART0_OutString((uint8_t *)"D/d : Sweep base channel reverse\r\n");
	UART0_OutString((uint8_t *)"W/w : Sweep pen channel forward\r\n");
	UART0_OutString((uint8_t *)"S/s : Sweep pen channel reverse\r\n");
	UART0_OutString((uint8_t *)"H/h : Home arm (pen/eraser up, mid angles)\r\n");
	UART0_OutString((uint8_t *)"M/m : Show this menu\r\n");
	UART0_OutString((uint8_t *)"T/t : PCA9685 register test (prints MODE1/PRESCALE, toggles CH0)\r\n");
	UART0_OutString((uint8_t *)"\r\nAwaiting command> ");
}

static void print_hex8(uint8_t v){
	char hex[3];
	const char *hexchars = "0123456789ABCDEF";
	hex[0] = hexchars[(v >> 4) & 0x0F];
	hex[1] = hexchars[v & 0x0F];
	hex[2] = '\0';
	UART0_OutString((uint8_t *)hex);
}

static void print_hex16(uint16_t v){
	char buf[5];
	const char *hexchars = "0123456789ABCDEF";
	buf[0] = hexchars[(v >> 12) & 0x0F];
	buf[1] = hexchars[(v >> 8) & 0x0F];
	buf[2] = hexchars[(v >> 4) & 0x0F];
	buf[3] = hexchars[v & 0x0F];
	buf[4] = '\0';
	UART0_OutString((uint8_t *)buf);
}

int main(void){
	uint8_t c;

	// Initialize UART and writing arm
	UART0_Init();
	WritingArm_Init(0x40); // default PCA9685 address
	// Quick runtime check: ensure I2C1 peripheral is ready
	if((SYSCTL_PRI2C_R & 0x02) == 0){
		UART0_OutString((uint8_t *)"Error: I2C not ready\r\n");
	}
	// Read PCA9685 MODE1 and validate response
	{
		uint8_t mode = I2C1_Receive(0x40, 0x00);
		if(mode == 0x00){
			UART0_OutString((uint8_t *)"Error: PCA9685 not responding on I2C\r\n");
		} else {
			/* PCA9685_Reset() enables Auto-Increment and All-Call => MODE1 bits 0x21 expected */
			if((mode & 0x21) != 0x21){
				UART0_OutString((uint8_t *)"Warning: PCA9685 MODE1=0x");
				char hex[3];
				const char *hexchars = "0123456789ABCDEF";
				hex[0] = hexchars[(mode >> 4) & 0x0F];
				hex[1] = hexchars[mode & 0x0F];
				hex[2] = '\0';
				UART0_OutString((uint8_t *)hex);
				UART0_OutString((uint8_t *)"\r\n");
			}
		}
	}
	WritingArm_Home();

	show_menu();
 
	for(;;){
		c = UART0_InChar();
		// echo and newline
		UART0_OutChar(c);
		UART0_NextLine();

		switch(c){
			case 'A': case 'a':{
				// sweep base 0->180
				for(int ang=0; ang<=180; ang+=5){
					WritingArm_SetServoAngle(ARM_CH, ang);
					delay_ms(25);
				}
				for(int ang=180; ang>=0; ang-=5){
					WritingArm_SetServoAngle(ARM_CH, ang);
					delay_ms(15);
				}
			} break;

			case 'D': case 'd':{
				// sweep base 180->0
				for(int ang=180; ang>=0; ang-=5){
					WritingArm_SetServoAngle(ARM_CH, ang);
					delay_ms(25);
				}
				for(int ang=0; ang<=180; ang+=5){
					WritingArm_SetServoAngle(ARM_CH, ang);
					delay_ms(15);
				}
			} break;

			case 'W': case 'w':{
				// sweep pen 0->180
				for(int ang=0; ang<=180; ang+=5){
					WritingArm_SetServoAngle(BASE_CH, ang);
					delay_ms(25);
				}
				for(int ang=180; ang>=0; ang-=5){
					WritingArm_SetServoAngle(BASE_CH, ang);
					delay_ms(15);
				}
			} break;

			case 'S': case 's':{
				// sweep pen 180->0
				for(int ang=180; ang>=0; ang-=5){
					WritingArm_SetServoAngle(BASE_CH, ang);
					delay_ms(25);
				}
				for(int ang=0; ang<=180; ang+=5){
					WritingArm_SetServoAngle(BASE_CH, ang);
					delay_ms(15);
				}
			} break;

			case 'H': case 'h':{
				WritingArm_Home();
			} break;

			case 'T': case 't':{
				UART0_OutString((uint8_t *)"PCA9685 Test:\r\n");
				uint8_t mode = PCA9685_read8(0x40, PCA9685_MODE1);
				UART0_OutString((uint8_t *)"MODE1=0x"); print_hex8(mode); UART0_OutString((uint8_t *)"\r\n");
				uint8_t mode2 = PCA9685_read8(0x40, PCA9685_MODE2);
				UART0_OutString((uint8_t *)"MODE2=0x"); print_hex8(mode2); UART0_OutString((uint8_t *)"\r\n");
				uint8_t pres = PCA9685_readPrescale(0x40);
				UART0_OutString((uint8_t *)"PRESCALE=0x"); print_hex8(pres); UART0_OutString((uint8_t *)"\r\n");
				UART0_OutString((uint8_t *)"Setting CH0 to 50% (test)\r\n");
				PCA9685_SetPin(0x40, 0, 2048, 0);
				// wait a bit for I2C write to complete and scope to see signal
				SysTick_Wait(100 * T1ms);
				uint16_t onv = PCA9685_getPWM(0x40, 0, 0);
				UART0_OutString((uint8_t *)"CH0 ON read=0x"); print_hex16(onv); UART0_OutString((uint8_t *)"\r\n");
				uint16_t off = PCA9685_getPWM(0x40, 0, 1);
				UART0_OutString((uint8_t *)"CH0 OFF read=0x"); print_hex16(off); UART0_OutString((uint8_t *)"\r\n");
			} break;

			case 'M': case 'm':{
				show_menu();
			} break;

			default:{
				UART0_OutString((uint8_t *)"Unknown command. Press M for menu.\r\n");
			} break;
		}

		UART0_OutString((uint8_t *)"\r\nAwaiting command> ");
	}

	// never reached
	return 0;
}

