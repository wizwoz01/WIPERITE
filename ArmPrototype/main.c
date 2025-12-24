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
#include "Button_LED.h"

// millisecond tick used for non-blocking debounce (updated in main loop)
volatile uint32_t System_ms = 0;

// Local channel mapping 
#define BASE_CH   0
#define ARM_CH    1
#define PEN_CH    2
#define ERASER_CH 3

#define ANGLE_STEP 5

/* Per-servo current angle state (0-180). Kept in sync by commands and on Home. */
static int baseAngle = 90;
static int armAngle = 110;
static int penAngle = 90;
static int eraserAngle = 90;
// write mode flag: 0 = normal, 1 = write-mode active
static volatile uint8_t write_mode = 0;

static void delay_ms(uint32_t ms){
	while(ms--) SysTick_Wait(T1ms);
}

static void show_menu(void){
	UART0_OutString((uint8_t *)"Writing Arm Menu:\r\n");
	UART0_OutString((uint8_t *)"A/a : Sweep base channel forward\r\n");
	UART0_OutString((uint8_t *)"D/d : Sweep base channel reverse\r\n");
	UART0_OutString((uint8_t *)"W/w : Sweep pen channel forward\r\n");
	UART0_OutString((uint8_t *)"S/s : Sweep pen channel reverse\r\n");
	UART0_OutString((uint8_t *)"J/j : Sweep pen channel forward (PEN_CH)\r\n");
	UART0_OutString((uint8_t *)"K/k : Sweep pen channel reverse (PEN_CH)\r\n");
	UART0_OutString((uint8_t *)"H/h : Home arm (pen/eraser up, mid angles)\r\n");
	UART0_OutString((uint8_t *)"M/m : Show this menu\r\n");
	UART0_OutString((uint8_t *)"T/t : PCA9685 register test (prints MODE1/PRESCALE, toggles CH8)\r\n");
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
	// Initialize on-board button and RGB LED (Port F)
	ButtonLED_Init();
	ButtonLED_SetColor(0,0,0);
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

	/* initialize local angle state to match WritingArm_Home() */
	baseAngle = 90;
	armAngle = 110;
	penAngle = 90;
	eraserAngle = 0;

	show_menu();

	for(;;){
		// Toggle write mode on button press
		if(ButtonLED_ButtonPressed()){
			ButtonLED_ClearButton();
			write_mode = !write_mode;
			if(write_mode){
				ButtonLED_SetColor(1,0,0); // indicate write mode (red)
				UART0_OutString((uint8_t *)"\r\nEntering WRITE mode. Press button again to exit.\r\n");
				WritingArm_Home();
				/* keep local state consistent with homing: base stays 90, arm 110, pen fully up */
				baseAngle = 90;
				armAngle = 110;
				penAngle = 90;
				eraserAngle = 0;
				UART0_OutString((uint8_t *)"Write> ");
			} else {
				ButtonLED_SetColor(0,0,0);
				UART0_OutString((uint8_t *)"\r\nExiting WRITE mode.\r\n");
				WritingArm_Home();
				/* keep local state consistent with homing */
				baseAngle = 90;
				armAngle = 110;
				penAngle = 90;
				eraserAngle = 0;
				show_menu();
			}
		}

		// When in write-mode, read lines from UART but allow button to cancel/exit
		if(write_mode){
			uint8_t buf[128];
			uint16_t idx = 0;
			// loop until newline or button toggles mode off
			while(write_mode){
				// check if user pressed button to exit
				if(ButtonLED_ButtonPressed()){
					ButtonLED_ClearButton();
					write_mode = 0;
					ButtonLED_SetColor(0,0,0);
					UART0_OutString((uint8_t *)"\r\nWrite mode canceled by button.\r\n");
					WritingArm_Home();
					/* keep local state consistent with homing */
					baseAngle = 90;
					armAngle = 110;
					penAngle = 90;
					eraserAngle = 0;
					show_menu();
					break;
				}

				// non-blocking UART read
				if((UART0_FR_R & UART_FR_RXFE) == 0){
					uint8_t ch = UART0_InChar();
					if(ch == CR || ch == LF){
						UART0_NextLine();
						if(idx){
							buf[idx] = 0;
							// Draw received string
							WritingArm_Home();
							WritingArm_DrawString((const char *)buf, 0);
							WritingArm_Home();
							ButtonLED_SetColor(0,1,0); // done: green flash
							SysTick_Wait(200 * T1ms);
							ButtonLED_SetColor(1,0,0); // still in write-mode (red)
						}
						idx = 0;
						UART0_OutString((uint8_t *)"Write> ");
					} else if(ch == BS || ch == DEL){
						if(idx){
							idx--;
							UART0_OutChar(BS); UART0_OutChar(SP); UART0_OutChar(BS);
						}
					} else if(ch >= 0x20 && idx < (sizeof(buf)-1)){
						buf[idx++] = ch;
						UART0_OutChar(ch);
					}
				}

				// small delay to avoid tight loop
				SysTick_Wait(5 * T1ms);
				// advance millisecond tick while in write-mode loop
				System_ms += 5;
			}
			// ensure we fall through and not process main commands while writing
			continue;
		}

		// Non-blocking UART poll: if a char is available, process it
		if((UART0_FR_R & UART_FR_RXFE) == 0){
			c = UART0_InChar();
			// echo and newline
			UART0_OutChar(c);
			UART0_NextLine();

			switch(c){
			case 'A': case 'a':{
				/* step ARM_CH up */
				armAngle += ANGLE_STEP;
				if(armAngle > 180) armAngle = 180;
				WritingArm_SetServoAngle(ARM_CH, armAngle);
			} break;

			case 'D': case 'd':{
				/* step ARM_CH down */
				armAngle -= ANGLE_STEP;
				if(armAngle < 0) armAngle = 0;
				WritingArm_SetServoAngle(ARM_CH, armAngle);
			} break;

			case 'W': case 'w':{
				/* step BASE_CH up */
				baseAngle += ANGLE_STEP;
				if(baseAngle > 180) baseAngle = 180;
				WritingArm_SetServoAngle(BASE_CH, baseAngle);
			} break;

			case 'S': case 's':{
				/* step BASE_CH down */
				baseAngle -= ANGLE_STEP;
				if(baseAngle < 0) baseAngle = 0;
				WritingArm_SetServoAngle(BASE_CH, baseAngle);
			} break;

			case 'J': case 'j':{
				/* step PEN_CH up */
				penAngle += ANGLE_STEP;
				if(penAngle > 180) penAngle = 180;
				WritingArm_SetServoAngle(PEN_CH, penAngle);
			} break;

			case 'K': case 'k':{
				/* step PEN_CH down */
				penAngle -= ANGLE_STEP;
				if(penAngle < 0) penAngle = 0;
				WritingArm_SetServoAngle(PEN_CH, penAngle);
			} break;

			case 'H': case 'h':{
				WritingArm_Home();
				/* keep local state in sync with home positions */
				baseAngle = 90;
				armAngle = 110;
				penAngle = 90;
				eraserAngle = 0;
				/* set CH8(TEST), ensure it's cleared when homing */
				PCA9685_SetPin(0x40, 8, 0, 0);
			} break;

			case 'T': case 't':{
				UART0_OutString((uint8_t *)"PCA9685 Test:\r\n");
				uint8_t mode = PCA9685_read8(0x40, PCA9685_MODE1);
				UART0_OutString((uint8_t *)"MODE1=0x"); print_hex8(mode); UART0_OutString((uint8_t *)"\r\n");
				uint8_t mode2 = PCA9685_read8(0x40, PCA9685_MODE2);
				UART0_OutString((uint8_t *)"MODE2=0x"); print_hex8(mode2); UART0_OutString((uint8_t *)"\r\n");
				uint8_t pres = PCA9685_readPrescale(0x40);
				UART0_OutString((uint8_t *)"PRESCALE=0x"); print_hex8(pres); UART0_OutString((uint8_t *)"\r\n");
				UART0_OutString((uint8_t *)"Setting CH8 to 50% (test)\r\n");
				PCA9685_SetPin(0x40, 8, 2048, 0);
				// wait a bit for I2C write to complete and scope to see signal
				SysTick_Wait(100 * T1ms);
				uint16_t onv = PCA9685_getPWM(0x40, 8, 0);
				UART0_OutString((uint8_t *)"CH8 ON read=0x"); print_hex16(onv); UART0_OutString((uint8_t *)"\r\n");
				uint16_t off = PCA9685_getPWM(0x40, 8, 1);
				UART0_OutString((uint8_t *)"CH8 OFF read=0x"); print_hex16(off); UART0_OutString((uint8_t *)"\r\n");
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

		// small sleep to allow other interrupts and avoid busy spin
		SysTick_Wait(5 * T1ms);
		// update millisecond tick (matches the 5ms sleep above)
		System_ms += 5;
	}

	// never reached
	return 0;
}

