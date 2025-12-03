#define LED (*((volatile unsigned long *)0x40025038))  // use onboard three LEDs: PF321
// Legacy Port E data alias (PE3..PE0). Kept for reference; not used with new PD2 mapping.

////////// Constants //////////  
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// white    RGB    0x0E
// pink     R-B    0x06
// Cran     -GB    0x0C

#define Dark    	0x00
#define Red     	0x02
#define Blue    	0x04
#define Green   	0x08
#define Yellow  	0x0A
#define Cran      0x0C
#define White   	0x0E
#define Purple  	0x06

// Constant definitions for motor direction control on PD3,PE2,PE1,PD2
// New Bit mapping (Dir_Write in motor.c):
//   bit3 -> PD3 (Left IN1)
//   bit2 -> PE2 (Left IN2)
//   bit1 -> PE1 (Right IN1)
//   bit0 -> PD2 (Right IN2)
// For each motor (L: PE3/PE2, R: PE1/PD2):
//   Forward  = IN1=1, IN2=0 per side  -> 0b1010 (0x0A)
//   Backward = IN1=0, IN2=1 per side  -> 0b0101 (0x05)
//   Pivot Left  (L back, R fwd)       -> 0b0110 (0x06)
//   Pivot Right (L fwd, R back)       -> 0b1001 (0x09)
// Note: The values below reflect the logical intentions. Any wiring swaps
// are handled in motor.c via SWAP_LEFT_DIR/SWAP_RIGHT_DIR.
// Standard logical direction encodings (do not change these):
// FORWARD:  0b1010 (0x0A)  L:IN1=1,IN2=0  R:IN1=1,IN2=0
// BACKWARD: 0b0101 (0x05)  L:IN1=0,IN2=1  R:IN1=0,IN2=1
// LEFTPIVOT:0b0110 (0x06)  L:backward      R:forward
// RIGHTPIVOT:0b1001 (0x09) L:forward       R:backward
#define BRAKE     0x00
//#define FORWARD   0x0A
//#define BACKWARD  0x05
//#define LEFTPIVOT 0x03
//#define RIGHTPIVOT 0x0C

#define FORWARD   0x0C
#define BACKWARD  0x03
#define LEFTPIVOT 0x05
#define RIGHTPIVOT 0x0A

// standard ASCII symbols
#define CR   0x0D
#define LF   0x0A

//////////////////////1. Declarations Section////////////////////////////////
////////// Function Prototypes //////////
// Initializes PE3-0 for L298N motor driver direction
void Car_Dir_Init(void);
void UART_Init(void);
unsigned char UART1_InChar(void);
void BLT_InString(unsigned char *bufPt);
void UART0_OutChar(unsigned char data);
void UART0_OutString(unsigned char *pt);
void PortF_Init(void);
/////////////////////////////////////////////////////////////////////////////
