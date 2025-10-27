#define LED (*((volatile unsigned long *)0x40025038))  // use onboard three LEDs: PF321
#define DIRECTION_E (*((volatile unsigned long *)0x4002403C)) // PE3,2,1,0 for motor direction

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

// Constant definitions for motor direction control on PE3-0
#define BACKWARD 0x0A
#define FORWARD 0x05
#define LEFTPIVOT 0x09
#define RIGHTPIVOT 0x06
#define BRAKE 0x00

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
