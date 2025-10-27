/////////////////////////////////////////////////////////////////////////////
// Course Number: CECS 347
// Assignment: Example project for Hardware PWM controlled Car
// Description: 
/////////////////////////////////////////////////////////////////////////////

//////////////////////1. Pre-processor Directives Section////////////////////
#include "tm4c123gh6pm.h"
#include "GPIO.h"

/////////////////////////////////////////////////////////////////////////////

//////////////////////2. Declarations Section////////////////////////////////
////////// Constants //////////

////////// Local Global Variables //////////
/////////////////////////////////////////////////////////////////////////////

//////////////////////3. Subroutines Section/////////////////////////////////
// Dependency: None
// Inputs: None
// Outputs: None
// Description: Initializes PD2,1,0 and PC6 for use with L298N motor driver direction
void Car_Dir_Init(void){
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOD)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;	// Activate D clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOD)==0){};
	}
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOC)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC;	// Activate C clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOC)==0){};
	}
		
  GPIO_PORTD_AMSEL_R &= ~0x07;	// disable analog function on PD2-0
	GPIO_PORTD_AFSEL_R &= ~0x07;	// no alternate function on PD2-0
  GPIO_PORTD_PCTL_R &= ~0x00000FFF;	// GPIO clear bit PCTL for PD2-0
	GPIO_PORTD_DIR_R |= 0x07; // output on PD2-0
  GPIO_PORTD_DEN_R |= 0x07;	// enable digital I/O on PD2-0
	
	GPIO_PORTC_AMSEL_R &= ~0x40;	// disable analog function on PC6
	GPIO_PORTC_AFSEL_R &= ~0x40;	// no alternate function on PC6
	GPIO_PORTC_PCTL_R &= ~0x0F000000;	// GPIO clear bit PCTL for PC6
	GPIO_PORTC_DIR_R |= 0x40; // output on PC6
	GPIO_PORTC_DEN_R |= 0x40;	// enable digital I/O on PC6
}

// Port F Initialization
//void LED_Init(void){ 
//	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;	// Activate F clocks
//	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)==0){};
//		
//  GPIO_PORTF_AMSEL_R &= ~0x0E;      // 3) disable analog function
//  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL  
//  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL  
//  GPIO_PORTF_DIR_R |= 0x0E;         // 6) PF1-PF3 output
//  GPIO_PORTF_AFSEL_R &= ~0x0E;      // 7) no alternate function     
//  GPIO_PORTF_DEN_R |= 0x0E;         // 8) enable digital pins PF3-PF1
//  LED = Dark;                       // Turn off all LEDs.
//}

#ifdef GPIO_UART_ENABLED
//------------UART_Init------------
// Initialize the UART for 19200 baud rate (assuming 16 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART_Init(void){
	// Activate Clocks
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // activate port B
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
	
	
	UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 17;                    // IBRD = int(16,000,000 / (16 * 57600)) = int(17.3611111)
  UART0_FBRD_R = 23;                     // FBRD = round(3611111 * 64) = 27
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= 0x301;                 // enable UART for both Rx and Tx

  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1,PA0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1,PA0
                                        // configure PA1,PA0 as UART0
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA1,PA0
	
  UART1_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
	
	// Data Communication Mode, Buad Rate = 57600
  UART1_IBRD_R = 17;                    // IBRD = int(16,000,000 / (16 * 57600)) = int(17.3611111)
  UART1_FBRD_R = 23;                     // FBRD = round(3611111 * 64) = 27
	
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART1_CTL_R |= 0x301;                 // enable UART for both Rx and Tx
  
  GPIO_PORTB_AFSEL_R |= 0x03;           // enable alt funct on PB1,PB0
  GPIO_PORTB_DEN_R |= 0x03;             // enable digital I/O on PB1,PB0
                                        // configure PB1,PB0 as UART1
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTB_AMSEL_R &= ~0x03;          // disable analog functionality on PB1,PB0

}

//------------UART0_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART0_OutChar(unsigned char data){
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}

//------------UART0_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART0_OutString(unsigned char *pt){
  while(*pt){
    UART0_OutChar(*pt);
    pt++;
  }
}

//------------UART1_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART1_InChar(void){
  while((UART1_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART1_DR_R&0xFF));
}

// This function reads response from HC-05 Bluetooth module.
void BLT_InString(unsigned char *bufPt) {
  unsigned char length=0;
  bufPt[length] = UART1_InChar();
  
  // Two possible endings for a reply from HC-05: OK\r\n, ERROR:(0)\r\n
  while (bufPt[length]!=LF) {
    length++;
    bufPt[length] = UART1_InChar();
  };
    
  // add null terminator
  length++;
  bufPt[length] = 0;
}
#endif // GPIO_UART_ENABLED

// Port F Initialization
void PortF_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0
  GPIO_PORTF_CR_R |= 0x1F;          // allow changes to PF4-PF0
  GPIO_PORTF_AMSEL_R &= ~0x1F;      // 3) disable analog on PF4-PF0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // 4) PCTL GPIO on PF4-PF0
  GPIO_PORTF_DIR_R |= 0x0E;         // 5) PF3-1 outputs (LEDs), PF4/PF0 inputs (switches)
  GPIO_PORTF_DIR_R &= ~0x11;        // make PF4,PF0 inputs
  GPIO_PORTF_AFSEL_R &= ~0x1F;      // 6) disable alt funct on PF4-PF0
  GPIO_PORTF_DEN_R |= 0x1F;         // 7) enable digital I/O on PF4-PF0
  GPIO_PORTF_PUR_R |= 0x11;         // 8) enable internal pull-up on PF4,PF0 (negative logic switches)
}


/////////////////////////////////////////////////////////////////////////////
