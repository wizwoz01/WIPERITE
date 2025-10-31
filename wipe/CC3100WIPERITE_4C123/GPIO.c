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
// Description: Initializes motor direction pins avoiding CC3100 conflicts.
// New mapping (to keep PE0 free for CC3100 SPI CS):
//   Direction bit3 -> PE3
//   Direction bit2 -> PE2
//   Direction bit1 -> PE1
//   Direction bit0 -> PD2  (was PE0)
// Notes:
//   - PE0 is left as input (no drive) to avoid contention with CC3100.
//   - Update motor.c to write using helper that targets PE1-3 and PD2.
void Car_Dir_Init(void){
    volatile unsigned long delay;
    // Enable Port E and Port D clocks
    SYSCTL_RCGC2_R |= (SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOD);
    delay = SYSCTL_RCGC2_R; (void)delay;

    // Configure PE1-PE3 as digital outputs, leave PE0 as input (for CC3100 CS)
    GPIO_PORTE_AMSEL_R &= ~0x0E;      // disable analog on PE3-PE1
    GPIO_PORTE_AFSEL_R &= ~0x0E;      // GPIO on PE3-PE1
    GPIO_PORTE_PCTL_R  &= ~0x0000FFF0;// clear PCTL for PE3-PE1
    GPIO_PORTE_DIR_R   |=  0x0E;      // outputs on PE3-PE1
    GPIO_PORTE_DEN_R   |=  0x0E;      // enable digital on PE3-PE1
    // Ensure PE0 is digital input (no drive)
    GPIO_PORTE_DEN_R   |=  0x01;      // digital enable PE0
    GPIO_PORTE_DIR_R   &= ~0x01;      // input on PE0
    GPIO_PORTE_AFSEL_R &= ~0x01;      // GPIO on PE0
    GPIO_PORTE_AMSEL_R &= ~0x01;      // disable analog on PE0

  // Configure PD2 as digital output for direction bit0 (was PD3)
  // PD2 mask = 0x04, PCTL bits for PD2 are bits 8..11 (0x00000F00)
  GPIO_PORTD_AMSEL_R &= ~0x04;      // disable analog on PD2
  GPIO_PORTD_AFSEL_R &= ~0x04;      // GPIO on PD2
  GPIO_PORTD_PCTL_R  &= ~0x00000F00;// clear PCTL for PD2
  GPIO_PORTD_DIR_R   |=  0x04;      // output on PD2
  GPIO_PORTD_DEN_R   |=  0x04;      // enable digital on PD2
}


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
