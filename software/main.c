#include <stdint.h>
#include "tm4c123gh6pm.h"

static void delay(void){
  volatile uint32_t i;
  for(i = 0; i < 3000000; i++); // ~0.2–0.3s at default clock
}

int main(void){
  // Clocks: Port B (motors) and Port F (LED)
  SYSCTL_RCGCGPIO_R |= (1<<1) | (1<<5);         // B, F
  while((SYSCTL_PRGPIO_R & (1<<1)) == 0){}      // wait B
  while((SYSCTL_PRGPIO_R & (1<<5)) == 0){}      // wait F

  // PB2..PB7 as GPIO outputs (DIR+EN)
  GPIO_PORTB_DIR_R |= 0xFC;                     // PB2..PB7 outputs
  GPIO_PORTB_DEN_R |= 0xFC;                     // digital enable

  // PF1 (red LED) output
  GPIO_PORTF_DIR_R |= (1<<1);
  GPIO_PORTF_DEN_R |= (1<<1);

  // ----- Enable both H-bridges via ENA/ENB -----
  // If EN jumpers are ON, this does no harm.
  // If jumpers are OFF, this actively enables the bridges.
  GPIO_PORTB_DATA_R |= (1<<6) | (1<<7);         // PB6=1 (ENA), PB7=1 (ENB)

  while(1){
    // LED ON (loop heartbeat)
    GPIO_PORTF_DATA_R |= (1<<1);

    // Forward: Left(IN1=1,IN2=0)=PB4=1,PB5=0; Right(IN3=1,IN4=0)=PB2=1,PB3=0
    GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~0x3C) | (1<<4) | (1<<2);
    delay();

    // Stop (coast): INx = 0,0
    GPIO_PORTB_DATA_R &= ~0x3C;
    delay();

    // Reverse: Left(IN1=0,IN2=1)=PB4=0,PB5=1; Right(IN3=0,IN4=1)=PB2=0,PB3=1
    GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~0x3C) | (1<<5) | (1<<3);
    delay();

    // Stop (coast)
    GPIO_PORTB_DATA_R &= ~0x3C;
    delay();

    // LED OFF
    GPIO_PORTF_DATA_R &= ~(1<<1);
  }
}
