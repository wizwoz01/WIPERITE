#include <stdint.h>
#include "tm4c123gh6pm.h"

#define MOTOR_DIR_MASK 0x3C  // PB2,3,4,5
//ENA PB6 PWM Left
//ENB PB7 PWM Right
//IN1 PB4 Left DIR
//IN2 PB5 (sleep -> HIGH)
//IN3 PB2 Right DIR
//IN4 PB3 (sleep -> HIGH)

void forward(void);
void reverse(void);
void right_pivot(void);
void left_pivot(void);
void stop(void);

static void delay(void){
  volatile uint32_t i;
  for(i = 0; i < 3000000; i++);
}

int main(void){
	// ----- Unlock and Enable Port F -----
	SYSCTL_RCGCGPIO_R |= (1<<5);
	while((SYSCTL_PRGPIO_R & (1<<5)) == 0){}

	// Unlock PF0
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
	GPIO_PORTF_CR_R |= 0x1F; // allow changes to PF0–PF4

	// PF1 = LED, PF4 = SW1
	GPIO_PORTF_DIR_R |=  (1<<1);      // PF1 output
	GPIO_PORTF_DIR_R &= ~(1<<4);      // PF4 input
	GPIO_PORTF_DEN_R |=  (1<<1) | (1<<4); // digital enable PF1, PF4
	GPIO_PORTF_PUR_R |=  (1<<4);      // enable pull-up on PF4 (SW1)

	GPIO_PORTF_DATA_R &= ~(1<<1);     // LED off initially


  SYSCTL_RCGCGPIO_R |= (1<<1) | (1<<5);
  while((SYSCTL_PRGPIO_R & (1<<1)) == 0){}
  while((SYSCTL_PRGPIO_R & (1<<5)) == 0){}

  GPIO_PORTB_DIR_R |= 0xFC;
  GPIO_PORTB_DEN_R |= 0xFC;

  GPIO_PORTF_DIR_R |= (1<<1);
  GPIO_PORTF_DEN_R |= (1<<1);

	// ---- Wait for SW1 press (PF4 == 0) ----
	while(GPIO_PORTF_DATA_R & (1<<4)){   // while PF4 is HIGH (not pressed)
			GPIO_PORTF_DATA_R ^= (1<<1);     // blink LED while waiting
			for(volatile int i = 0; i < 500000; i++); // small delay
	}
	GPIO_PORTF_DATA_R &= ~(1<<1); // LED off when starting

  // Enable both bridges (PB6, PB7 high)
  GPIO_PORTB_DATA_R |= (1<<6) | (1<<7);

	while(1){
    GPIO_PORTF_DATA_R ^= (1<<1); // toggle LED each step
    forward(); 
		delay();

    GPIO_PORTF_DATA_R ^= (1<<1);
    stop(); 
		delay();

    GPIO_PORTF_DATA_R ^= (1<<1);
    reverse(); 
		delay();

    GPIO_PORTF_DATA_R ^= (1<<1);
    stop(); 
		delay();

    GPIO_PORTF_DATA_R ^= (1<<1);
    right_pivot(); 
		delay();

    GPIO_PORTF_DATA_R ^= (1<<1);
    stop(); 
		delay();

    GPIO_PORTF_DATA_R ^= (1<<1);
    left_pivot(); 
		delay();

    GPIO_PORTF_DATA_R ^= (1<<1);
    stop(); 
		delay();
	}

}

void forward(void){
	// Forward: Left(IN1=1,IN2=0)=PB4=1,PB5=0; Right(IN3=1,IN4=0)=PB2=1,PB3=0
  GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~MOTOR_DIR_MASK) | ((1<<4) | (1<<2));
}

void reverse(void){
	// Reverse: Left(IN1=0,IN2=1)=PB4=0,PB5=1; Right(IN3=0,IN4=1)=PB2=0,PB3=1
  GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~MOTOR_DIR_MASK) | ((1<<5) | (1<<3));
}

void right_pivot(void){
  GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~MOTOR_DIR_MASK) | ((1<<4) | (1<<3));
}

void left_pivot(void){
  GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~MOTOR_DIR_MASK) | ((1<<5) | (1<<2));
}

void stop(void){
  GPIO_PORTB_DATA_R &= ~MOTOR_DIR_MASK;
}


