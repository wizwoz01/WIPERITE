#include "PWM.h"
#include <stdint.h>
#include "tm4c123gh6pm.h"

// Initializes PWM1 for PD0 (M1PWM4) and PD1 (M1PWM5)
void PWM_Init(void){
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;     // 1) activate PWM1
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;   // 2) activate port D
  delay = SYSCTL_RCGCGPIO_R;               // allow time for clock to stabilize

  // Configure PD0, PD1 for PWM
  GPIO_PORTD_AFSEL_R |= 0x03;           // enable alt funct on PD0, PD1
  GPIO_PORTD_PCTL_R &= ~0x000000FF;     // clear PCTL
  GPIO_PORTD_PCTL_R |= 0x00000055;     // configure PD0, PD1 as PWM
  GPIO_PORTD_AMSEL_R &= ~0x03;          // disable analog functionality
  GPIO_PORTD_DEN_R |= 0x03;             // enable digital I/O

  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider (50MHz/2 = 25MHz)

  // Configure PWM1 Generator 2 for PD0 (Left Motor, M1PWM4) and PD1 (Right Motor, M1PWM5)
  PWM1_2_CTL_R = 0;                      // disable generator during setup
  PWM1_2_GENA_R = 0x000000C8;            // low on LOAD, high on CMPA down
  PWM1_2_GENB_R = 0x00000C08;            // low on LOAD, high on CMPB down
  PWM1_2_LOAD_R = PERIOD - 1;            // set period
  PWM1_2_CMPA_R = 0;                     // set initial duty to 0 (PD0)
  PWM1_2_CMPB_R = 0;                     // set initial duty to 0 (PD1)
  PWM1_2_CTL_R |= 0x00000001;            // enable generator

  // Enable PWM1 outputs 4 and 5 (PD0=M1PWM4, PD1=M1PWM5)
  PWM1_ENABLE_R |= (PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN);
}

// Set duty cycle for both motors
void PWM_Duty(uint16_t duty_L, uint16_t duty_R){
  PWM1_2_CMPA_R = duty_L > 0 ? duty_L - 1 : 0;
  PWM1_2_CMPB_R = duty_R > 0 ? duty_R - 1 : 0;
}

// Set duty cycle for Left motor (PD0)
void PWM_PD0_Duty(uint16_t duty_L){
  PWM1_2_CMPA_R = duty_L > 0 ? duty_L - 1 : 0;
}

// Set duty cycle for Right motor (PD1)
void PWM_PD1_Duty(uint16_t duty_R){
  PWM1_2_CMPB_R = duty_R > 0 ? duty_R - 1 : 0;
}
