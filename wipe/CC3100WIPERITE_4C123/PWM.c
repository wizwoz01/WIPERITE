#include "PWM.h"
#include <stdint.h>
#include "tm4c123gh6pm.h"

static volatile uint8_t s_pwm_inited = 0;

// Initializes PWM1 for PD0 (M1PWM0) and PD1 (M1PWM1)
void PWM_Init(void){
  if(s_pwm_inited){
    // Ensure outputs remain disabled on repeated init
    PWM1_ENABLE_R &= ~(PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN);
    return;
  }
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;     // 1) activate PWM1
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;   // 2) activate port D
  delay = SYSCTL_RCGCGPIO_R;               // allow time for clock to stabilize

  // Configure PD0/PD1 to PWM alternate function (M1PWM0/M1PWM1)
  GPIO_PORTD_AFSEL_R |= 0x03;           // alt function on PD0, PD1
  GPIO_PORTD_PCTL_R  &= ~0x000000FF;    // clear
  GPIO_PORTD_PCTL_R  |=  0x00000055;    // PD0->M1PWM0, PD1->M1PWM1
  GPIO_PORTD_AMSEL_R &= ~0x03;          // disable analog
  GPIO_PORTD_DEN_R   |=  0x03;          // digital enable

  // 3) Use PWM divider: system clock 50MHz -> PWM clock 25MHz (divide by 2)
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // enable PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; // set divider to /2 (value 0)

  // Configure PWM1 Generator 0 for PD0 (Left Motor, M1PWM0) and PD1 (Right Motor, M1PWM1)
  PWM1_0_CTL_R = 0;                      // disable generator during setup
  PWM1_0_GENA_R = 0x000000C8;            // low on LOAD, high on CMPA down
  PWM1_0_GENB_R = 0x00000C08;            // low on LOAD, high on CMPB down
  PWM1_0_LOAD_R = PERIOD - 1;            // set period
  PWM1_0_CMPA_R = 0;                     // set initial duty to 0 (PD0)
  PWM1_0_CMPB_R = 0;                     // set initial duty to 0 (PD1)
  PWM1_0_CTL_R |= 0x00000001;            // enable generator
  // Keep outputs disabled initially to avoid any idle switching glitches.
  PWM1_ENABLE_R &= ~(PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN);
  s_pwm_inited = 1;
}

static volatile uint8_t s_swapLR = 0; // 0: normal (PD0=Left, PD1=Right); 1: swapped

// Set duty cycle for both motors
void PWM_Duty(uint16_t duty_L, uint16_t duty_R){
  uint16_t a = s_swapLR ? duty_R : duty_L;
  uint16_t b = s_swapLR ? duty_L : duty_R;
  // Channel A (PD0/M1PWM0)
  if(a > 0){
    PWM1_0_CMPA_R = a - 1;
    PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN;   // enable output when needed
  } else {
    PWM1_0_CMPA_R = 0;
    PWM1_ENABLE_R &= ~PWM_ENABLE_PWM0EN;  // disable output when 0% duty
  }

  // Channel B (PD1/M1PWM1)
  if(b > 0){
    PWM1_0_CMPB_R = b - 1;
    PWM1_ENABLE_R |= PWM_ENABLE_PWM1EN;   // enable output when needed
  } else {
    PWM1_0_CMPB_R = 0;
    PWM1_ENABLE_R &= ~PWM_ENABLE_PWM1EN;  // disable output when 0% duty
  }
}

// Set duty cycle for Left motor (PD0)
void PWM_PD0_Duty(uint16_t duty_L){
  if(s_swapLR){
    // route to PD1 when swapped
    if(duty_L > 0){
      PWM1_0_CMPB_R = duty_L - 1;
      PWM1_ENABLE_R |= PWM_ENABLE_PWM1EN;
    } else {
      PWM1_0_CMPB_R = 0;
      PWM1_ENABLE_R &= ~PWM_ENABLE_PWM1EN;
    }
  } else {
    if(duty_L > 0){
      PWM1_0_CMPA_R = duty_L - 1;
      PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN;
    } else {
      PWM1_0_CMPA_R = 0;
      PWM1_ENABLE_R &= ~PWM_ENABLE_PWM0EN;
    }
  }
}

// Set duty cycle for Right motor (PD1)
void PWM_PD1_Duty(uint16_t duty_R){
  if(s_swapLR){
    // route to PD0 when swapped
    if(duty_R > 0){
      PWM1_0_CMPA_R = duty_R - 1;
      PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN;
    } else {
      PWM1_0_CMPA_R = 0;
      PWM1_ENABLE_R &= ~PWM_ENABLE_PWM0EN;
    }
  } else {
    if(duty_R > 0){
      PWM1_0_CMPB_R = duty_R - 1;
      PWM1_ENABLE_R |= PWM_ENABLE_PWM1EN;
    } else {
      PWM1_0_CMPB_R = 0;
      PWM1_ENABLE_R &= ~PWM_ENABLE_PWM1EN;
    }
  }
}

void PWM_EnableOutputs(void){
  PWM1_ENABLE_R |= (PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN);
}

void PWM_DisableOutputs(void){
  PWM1_ENABLE_R &= ~(PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN);
}

void PWM_PinsToPWM(void){
  // Configure PD0, PD1 to alternate function PWM (M1PWM0/M1PWM1)
  GPIO_PORTD_AFSEL_R |= 0x03;           // alt function on PD0, PD1
  GPIO_PORTD_PCTL_R  &= ~0x000000FF;    // clear
  GPIO_PORTD_PCTL_R  |=  0x00000055;    // PD0->M1PWM0, PD1->M1PWM1
  GPIO_PORTD_AMSEL_R &= ~0x03;          // disable analog
  GPIO_PORTD_DEN_R   |=  0x03;          // digital enable
}

void PWM_PinsToHiZ(void){
  // Ensure PWM outputs are disabled, then tri-state the pins
  PWM_DisableOutputs();
  GPIO_PORTD_AFSEL_R &= ~0x03;          // GPIO mode
  GPIO_PORTD_PCTL_R  &= ~0x000000FF;    // clear PCTL
  GPIO_PORTD_DEN_R   &= ~0x03;          // digital disabled -> Hi-Z
  GPIO_PORTD_DIR_R   &= ~0x03;          // input
  GPIO_PORTD_AMSEL_R &= ~0x03;          // analog disabled (leave as digital Hi-Z)
}

void PWM_PinsToIdleLow(void){
  // Ensure Port D clock is enabled
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
  volatile unsigned long delay = SYSCTL_RCGCGPIO_R; (void)delay;
  // Drive PD0/PD1 low as GPIO outputs
  PWM_DisableOutputs();
  GPIO_PORTD_AFSEL_R &= ~0x03;          // GPIO mode
  GPIO_PORTD_PCTL_R  &= ~0x000000FF;    // clear PCTL
  GPIO_PORTD_AMSEL_R &= ~0x03;          // disable analog
  GPIO_PORTD_DIR_R   |=  0x03;          // outputs
  GPIO_PORTD_DEN_R   |=  0x03;          // digital enable
  GPIO_PORTD_DATA_R  &= ~0x03;          // drive low
}

void PWM_SetSwapLR(uint8_t enable){
  s_swapLR = (enable != 0) ? 1 : 0;
}
