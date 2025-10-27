#include <stdint.h>

// This file was modified to support the motor driver on PD0 and PD1
// to avoid conflicts with the CC3100 WiFi BoosterPack.
// System clock is 50MHz, PWM clock is 25MHz (50MHz/2)
// Target frequency: 1000 Hz
// Period = 25,000,000 Hz / 1000 Hz = 25000
#define PERIOD 25000
#define SPEED_50  (PERIOD/2)
#define SPEED_35	(PERIOD/3)
#define SPEED_20  (PERIOD/5)
#define SPEED_10 	(PERIOD/10)

/**
 * @brief Initializes PWM Module 1 for motor control on PD0 and PD1.
 * 
 * This function configures PWM1 Generator 2 for both channels:
 *  - PD0 as M1PWM4 (Left Motor, Generator 2 CMPA/GENA)
 *  - PD1 as M1PWM5 (Right Motor, Generator 2 CMPB/GENB)
 * This setup avoids using pins PB4, PB5, PE4, PE5 which may conflict
 * with the CC3100 BoosterPack.
 */
void PWM_Init(void);

/**
 * @brief Sets the duty cycle for both the left and right motors.
 * @param duty_L The duty cycle for the left motor (PD0).
 * @param duty_R The duty cycle for the right motor (PD1).
 */
void PWM_Duty(uint16_t duty_L, uint16_t duty_R);

/**
 * @brief Sets the duty cycle for the left motor (PD0).
 * @param duty_L The duty cycle value.
 */
void PWM_PD0_Duty(uint16_t duty_L);

/**
 * @brief Sets the duty cycle for the right motor (PD1).
 * @param duty_R The duty cycle value.
 */
void PWM_PD1_Duty(uint16_t duty_R);
