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
 *  - PD0 as M1PWM0 (Left Motor, Generator 0 CMPA/GENA)
 *  - PD1 as M1PWM1 (Right Motor, Generator 0 CMPB/GENB)
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

/**
 * @brief Explicitly enable both PWM outputs (M1PWM0/1). Normally not required
 *        because PWM_Duty auto-enables per channel when duty > 0.
 */
void PWM_EnableOutputs(void);

/**
 * @brief Explicitly disable both PWM outputs (M1PWM0/1) regardless of duty.
 *        Use to guarantee no switching on the motor driver when idle.
 */
void PWM_DisableOutputs(void);

/**
 * @brief Configure PD0/PD1 pins to alternate-function PWM mode.
 * Call before enabling or driving PWM to ensure proper pin muxing.
 */
void PWM_PinsToPWM(void);

/**
 * @brief Place PD0/PD1 pins into high-impedance GPIO state (digital disabled).
 * Use when motors are disabled to minimize coupling into the motor driver.
 */
void PWM_PinsToHiZ(void);

/**
 * @brief Drive PD0/PD1 as GPIO outputs low (safe idle for motor PWMs).
 * Can be called before PWM_Init; will enable Port D clock if needed.
 */
void PWM_PinsToIdleLow(void);

// Optional: swap left/right PWM channels at runtime for wiring mismatches
void PWM_SetSwapLR(uint8_t enable);
