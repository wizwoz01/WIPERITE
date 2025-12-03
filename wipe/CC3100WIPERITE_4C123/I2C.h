/*
 * I2C.h
 *
 *	Provides the I2C Init, Read, and Write Function
 *
 * Created on: May 24th, 2023
 *		Author: Jackie Huynh
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "util.h"

/* List of Fill In Macros */

/*
 * Clocks
 *  - I2C1 clock: SYSCTL_RCGCI2C_R bit 1
 *  - GPIOA clock: SYSCTL_RCGCGPIO_R bit 0
 * GPIOA pins for I2C1:
 *  - PA6 -> I2C1SCL
 *  - PA7 -> I2C1SDA (open-drain)
 * Alternate function (GPIOPCTL) uses mux value 3 for both PA6/PA7.
 */

//Init Function
#define EN_I2C1_CLOCK            do{ SYSCTL_RCGCI2C_R |= 0x02; }while(0)
#define EN_GPIOA_CLOCK           do{ SYSCTL_RCGCGPIO_R |= 0x01; }while(0)
#define I2C1_PINS                (0xC0)                    /* PA7|PA6 */
#define I2C1_ALT_FUNC_MSK        (0xFF000000)              /* PCTL mask for PA6/PA7 */
#define I2C1_ALT_FUNC_SET        (0x33000000)              /* PA6/PA7 -> I2C */
#define I2C1_SDA_PIN             (0x80)                    /* PA7 */
#define I2C1_SCL_PIN             (0x40)                    /* PA6 */
#define EN_I2C1_MASTER           do{ I2C1_MCR_R = 0x10; }while(0) /* MFE bit */
#define I2C_MTPR_TPR_VALUE       (7)                      /* ~100kHz @ 16MHz sysclk */
#define I2C_MTPR_STD_SPEED       (0)                      /* HS=0 for standard/fast mode */

//Transmit/Receive helpers
#define I2C1_RW_PIN              (0x01)                    /* MSA[0] R(1)/W(0) */

//Burst Transmit Function
#define RUN_CMD                  (I2C_MCS_RUN)

/*
 *	-------------------I2C0_Init------------------
 *	Basic I2C Initialization function for master mode @ 100kHz
 *	Input: None
 *	Output: None
 */
void I2C1_Init(void);

/*
 *	-------------------I2C0_Receive------------------
 *	Polls to receive data from specified peripheral
 *	Input: Slave address & Slave Register Address
 *	Output: Returns 8-bit data that has been received
 */
uint8_t I2C1_Receive(uint8_t slave_addr, uint8_t slave_reg_addr);

/*
 *	-------------------I2C0_Transmit------------------
 *	Transmit a byte of data to specified peripheral
 *	Input: Slave address, Slave Register Address, Data to Transmit
 *	Output: Any Errors if detected, otherwise 0
 */
uint8_t I2C1_Transmit(uint8_t slave_addr, uint8_t slave_reg_addr, uint8_t data);

//Has Yet to be Implemented
/*
 *	----------------I2C0_Burst_Receive-----------------
 *	Polls to receive multiple bytes of data from specified
 *  peripheral by incrementing starting slave register address
 *	Input: Slave address, Slave Register Address, Data Buffer
 *	Output: None
 */
void I2C1_Burst_Receive(uint8_t slave_addr, uint8_t slave_reg_addr, uint8_t* data, uint32_t size);

/*
 *	----------------I2C0_Burst_Transmit-----------------
 *	Transmit multiple bytes of data to specified peripheral
 *  by incrementing starting slave address
 *	Input: Slave address, Slave Register Address, Data Buffer to transmit, Size of Transmit
 *	Output: None
 */
uint8_t I2C1_Burst_Transmit(uint8_t slave_addr, uint8_t slave_reg_addr, uint8_t* data, uint32_t size);

#endif //I2C_H_