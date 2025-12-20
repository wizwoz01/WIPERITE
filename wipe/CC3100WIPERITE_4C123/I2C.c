/*
 * I2C.h
 *
 * Description: Main implementation of the I2C Init, Read, and Write Function
 * Authors: Ricardo C. 12-02-2025
 * Team 9
 *
 */

#include "I2C.h"
#include "tm4c123gh6pm.h"


/*
 *	-------------------I2C1_Init------------------
 *	Basic I2C Initialization function for master mode @ 100kHz
 *	Input: None
 *	Output: None
 */
void I2C1_Init(void){
	volatile uint32_t delay;

	/* Enable Required System Clock */
	//Enable I2C1 System Clock
	//Enable GPIOA System Clock
	EN_I2C1_CLOCK;
	EN_GPIOA_CLOCK;

	// Wait until I2C1 peripheral and GPIOA are ready
	while((SYSCTL_PRI2C_R & 0x02) == 0){};

	//Wait Until GPIOx System Clock is enabled
	while((SYSCTL_PRGPIO_R & 0x01) == 0){};

	/* GPIOA I2C Alternate Function Setup	*/
	//Disable Analog Mode first
	GPIO_PORTA_AMSEL_R &= ~I2C1_PINS;
	//Enable Alternate Function Selection
	GPIO_PORTA_AFSEL_R |= I2C1_PINS;
	//Select I2C1 as the alternate function (PA6=SCL, PA7=SDA use mux 3)
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~I2C1_ALT_FUNC_MSK) | I2C1_ALT_FUNC_SET;
	//Enable Open Drain for SDA pin (required for I2C)
	GPIO_PORTA_ODR_R  |= I2C1_SDA_PIN;
	//Enable internal pull-ups in case external pull-ups are absent
	GPIO_PORTA_PUR_R   |= I2C1_PINS;
	//Enable Digital I/O
	GPIO_PORTA_DEN_R   |= I2C1_PINS;
	
	// Note: Do NOT set DIR register bits when using alternate function
	// The I2C peripheral controls pin direction automatically

	/*	I2C1 Setup as Master Mode @ 100kBits	*/
	//Configure I2C1 as Master 
	EN_I2C1_MASTER;

	/* Configuring I2C Clock Frequency to 100KHz

		TPR = (System Clock / (2*(SCL_LP + SCL_HP) * SCL_CLK)) - 1
		SCL_LP and SCL_HP are fixed
		SCL_LP = 6 & SCL_HP = 4

		Example if we want to configure I2C speed to 100kHz for 40MHz system clock
		TPR = (40MHz / ((2*(6+4)) * 100kHz)) - 1 		(Convert Everything to Hz)
		TPR = 19

	*/

	// take care of master timer period: standard speed and TPR value	
	I2C1_MTPR_R = (I2C_MTPR_STD_SPEED) | (I2C_MTPR_TPR_VALUE & 0x7F);
	
	// Small delay to let I2C bus stabilize
	for(delay = 0; delay < 10000; delay++){}
}

/*
 *	-------------------I2C1_ProbeStatus------------------
 *	Debug function: Returns the raw MCS status after attempting to address a slave
 *	Input: Slave address (7-bit)
 *	Output: Raw MCS register value (for debugging)
 */
uint32_t I2C1_ProbeStatus(uint8_t slave_addr){
	volatile uint32_t delay;
	uint32_t status;
	
	/* Wait for I2C1 to be idle */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	
	/* Try to address the slave in write mode */
	I2C1_MSA_R = (slave_addr << 1) & 0xFE;
	I2C1_MDR_R = 0x00;  // Try to read register 0
	
	/* Generate START and send address */
	I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
	
	/* Wait for completion */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	
	/* Capture status */
	status = I2C1_MCS_R;
	
	/* Send STOP to release bus */
	I2C1_MCS_R = I2C_MCS_STOP;
	for(delay = 0; delay < 1000; delay++){}
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	
	return status;
}

/*
 *	-------------------I2C1_Receive------------------
 *	Polls to receive data from specified peripheral
 *	Input: Slave address & Slave Register Address
 *	Output: Returns 8-bit data that has been received (0xFF on error)
 */
uint8_t I2C1_Receive(uint8_t slave_addr, uint8_t slave_reg_addr){
	volatile uint32_t delay;
	uint32_t error;
	uint8_t data;

	/* Wait for I2C1 to be idle and bus to be free */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	for(delay = 0; delay < 500; delay++){}

	/*=== Phase 1: Send slave address (write) + register address ===*/
	I2C1_MSA_R = (slave_addr << 1) & 0xFE;  // write mode
	I2C1_MDR_R = slave_reg_addr;

	/* Single byte write: START + RUN + STOP (complete transaction) */
	I2C1_MCS_R = 0x07;  // START=1, RUN=1, STOP=1

	/* Wait for completion */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	for(delay = 0; delay < 500; delay++){}

	/* Check for errors */
	error = I2C1_MCS_R & 0x0E;  // ERROR, ADRACK, DATACK
	if(error){
		return 0xFF;
	}

	/* Wait for bus to be completely free */
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};
	for(delay = 0; delay < 1000; delay++){}

	/*=== Phase 2: Read one byte from slave ===*/
	I2C1_MSA_R = (slave_addr << 1) | 0x01;  // read mode

	/* Single byte read: START + RUN + STOP */
	I2C1_MCS_R = 0x07;  // START=1, RUN=1, STOP=1

	/* Wait for completion */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	for(delay = 0; delay < 500; delay++){}

	/* Read data from MDR */
	data = (uint8_t)(I2C1_MDR_R & 0xFF);

	/* Check for errors (ignore DATACK on read - NACK is normal for last byte) */
	error = I2C1_MCS_R & 0x06;  // ERROR, ADRACK only
	if(error){
		return 0xFF;
	}

	/* Wait for bus to be free */
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};
	for(delay = 0; delay < 500; delay++){}

	return data;

}

/*
 *	-------------------I2C1_Transmit------------------
 *	Transmit a byte of data to specified peripheral
 *	Input: Slave address, Slave Register Address, Data to Transmit
 *	Output: Any Errors if detected, otherwise 0
 */
uint8_t I2C1_Transmit(uint8_t slave_addr, uint8_t slave_reg_addr, uint8_t data){
	volatile uint32_t delay;
	uint32_t error;

	/* Wait for I2C1 to be idle and bus to be free */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};
	for(delay = 0; delay < 1000; delay++){}

	/* Configure I2C Slave Address for write mode */
	I2C1_MSA_R = (slave_addr << 1) & 0xFE;
	/* Load register address to transmit */
	I2C1_MDR_R = slave_reg_addr;

	/* Initiate I2C by generating START and RUN (keep bus, no STOP) */
	I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;

	/* Wait until write has been completed */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	for(delay = 0; delay < 500; delay++){}

	/* Check first-cycle errors (address + register byte) */
	error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
	if(error){
		I2C1_MCS_R = I2C_MCS_STOP;
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		for(delay = 0; delay < 1000; delay++){}
		return (uint8_t)error;
	}

	/* Load data byte to transmit */
	I2C1_MDR_R = data;

	/* Generate RUN + STOP to send data and complete transmission */
	I2C1_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;

	/* Wait until write has been completed */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Wait until bus is free */
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};

	/* Delay between transactions */
	for(delay = 0; delay < 1000; delay++){}

	/* Check for any error */
	return (uint8_t)(I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO));
}

//Has Yet to be Implemented
/*
 *	----------------I2C1_Burst_Receive-----------------
 *	Polls to receive multiple bytes of data from specified
 *  peripheral by incrementing starting slave register address
 *	Input: Slave address, Slave Register Address, Data Buffer
 *	Output: None
 */
void I2C1_Burst_Receive(uint8_t slave_addr, uint8_t slave_reg_addr, uint8_t* data, uint32_t size){
	(void)slave_addr; (void)slave_reg_addr; (void)data; (void)size;
	/* TODO: Implement as needed */
}

/*
 *	----------------I2C1_Burst_Transmit-----------------
 *	Transmit multiple bytes of data to specified peripheral
 *  by incrementing starting slave address
 *	Input: Slave address, Slave Register Address, Data Buffer to transmit
 *	Output: None
 */
uint8_t I2C1_Burst_Transmit(uint8_t slave_addr, uint8_t slave_reg_addr, uint8_t* data, uint32_t size){
	volatile uint32_t delay;
	uint32_t error;

	/* Validate parameters */
	if((data == 0) || (size == 0)){
		return 1;
	}

	/* Wait for I2C1 to be idle and bus to be free */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};

	/* Configure I2C Slave Address for write mode */
	I2C1_MSA_R = (slave_addr << 1) & 0xFE;
	/* Load register address to transmit */
	I2C1_MDR_R = slave_reg_addr;

	/* Initiate I2C by generating START and RUN (keep transaction open) */
	I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;

	/* Wait until write has been completed */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Check for any error on the register address write */
	error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
	if(error){
		I2C1_MCS_R = I2C_MCS_STOP;
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		for(delay = 0; delay < 1000; delay++){}
		return (uint8_t)error;
	}

	/* Loop to Burst Transmit data bytes */
	while(size > 1){
		I2C1_MDR_R = *data++;
		I2C1_MCS_R = I2C_MCS_RUN;  /* middle bytes, no START/STOP */
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
		if(error){
			I2C1_MCS_R = I2C_MCS_STOP;
			while(I2C1_MCS_R & I2C_MCS_BUSY){};
			for(delay = 0; delay < 1000; delay++){}
			return (uint8_t)error;
		}
		size--;
	}

	/* Send last byte with STOP */
	I2C1_MDR_R = *data;
	I2C1_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;

	/* Wait until write has been completed */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Wait until bus is free */
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};

	/* Small delay between transactions */
	for(delay = 0; delay < 500; delay++){};

	/* Check for any error */
	return (I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO));
}
