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
	//Enable Digital I/O
	GPIO_PORTA_DEN_R   |= I2C1_PINS;
	//Enable internal pull-ups in case external pull-ups are absent
	GPIO_PORTA_PUR_R   |= I2C1_PINS;
	//Enable Alternate Function Selection
	GPIO_PORTA_AFSEL_R |= I2C1_PINS;

	//Select I2C1 as the alternate function 
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~I2C1_ALT_FUNC_MSK) | I2C1_ALT_FUNC_SET;
	//Enable Open Drain for SDA pin
	GPIO_PORTA_ODR_R  |= I2C1_SDA_PIN;
//Disable Analog Mode
	GPIO_PORTA_AMSEL_R &= ~I2C1_PINS;
	// Set SCL as output; leave SDA as input (open-drain handles driving low)
	GPIO_PORTA_DIR_R   |= I2C1_SCL_PIN;
	GPIO_PORTA_DIR_R   &= ~I2C1_SDA_PIN;

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
}

/*
 *	-------------------I2C1_Receive------------------
 *	Polls to receive data from specified peripheral
 *	Input: Slave address & Slave Register Address
 *	Output: Returns 8-bit data that has been received
 */
uint8_t I2C1_Receive(uint8_t slave_addr, uint8_t slave_reg_addr){

	char error;																	//Temp Variable to hold errors

	/* Check if I2C1 is busy: check MCS register Busy bit */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Configure I2C0 Slave Address and Write Mode */
	// Slave Address is the 7 MSB
	I2C1_MSA_R = (slave_addr << 1);  // write (bit 0 = 0)
	// Set Data Register to slave register address
	I2C1_MDR_R = slave_reg_addr;

	/* Initiate I2C by generating a START & RUN cmd:
	   Set MCS register START bit to generate and RUN bit to enable I2C Master	*/
	I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;

	/* Wait until write is done: check MCS register to see is I2C is still busy */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Check for address/datack/arblost errors on first write */
	error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
	if(error){
		/* send STOP if needed and return 0 */
		I2C1_MCS_R = I2C_MCS_STOP;
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		return 0;
	}

	/* Set I2C to Receive with Slave Address and change to Read */
	I2C1_MSA_R = (slave_addr << 1) | 1;   // read (bit 0 = 1)

	/* Initiate I2C by generating a repeated START, STOP, & RUN cmd */
	I2C1_MCS_R = I2C_MCS_START | I2C_MCS_STOP | I2C_MCS_RUN;

	/* Wait until I2C bus is not busy: check MCS register for I2C bus busy bit */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Check for any error: read the error flag from MCS register */
	error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
	if(error){
		/* ensure STOP and clear bus on error */
		I2C1_MCS_R = I2C_MCS_STOP;
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		while(I2C1_MCS_R & I2C_MCS_BUSBSY){};
		return 0;
	}

	/* ensure bus not busy before returning */
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};

	return (uint8_t)I2C1_MDR_R;

}

/*
 *	-------------------I2C1_Transmit------------------
 *	Transmit a byte of data to specified peripheral
 *	Input: Slave address, Slave Register Address, Data to Transmit
 *	Output: Any Errors if detected, otherwise 0
 */
uint8_t I2C1_Transmit(uint8_t slave_addr, uint8_t slave_reg_addr, uint8_t data){

	char error;																	//Temp Variable to hold errors

	/* Check if I2C1 is busy: check MCS register Busy bit */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Configure I2C Slave Address, R/W Mode, and what to transmit */
	//Slave Address is the first 7 MSB
	// Clear LSB to write
	I2C1_MSA_R = ((slave_addr << 1) & ~I2C1_RW_PIN);
	//Transmit register addr to interact
	I2C1_MDR_R = slave_reg_addr;

	/* Initiate I2C by generate a START bit and RUN cmd */
	I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;

	/* Wait until write has been completed */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Check first-cycle errors */
	error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
	if(error){
		I2C1_MCS_R = I2C_MCS_STOP;
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		return error;
	}

	/* Update Data Register with data to be transmitted */
	I2C1_MDR_R = data;

	/* Initiate I2C by generating a STOP & RUN cmd */
	I2C1_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;

	/* Wait until write has been completed: check MCS register Busy bit */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Wait until bus isn't busy: check MCS register for I2C bus busy bit */
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};

	/* Check for any error: read the error flag from MCS register */
	return (I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO));
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

	char error;															//Temp Error Variable

	/* Asserting Param */
	if((data == 0) || (size == 0)){
		return 1;
	}

	/* Check if I2C1 is busy */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Configure I2C Slave Address, R/W Mode, and what to transmit */
	//Slave Address is the first 7 MSB
	//Clear LSB to write
	I2C1_MSA_R = ((slave_addr << 1) & ~I2C1_RW_PIN);
	//Transmit register addr to interact
	I2C1_MDR_R = slave_reg_addr;

	/* Initiate I2C by generate a START bit and RUN cmd */
	I2C1_MCS_R = I2C_MCS_START | I2C_MCS_STOP | I2C_MCS_RUN;

	/* Wait until write has been completed */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Check for any error on the register address write */
	error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
	if(error){
		I2C1_MCS_R = I2C_MCS_STOP;
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		return error;
	}

	/* Loop to Burst Transmit what is stored in data buffer */
	while(size > 1){

		//Deference Pointer from data array and load into data reg. Post-Increment the pointer after
		I2C1_MDR_R = *data++; 
		//Initiate I2C RUN CMD
		I2C1_MCS_R = RUN_CMD; /* middle bytes, no START/STOP */
		//Wait until transmit is complete
		while(I2C1_MCS_R & I2C_MCS_BUSY){};
		error = I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO);
		if(error){
			I2C1_MCS_R = I2C_MCS_STOP;
			while(I2C1_MCS_R & I2C_MCS_BUSY){};
			return error;
		}
		size--;																//Reduce size until 1 is left

	}

	//Deference Pointer from data array and load into data reg
	I2C1_MDR_R = *data;
	//Initiate I2C STOP condition and RUN CMD
	I2C1_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;

	/* Wait until write has been completed */
	while(I2C1_MCS_R & I2C_MCS_BUSY){};

	/* Wait until bus isn't busy */
	while(I2C1_MCS_R & I2C_MCS_BUSBSY){};

	/* Check for any error */
	return (I2C1_MCS_R & (I2C_MCS_ERROR | I2C_MCS_DATACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO));
}
