/*
 * I2C.c
 *
 *  Created on: Dec 26, 2025
 *      Author: mudit
 */


#include "I2C.h"

/*
 * I2C1
 * SCL ->PB8
 * SDA ->PB9
 * AF4
 */
void I2C1_GPIO_Init(void)
{
	//For PB8 and PB9

	//Enable GPIOB clock
	RCC_AHB1ENR |= (1 << 1);

	//Mode Alternate function
	GPIOB_MODER &= ~(3 <<16);//Clear bits for PB8
	GPIOB_MODER &= ~(3 <<18);//Clear bits for PB9

	GPIOB_MODER |= (1 <<17);//Set bits for PB8
	GPIOB_MODER |= (1 <<19);//Set bits for PB9

	//AF4
	GPIOB_AFRH &= ~(0xF << 0);//Clear bits for PB8
	GPIOB_AFRH &= ~(0xF << 4);//Clear bits for PB9

	GPIOB_AFRH |= (4 << 0);//PB8 AF4
	GPIOB_AFRH |= (4 << 4);//PB9 AF4

	//Open drain
	GPIOB_OTYPER |= (3 << 8);

	//Pull Up (Remove if external pullup are used)
	GPIOB_PUPDR &= ~(3 << 16);//Clear bits for PB8
	GPIOB_PUPDR &= ~(3 << 18);//Clear bits for PB9

	GPIOB_PUPDR |= (1 << 16);//Pull up for PB8
	GPIOB_PUPDR |= (1 << 18);//Pull up for PB9

	//Speed
	GPIOB_OSPEEDR |= (3 << 16);
	GPIOB_OSPEEDR |= (3 << 18);
}

void I2C1_Init(void)
{
	//1. Enable Clock
	RCC_APB1ENR |= (1 << 21);

	//2. Make sure Peripheral is disabled
	I2C1_DI();

	//3. Configure Standard Mode
	I2C1_CCR &= ~(1 << 15);

	//4. Speed of SCL
	//CR2 FREQ bits
	I2C1_CR2 &= ~(0x3F);
	I2C1_CR2 |= 0x10;

	//CCR field in CCR Register
	I2C1_CCR = 0x50;

	//5. Configure Rise Time
	I2C1_TRISE &= ~(0x3F);
	I2C1_TRISE |= 0x11;

	//Enable ACKing
	I2C1_CR1 |= (1 << 10);

}

/*
 * Enable I2C1
 */
void I2C1_EN(void)
{
	I2C1_CR1 |= (1 << 0);

	//Enable ACKing
	I2C1_CR1 |= (1 << 10);
}

/*
 * Disable I2C1
 */
void I2C1_DI(void)
{
	I2C1_CR1 &= ~(1 << 0);
}
void I2C_GenerateStart(void)
{
	I2C1_CR1 |= (1 << 8);
}
void I2C_GenerateStop(void)
{
	I2C1_CR1 |= (1 << 9);
}

void Execute_Send_Addr(uint8_t Slave_ADDR)
{
	Slave_ADDR = Slave_ADDR << 1;
	I2C1_DR = Slave_ADDR;
}

void Execute_Read_Addr(uint8_t Slave_ADDR)
{
	Slave_ADDR = Slave_ADDR << 1;
	Slave_ADDR |= 1;
	I2C1_DR = Slave_ADDR;
}
void Clear_ADDR_Flag(void)
{
	uint32_t dummy = I2C1_SR1;
	dummy = I2C1_SR2;
	(void)dummy;
}
void I2C1_SendData(uint8_t *Data,uint8_t len,uint8_t Slave_ADDR)
{
	//Generate Start Condition
	I2C_GenerateStart();

	//Check for SB flag in SR1
	while(!(I2C1_SR1 & (1 << 0)));

	//Send Address
	Execute_Send_Addr(Slave_ADDR);

	//Check for ADDR Flag
	while(!(I2C1_SR1 & (1 << 1)));

	//Clear ADDR flag
	Clear_ADDR_Flag();

	while(len > 0)
	{
		while(!(I2C1_SR1 & (1 << 7)));//Check for TXE flag
		I2C1_DR = *Data;//Load DR
		len--;//Decrement length
		Data++;//Increase pointer address
	}

	//When length becomes zero check for TXE and BTF flag
	while(!(I2C1_SR1 & (1 << 7)));//Check for TXE flag
	while(!(I2C1_SR1 & (1 << 2)));//Check for BTF flag

	//Generate stop condition
	I2C_GenerateStop();
}


void I2C1_ReadData(uint8_t *Data,uint8_t len,uint8_t Slave_ADDR)
{

	//Generate Start Condition
	I2C_GenerateStart();

	//Check for SB flag in SR1
	while(!(I2C1_SR1 & (1 << 0)));

	//Send Address
	Execute_Read_Addr(Slave_ADDR);

	//Check for ADDR Flag
	while(!(I2C1_SR1 & (1 << 1)));


	if(len == 1)
	{
		//Clear ACK
		I2C1_CR1 &= ~(1 << 10);

		//Clear ADDR flag
		Clear_ADDR_Flag();

		//Wait until RXNE is 1
		while(!(I2C1_SR1 & (1 << 6)));

		//Generate stop condition
		I2C_GenerateStop();

		//Read data into buffer
		*Data = I2C1_DR ;
	}

	if(len > 1)
	{
		//Clear ADDR flag
		Clear_ADDR_Flag();

		while(len > 0)
		{
			//Wait till RXNE is 1
			while(!(I2C1_SR1 & (1 << 6)));

			if(len == 2)
			{
				//Clear ACK bit
				I2C1_CR1 &= ~(1 << 10);

				//Generate STOP condition
				I2C_GenerateStop();

			}
			//Read data into buffer
			*Data = I2C1_DR ;

			//increment buffer address
			Data++;

			//Decrement length
			len--;
		}
	}
	//Enable ACKing
	I2C1_CR1 |= (1 << 10);
}
