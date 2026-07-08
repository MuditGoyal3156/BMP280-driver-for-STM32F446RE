#include "I2C.h"

I2C_Handle_t I2C_Handle;
/*
 * I2C1
 * AF4
 * PB8 -> SCL
 * PB9 -> SDA
 */
void GPIO_INIT(void){
	//Enable GPIO B peripheral clock
	RCC->AHB1ENR |= (1U << 1);

	//Set output mode to Alternate function
	GPIOB->MODER &= ~(3U << 16);
	GPIOB->MODER |= (1U << 17);

	GPIOB->MODER &= ~(3U << 18);
	GPIOB->MODER |= (1U << 19);

	//Select AF4
	GPIOB->AFR[1] &= ~(0XF << 0);
	GPIOB->AFR[1] |= (4U << 0);

	GPIOB->AFR[1] &= ~(0XF << 4);
	GPIOB->AFR[1] |= (4U << 4);

	//Select open drain
	GPIOB->OTYPER |= (3U << 8);

	//Enable pull-ups(If external pull-ups not there)
	GPIOB->PUPDR &= ~(3U << 16);
	GPIOB->PUPDR |= (1U << 16);

	GPIOB->PUPDR &= ~(3U << 18);
	GPIOB->PUPDR |= (1U << 18);

	//Select GPIO Speed
	GPIOB->OSPEEDR |= (3U << 16);
	GPIOB->OSPEEDR |= (3U << 18);

}

void I2C_INIT(void){
	//Initialize GPIO
	GPIO_INIT();

	//Enable peripheral clock
	RCC->APB1ENR |= (1U << 21);

	//Make sure that peripheral is disabled
	I2C_DISABLE();

	//Select Standard Mode
	I2C1->CCR &= ~(1U << 15);

	//Program peripheral input clock in CR2
	I2C1->CR2 &= ~(0X3F);
	I2C1->CR2 |= 0X10;

	//Configure clock control register
	I2C1->CCR = 0x50;

	//Configure rise time
	I2C1->TRISE &= ~(0x3F);
	I2C1->TRISE |= 0x11;

	//Enable ACK
	I2C1->CR1 |= (1U << 10);

}

void I2C_ENABLE(void){
	I2C1->CR1 |= (1U << 0);

	//Enable ACK
	I2C1->CR1 |= (1U << 10);
}
void I2C_DISABLE(void){
	I2C1->CR1 &= ~(1U << 0);

}

void Generate_Start(void){
	I2C1->CR1 |= (1U << 8);
}

void Generate_Stop(void){
	I2C1->CR1 |= (1U << 9);

}

void send_read_adddr(uint8_t ADDR){
	ADDR = ADDR << 1;//7Bit address

	ADDR |= 1;

	//Load it in Register
	I2C1->DR = ADDR;
}

void send_write_adddr(uint8_t ADDR){
	ADDR = ADDR << 1;//7Bit address

	//Load it in Register
	I2C1->DR = ADDR;
}

void Clear_ADDR_Flag(void){
	uint32_t dummy = I2C1->SR1;
	dummy = I2C1->SR2;
	(void)dummy;

}

uint8_t I2C_SEND_DATA(I2C_Handle_t *pI2CHandle,uint8_t *Data,uint8_t len,uint8_t Slave_ADDR,uint8_t Sr){

	uint8_t state = pI2CHandle->TxRxState;
	if( (state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pTxBuffer = Data;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->TxLen = len;
		pI2CHandle->DevAddr = Slave_ADDR;
		pI2CHandle->Sr = Sr;//For BMP280
		//Generate Start
		Generate_Start();

		//Enable interrupts
		I2C1->CR2 |= (1U << 8);
		I2C1->CR2 |= (1U << 9);
		I2C1->CR2 |= (1U << 10);
	}
	return state;

}

uint8_t I2C_RX_DATA(I2C_Handle_t *pI2CHandle,uint8_t *Data,uint8_t len,uint8_t Slave_ADDR){

	uint8_t state = pI2CHandle->TxRxState;
	if( (state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pRxBuffer = Data;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize =  len;
		pI2CHandle->RxLen = len;
		pI2CHandle->DevAddr = Slave_ADDR;

		//Generate Start
		Generate_Start();

		//Enable interrupts
		I2C1->CR2 |= (1U << 8);
		I2C1->CR2 |= (1U << 9);
		I2C1->CR2 |= (1U << 10);
	}
	return state;
}

void I2C_INTERRUPT_EN(void)
{
	//IRQ number for I2C1 ->31(EV) and 32(ER)

	//Set ISER
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);

	//Configure Priority
	NVIC_SetPriority(I2C1_EV_IRQn,2);
	NVIC_SetPriority(I2C1_ER_IRQn,3);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	I2C1->CR2 &= ~( 1U << 10);

	//Implement the code to disable ITEVFEN Control Bit
	I2C1->CR2 &= ~( 1U << 9);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

	//Enable ACKing
	I2C1->CR1 |= (1 << 10);
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	I2C1->CR2 &= ~( 1U << 10);

	//Implement the code to disable ITEVFEN Control Bit
	I2C1->CR2 &= ~( 1U << 9);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	//Enable ACKing
	I2C1->CR1 |= (1 << 10);

}
void I2C_EV_HANDLE(I2C_Handle_t *pI2CHandle){

	//Check for SB flag

	if((I2C1->SR1)&(1U << 0)){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			send_write_adddr(pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			send_read_adddr(pI2CHandle->DevAddr);
		}
	}

	//Check for ADDR flag
	if((I2C1->SR1)&(1U << 1)){
		if((pI2CHandle->TxRxState == I2C_BUSY_IN_RX) && (pI2CHandle->RxSize == 1))
		{
			I2C1->CR1 &= ~(1U << 10);
		}

		Clear_ADDR_Flag();
	}
	//Check for BTF flag
	if((I2C1->SR1)&(1U << 2)){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure that TXE is also set
			if((I2C1->SR1)&(1U << 7))
			{
				//BTF and TXE both are set
				if(pI2CHandle->TxLen == 0)
				{
					//Generate the stop condition
					if(pI2CHandle->Sr == 0)
					{
					    Generate_Stop();
					}

					//Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}
	}
	//Check for STOPF
	if((I2C1->SR1)&(1U << 4)){
		//Clear the STOPF Flag(1.read the SR1 2.write to CR1)
		uint32_t dummy = I2C1->SR1;
		I2C1->CR1 |= 0x0000 ;
		(void) dummy;

		//Notify the application that stop is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}
	//Check for TxE
	if((I2C1->SR1)&(1U << 7)){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->TxLen > 0)
			{
				//Load the data into DR
				I2C1->DR = *(pI2CHandle->pTxBuffer);

				//Decrement TX length
				pI2CHandle->TxLen--;

				//Increment the buffer address
				pI2CHandle->pTxBuffer++;

			}
		}

	}
	//Check for RxE
	if((I2C1->SR1)&(1U << 6)){
		//RXNE flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxLen == 1)
			{
				I2C1->CR1 &= ~(1U << 10);   // disable ACK
			    Generate_Stop();
			     *(pI2CHandle->pRxBuffer) = I2C1->DR;
			     pI2CHandle->RxLen--;
			     // close and notify here
			     I2C_CloseReceiveData(pI2CHandle);
			     I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
			 }
			 else if(pI2CHandle->RxLen > 1)
			 {
			     if(pI2CHandle->RxLen == 2)
			         I2C1->CR1 &= ~(1U << 10);   // pre-clear ACK before last byte
			     *(pI2CHandle->pRxBuffer) = I2C1->DR;
			     pI2CHandle->RxLen--;
			     pI2CHandle->pRxBuffer++;
			 }

	}
}
}

void I2C_ER_Handler(I2C_Handle_t *pI2CHandle)
{
    if(I2C1->SR1 & (1U << 8))
    {
        I2C1->SR1 &= ~(1U << 8);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
    }

    if(I2C1->SR1 & (1U << 9))
    {
        I2C1->SR1 &= ~(1U << 9);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
    }

    if(I2C1->SR1 & (1U << 10))
    {
        I2C1->SR1 &= ~(1U << 10);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
    }

    if(I2C1->SR1 & (1U << 11))
    {
        I2C1->SR1 &= ~(1U << 11);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
    }
}

void __attribute__((weak)) I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv){

}
