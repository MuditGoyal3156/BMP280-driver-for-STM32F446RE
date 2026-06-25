#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#define STM32F446xx
#include "stm32f4xx.h"
#include <stddef.h>
typedef struct
{
	uint8_t	*pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	volatile uint8_t TxRxState;
	uint8_t DevAddr;
	uint8_t RxSize;
	uint8_t Sr;

}I2C_Handle_t;

extern I2C_Handle_t I2C_Handle;
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO 		4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9


void GPIO_INIT(void);
void I2C_INIT(void);
void I2C_DISABLE(void);
void I2C_ENABLE(void);
void I2C_INTERRUPT_EN(void);
void Generate_Start(void);
void Generate_Stop(void);
void send_read_adddr(uint8_t ADDR);
void send_write_adddr(uint8_t ADDR);
uint8_t I2C_SEND_DATA(I2C_Handle_t *pI2CHandle,uint8_t *Data,uint8_t len,uint8_t Slave_ADDR,uint8_t Sr);
uint8_t I2C_RX_DATA(I2C_Handle_t *pI2CHandle,uint8_t *Data,uint8_t len,uint8_t Slave_ADDR);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_EV_HANDLE(I2C_Handle_t *pI2CHandle);
void I2C_ER_Handler(I2C_Handle_t *pI2CHandle);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);
#endif /* I2C_H_ */
