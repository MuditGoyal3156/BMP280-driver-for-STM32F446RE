/*
 * I2C.h
 *
 *  Created on: Dec 26, 2025
 *      Author: mudit
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "Addresses.h"

void I2C1_GPIO_Init(void);
void I2C1_Init(void);
void I2C1_EN(void);
void I2C1_DI(void);
void I2C_GenerateStart(void);
void I2C_GenerateStop(void);
void Execute_Send_Addr(uint8_t Slave_ADDR);
void Execute_Read_Addr(uint8_t Slave_ADDR);
void I2C1_SendData(uint8_t *Data,uint8_t len,uint8_t Slave_ADDR);
void I2C1_ReadData(uint8_t *Data,uint8_t len,uint8_t Slave_ADDR);
#endif /* I2C_H_ */
