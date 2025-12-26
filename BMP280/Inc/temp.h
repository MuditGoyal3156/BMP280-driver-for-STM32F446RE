/*
 * temp.h
 *
 *  Created on: Dec 24, 2025
 *      Author: mudit
 */

#ifndef TEMP_H_
#define TEMP_H_

#include <stdint.h>

#define BMP280_ADDR       0x76

/* Registers */
#define BMP280_ID_REG     0xD0
#define BMP280_RESET_REG  0xE0
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_CONFIG     0xF5
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CALIB_REG  0x88

uint8_t BMP280_Init(void);
int32_t BMP280_ReadTemperature(void);
uint32_t BMP280_ReadPressure(void);


#endif /* TEMP_H_ */
