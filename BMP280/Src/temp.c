/*
 * temp.c
 *
 *  Created on: Dec 24, 2025
 *      Author: mudit
 */

#include "I2C.h"
#include "temp.h"

static int32_t t_fine;

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_Calib_t;

static BMP280_Calib_t calib;

static void bmp280_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    I2C1_SendData(buf, 2, BMP280_ADDR);
}

static void bmp280_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    //Generate Start
    I2C_GenerateStart();
    while(!(I2C1_SR1 & (1 << 0))); // Wait for SB

    Execute_Send_Addr(BMP280_ADDR); // Send Slave Address
    while(!(I2C1_SR1 & (1 << 1))); // Wait for ADDR flag

    // Clear ADDR flag
    uint32_t dummy = I2C1_SR1;
    dummy = I2C1_SR2;
    (void)dummy;

    while(!(I2C1_SR1 & (1 << 7))); // Wait for TXE
    I2C1_DR = reg;                 // Send the Register Address we want to read
    while(!(I2C1_SR1 & (1 << 7))); // Wait for TXE

    //Repeated Start

    I2C1_ReadData(buf, len, BMP280_ADDR);
}
static void bmp280_read_calibration(void)
{
    uint8_t buf[24];
    bmp280_read(BMP280_CALIB_REG, buf, 24);

    calib.dig_T1 = (buf[1] << 8) | buf[0];
    calib.dig_T2 = (buf[3] << 8) | buf[2];
    calib.dig_T3 = (buf[5] << 8) | buf[4];

    calib.dig_P1 = (buf[7] << 8) | buf[6];
    calib.dig_P2 = (buf[9] << 8) | buf[8];
    calib.dig_P3 = (buf[11] << 8) | buf[10];
    calib.dig_P4 = (buf[13] << 8) | buf[12];
    calib.dig_P5 = (buf[15] << 8) | buf[14];
    calib.dig_P6 = (buf[17] << 8) | buf[16];
    calib.dig_P7 = (buf[19] << 8) | buf[18];
    calib.dig_P8 = (buf[21] << 8) | buf[20];
    calib.dig_P9 = (buf[23] << 8) | buf[22];
}

uint8_t BMP280_Init(void)
{
    uint8_t id;
    bmp280_read(BMP280_ID_REG, &id, 1);
    if (id != 0x58)
    {
    	return 0;
    }

    bmp280_write(BMP280_RESET_REG, 0xB6);
    for(int i = 0; i < 100000; i++);
    bmp280_read_calibration();


    bmp280_write(BMP280_CTRL_MEAS, 0x27);


    bmp280_write(BMP280_CONFIG, 0xA0);

    return 1;
}

int32_t BMP280_ReadTemperature(void)
{
    uint8_t buf[3];
    bmp280_read(BMP280_TEMP_MSB, buf, 3);

    int32_t adc_T = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * calib.dig_T2) >> 11;

    int32_t var2 = (((((adc_T >> 4) - calib.dig_T1) * ((adc_T >> 4) - calib.dig_T1)) >> 12) * calib.dig_T3) >> 14;

    t_fine = var1 + var2;

    return (t_fine * 5 + 128) >> 8;
}

uint32_t BMP280_ReadPressure(void)
{
    uint8_t buf[3];
    bmp280_read(BMP280_PRESS_MSB, buf, 3);

    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * calib.dig_P6;
    var2 += (var1 * calib.dig_P5) << 17;
    var2 += ((int64_t)calib.dig_P4) << 35;

    var1 = ((var1 * var1 * calib.dig_P3) >> 8) +
           ((var1 * calib.dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * calib.dig_P1 >> 33;

    if (var1 == 0) return 0;

    int64_t p = 1048576 - adc_P;
    p = ((p << 31) - var2) * 3125 / var1;
    var1 = calib.dig_P9 * (p >> 13) * (p >> 13) >> 25;
    var2 = calib.dig_P8 * p >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)calib.dig_P7 << 4);

    return (uint32_t)p;
}
