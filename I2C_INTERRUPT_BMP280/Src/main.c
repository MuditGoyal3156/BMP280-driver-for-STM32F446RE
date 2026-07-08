#include <stdint.h>
#include <stdio.h>
#include "I2C.h"
#include "BMP280.h"
#include "uart.h"
char temp_str[32];
char pressure_str[32];

void delay(void)
{
	for(uint32_t i =0 ; i<250000 ; i++);
}
int main(void)
{
	I2C_INIT();
	I2C_INTERRUPT_EN();
	I2C_ENABLE();
	uart2_tx_init();
	if(BMP280_Init())
	    printf("Sensor OK\r\n");
	else
	    printf("Sensor NOT FOUND\r\n");

	int32_t temp;
	uint32_t press;
	while(1){
        temp = BMP280_ReadTemperature();
        press = BMP280_ReadPressure();

        //Calculations to process values of temperature and pressure
        int32_t t_int = temp / 100;
        int32_t t_dec = temp % 100;
        uint32_t p_pa = press / 256;
        uint32_t p_int = p_pa / 100;
        uint32_t p_dec = p_pa % 100;


        sprintf(temp_str, "Temperature: %ld.%02ld C", t_int, t_dec);
        sprintf(pressure_str, "Pressure: %ld.%02ld hPa", p_int, p_dec);

        printf("%s \r\n",temp_str);
        printf("%s \r\n",pressure_str);

        delay();
	}

}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_HANDLE(&I2C_Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_Handler(&I2C_Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	if(AppEv == I2C_ERROR_BERR){
        printf("I2C Bus Error\r\n");

	}else if(AppEv == I2C_ERROR_ARLO){

        printf("I2C Arbitration Lost\r\n");

	}else if(AppEv == I2C_ERROR_AF)
	{
		printf("Ack failure\n");

		I2C_CloseSendData(&I2C_Handle);

		Generate_Stop();

		while(1);
	}
	else if(AppEv == I2C_ERROR_OVR){

        printf("I2C Overrun Error\r\n");

        I2C_CloseReceiveData(pI2CHandle);

        Generate_Stop();

        while(1);
	}
}
