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

}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT){
	}else if(AppEv == I2C_EV_RX_CMPLT){


	}else if(AppEv == I2C_ERROR_AF)
	{
		printf("Error : Ack failure\n");
		//in master ACK failure happens when slave fails to send ack for the byte from the master
		I2C_CloseSendData(&I2C_Handle);

		//Generate the stop condition
		Generate_Stop();

		//hang in infinite loop
		while(1);
	}
}
