#include"UART.h"

int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

void uart2_tx_init(void)
{
	/*Configure UART GPIO pin*/
	/*Enable clock access to GPIOA*/
	RCC_AHB1ENR |= GPIOAEN;

	/*Set PA2 mode to Alternate function mode*/
	GPIOA_MODER &= ~(1U << 4);
	GPIOA_MODER |= (1U << 5);

	/*Set PA2 as Pull-up*/
	GPIOA_PUPDR &= ~(3 << 4);
	GPIOA_PUPDR |=  (1 << 4);

	/*Set PA2 alternate function type to UART_TX (AF07)*/
	GPIOA_AFRL |= (7U << 8);
	GPIOA_AFRL &= ~(1U << 11);


	/*Configure UART module*/
	/*Enable clock access to uart2*/
	RCC_APB1ENR |=UART2EN;

	/*Configure Baudrate*/
	uart_set_baudrate(APB1_CLK,UART_BAUDRATE);

	/*Configure the transfer direction*/
	UART2_CR1 = CR1_TE;

	/*Enable UART module*/
	UART2_CR1 |=CR1_UE;
}

void uart2_write(int ch)
{
	 /*Make sure transmit data register is empty*/
	while(!(UART2_SR & SR_TXE)){}
	/*Write to transmit data register*/
	UART2_DR = (ch & 0xFF);
}

void uart_set_baudrate(uint32_t PeriphClk, uint32_t BaudRate)
{
	UART2_BRR = compute_uart_bd(PeriphClk,BaudRate);
}


uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return (PeriphClk + (BaudRate/2U))/BaudRate;
}



