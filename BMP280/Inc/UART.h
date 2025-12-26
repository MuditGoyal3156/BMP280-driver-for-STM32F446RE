/*
 * UART.h
 *
 *  Created on: Dec 26, 2025
 *      Author: mudit
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "Addresses.h"

#define UART_BAUDRATE    115200

#define GPIOAEN          (1U << 0)
#define UART2EN          (1U << 17)

#define CR1_TE           (1U << 3)
#define CR1_UE           (1U << 13)
#define SR_TXE           (1U << 7)

#define SYS_FREQ         16000000
#define APB1_CLK         SYS_FREQ

#define UART_BAUDRATE    115200

void uart2_write(int ch);
void uart2_tx_init(void);
void uart_set_baudrate(uint32_t PeriphClk, uint32_t BaudRate);
uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

#endif /* UART_H_ */
