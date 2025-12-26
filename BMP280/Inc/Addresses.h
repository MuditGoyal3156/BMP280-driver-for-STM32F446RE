/*
 * Addresses.h
 *
 *  Created on: Dec 24, 2025
 *      Author: mudit
 */

#ifndef ADDRESSES_H_
#define ADDRESSES_H_

#include <stdint.h>

/* RCC */
#define RCC_BASE        0x40023800U
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30U))
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x40U))

/* GPIOA */
#define GPIOA_BASE      0x40020000U
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_PUPDR     (*(volatile uint32_t *)(GPIOA_BASE + 0x0CU))
#define GPIOA_IDR       (*(volatile uint32_t *)(GPIOA_BASE + 0x10U))
#define GPIOA_ODR       (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))
#define GPIOA_AFRL      (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))
#define GPIOA_AFRH      (*(volatile uint32_t *)(GPIOA_BASE + 0x24U))

/* GPIOB */
#define GPIOB_BASE      0x40020400U
#define GPIOB_MODER     (*(volatile uint32_t *)(GPIOB_BASE + 0x00U))
#define GPIOB_OTYPER    (*(volatile uint32_t *)(GPIOB_BASE + 0x04U))
#define GPIOB_OSPEEDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x08U))
#define GPIOB_PUPDR     (*(volatile uint32_t *)(GPIOB_BASE + 0x0CU))
#define GPIOB_IDR       (*(volatile uint32_t *)(GPIOB_BASE + 0x10U))
#define GPIOB_ODR       (*(volatile uint32_t *)(GPIOB_BASE + 0x14U))
#define GPIOB_AFRL      (*(volatile uint32_t *)(GPIOB_BASE + 0x20U))
#define GPIOB_AFRH      (*(volatile uint32_t *)(GPIOB_BASE + 0x24U))

/* UART2 */
#define UART2_BASE      0x40004400U
#define UART2_SR        (*(volatile uint32_t *)(UART2_BASE + 0x00U))
#define UART2_DR        (*(volatile uint32_t *)(UART2_BASE + 0x04U))
#define UART2_BRR       (*(volatile uint32_t *)(UART2_BASE + 0x08U))
#define UART2_CR1       (*(volatile uint32_t *)(UART2_BASE + 0x0CU))
#define UART2_CR2       (*(volatile uint32_t *)(UART2_BASE + 0x10U))
#define UART2_CR3       (*(volatile uint32_t *)(UART2_BASE + 0x14U))

/* I2C1 */
#define I2C1_BASE        0x40005400U
#define I2C1_CR1     	 (*(volatile uint32_t *)(I2C1_BASE))
#define I2C1_CR2     	 (*(volatile uint32_t *)(I2C1_BASE + 0x04U))
#define I2C1_OAR1     	 (*(volatile uint32_t *)(I2C1_BASE + 0x08U))
#define I2C1_DR     	 (*(volatile uint32_t *)(I2C1_BASE + 0x10U))
#define I2C1_SR1     	 (*(volatile uint32_t *)(I2C1_BASE + 0x14U))
#define I2C1_SR2     	 (*(volatile uint32_t *)(I2C1_BASE + 0x18U))
#define I2C1_CCR     	 (*(volatile uint32_t *)(I2C1_BASE + 0x1CU))
#define I2C1_TRISE    	 (*(volatile uint32_t *)(I2C1_BASE + 0x20U))






#endif /* ADDRESSES_H_ */
