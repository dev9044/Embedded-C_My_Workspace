/*
 * main.c
 *
 *  Created on: Dec 1, 2024
 *      Author: HP
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

int main(void)
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	// Handle the Interrupt
	GPIO_IRQHandling(0);
}
