/*
 * 001ledtoggle.c
 *
 *  Created on: Dec 1, 2024
 *      Author: HP
 */

/*
 * Write a program to toggle the on board LED with some delay.
 * case2: Use open drain configuration for the output pin.
 */
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	// This will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed = {0U};

	memset(&GpioLed,0U,sizeof(GpioLed));

	// This is GPIOD Port Configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	while(1U)
	{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay(); // 200ms Wait
	}

	return 0;
}
