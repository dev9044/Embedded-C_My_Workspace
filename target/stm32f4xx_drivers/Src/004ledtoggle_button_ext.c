/*
 * 001ledtoggle.c
 *
 *  Created on: Dec 1, 2024
 *      Author: HP
 */

/*
 * Write a program to connect external button to the pin number PB12 and external LED to PA8
 * and Toggle the LED whenever the external button is pressed.
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
	GPIO_Handle_t GpioBtn = {0U};

	memset(&GpioLed,0U,sizeof(GpioLed));
    memset(&GpioBtn,0U,sizeof(GpioBtn));

	// This is GPIOA Port Configuration for External LED
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	// This is Btn GPIO Configuration for External Button
	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Init(&GpioBtn);

	while(1U)
	{
		if (BTN_PRESSED_LOW == GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12))
		{
			delay(); // 200ms Wait till button de-bouncing gets over
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
		}
	}
	return 0;
}
