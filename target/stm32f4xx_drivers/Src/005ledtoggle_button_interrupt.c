/*
 * 005ledtoggle_button_interrupt.c
 *
 *  Created on: Dec 6, 2024
 *      Author: HP
 */

/*
 * Connect an external button to PD5 pin and toggle the LED whenever interrupt is triggered by the button press.
 * Interrupt should be triggered during falling edge of button press.
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

	// This is GPIOD Port Configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	// This is Btn GPIO Configuration
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioBtn);

	// IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	while(1U);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay(); // 200ms Wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_5);	// Clear the pending event from EXTI Line
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
