/*
 * 009spi_message_rcv_it.c
 *
 *  Created on: Jan 15, 2025
 *      Author: HP
 */

#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI Interrupt Mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 *
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board, acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */

/*
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCLK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 * ALT Function Mode - 5
 */

SPI_Handle_t SPI2Handle;

#define MAX_LEN			(500U)

char RcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop = 0U;

// This flag will be set in the interrupt handler of the Arduino interrupt GPIO
volatile uint8_t dataAvailable = 0U;

void delay(void)
{
	// This will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i = 0; i < 500000/2; i++);
}

void SPI2_GPIO_Inits(void)
{
	GPIO_Handle_t SPIPins = {0U};

	memset(&SPIPins,0U,sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5U;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle = {0U};

	memset(&SPI2Handle,0U,sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // Generates SCLK of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS Pin

	SPI_Init(&SPI2Handle);
}

void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin = {0U};

	memset(&spiIntPin, 0U, sizeof(spiIntPin));

	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&spiIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
}

int main(void)
{
	uint8_t dummy = 0xFFU;

	Slave_GPIO_InterruptPinInit();

	// This function is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIO_Inits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS Pin is automatically managed by the hardware.
	 * i.e. when SPE = 1, NSS will be pulled to low
	 * and NSS Pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while(1U)
	{
		rcvStop = 0U;

		// Wait till data available interrupt from transmitter device(slave)
		while(!dataAvailable);

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		// Enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!rcvStop)
		{
			// Fetch the data from the SPI peripheral byte by byte in interrupt mode
			while(SPI_BUSY_IN_TX == SPI_SendDataIT(&SPI2Handle, &dummy, 1U));
			while(SPI_BUSY_IN_RX == SPI_ReceiveDataIT(&SPI2Handle, (uint8_t*)&ReadByte, 1U));
		}

		// Lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		dataAvailable = 0U;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	}

	return 0U;
}

// Runs when a data byte is received from the peripheral over SPI
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t ApplEvent)
{
	static uint32_t i = 0U;

	/* In the Rx Complete event, copy data in to rcv buffer. '\0 indicates end of message (rcvStop = 1) */
	if (SPI_EVENT_RX_CMPLT == ApplEvent)
	{
		RcvBuff[i++] = ReadByte;

		if ((ReadByte == '\0') || (MAX_LEN == i))
		{
			rcvStop = 1U;
			RcvBuff[i-1U] = '\0';
			i = 0U;
		}
	}
}

// Slave data available interrupt handler
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1U;
}
