/*
 * 006spi_txonly_arduino.c
 *
 *  Created on: Dec 29, 2024
 *      Author: HP
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

/*
 * SPI Master (STM) and SPI Slave (Arduino) Communication.
 * When the button on the master is pressed, master should send string of data to the Arduino slave connected.
 * The data received by the Arduino will be displayed on the Arduino serial port.
 *
 * 1. Use SPI Full duplex mode
 * 2. ST Board will be in SPI master mode and Arduino will be configured for SPI slave mode
 * 3. Use DFF = 0
 * 4. Use Hardware slave management (SSM = 0)
 * 5. SCLK speed = 2Mhz, fclk = 16Mhz
 *
 * In this exercise master is not going to receive anything for the slave. So you may not configure the MISO Pin.
 *
 * Note - Slave does not know how many bytes of data master is going to send. So master first sends the number of bytes info
 * which slave is going to receive next.
 */

/*
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCLK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 * ALT Function Mode - 5
 */

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

void GPIO_Button_Init(void)
{
	GPIO_Handle_t GpioBtn = {0U};

	memset(&GpioBtn,0U,sizeof(GpioBtn));

	// This is Btn GPIO Configuration
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

int main(void)
{
	char user_data[] = "Hello World";

	GPIO_Button_Init();

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

	while(1U)
	{
		// Wait till button is pressed
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		// Enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// First send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1U);

		// To Send Data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// Lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0U;
}
