/*
 * 006spi_txbuffer_testing.c
 *
 *  Created on: Dec 29, 2024
 *      Author: HP
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

/*
 * Test the SPI_SendData API to send the string "Hello World" and use the below configurations
 * 1. SPI-2 Master Mode
 * 2. SCLK = Max Possible
 * 3. DFF = 0 and DFF = 1
 */

/*
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCLK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 * ALT Function Mode - 5
 */

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // Generates SCLK of 8MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // Software slave management enabled for NSS Pin

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	char user_data[] = "Hello World";

	// This function is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIO_Inits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// This makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	// Enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// To Send Data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	// Lets confirm SPI is not nusy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	// Disable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1U);

	return 0U;
}
