/*
 * 006spi_cmd_handling.c
 *
 *  Created on: Jan 11, 2025
 *      Author: HP
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

/*
 * SPI Master (STM) and SPI Slave (Arduino) Command & response based Communication.
 * When the button on the master is pressed, master send a command to the slave and slave responds as per the command implementation.
 *
 * 1. Use SPI Full duplex mode
 * 2. ST Board will be in SPI master mode and Arduino will be configured for SPI slave mode
 * 3. Use DFF = 0
 * 4. Use Hardware slave management (SSM = 0)
 * 5. SCLK speed = 2Mhz, fclk = 16Mhz
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

//extern void initialise_monitor_handles();

// Command Codes
#define COMMAND_LED_CTRL			(0x50U)
#define COMMAND_SENSOR_READ			(0x51U)
#define COMMAND_LED_READ			(0x52U)
#define COMMAND_PRINT				(0x53U)
#define COMMAND_ID_READ				(0x54U)

#define LED_ON						(1U)
#define LED_OFF						(0U)

// Arduino Analog Pins
#define ANALOG_PIN0					(0U)
#define ANALOG_PIN1					(1U)
#define ANALOG_PIN2					(2U)
#define ANALOG_PIN3					(3U)
#define ANALOG_PIN4					(4U)

// Arduino LED
#define LED_PIN						(9U)

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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if (0xF5 == ackbyte)
	{
		return 1U;
	}

	return 0U;
}
int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read = 0U;
	uint8_t ackbyte = 0U;
	uint8_t args[2U] = {0U};
	uint8_t CommandCode = 0U;
	uint8_t analog_read = 0U;
	uint8_t led_status = 0U;

	//initialize_monitor_handles();

	//printf("Application is running. \n");

	GPIO_Button_Init();

	// This function is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIO_Inits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//printf("SPI Initialized. \n");

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

		// 1. Start of COMMAND_LED_CTRL  <pin no(1)> <value(1)>
		CommandCode = COMMAND_LED_CTRL;

		// Send Command
		/* This transmission of 1 byte resulted 1 garbage byte collection in Rx buffer of the master
		    and RXNE flag is set. So do the dummy read and clear the flag. */
		SPI_SendData(SPI2, &CommandCode, 1U);

		// Do Dummy Read to clear of the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1U);

		// Send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1U);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1U);

		if (SPI_VerifyResponse(ackbyte))
		{
			// Send Arguments
			args[0U] = LED_PIN;
			args[1U] = LED_ON;

			SPI_SendData(SPI2, args, 2U);

			//printf("COMMAND_LED_CTRL Executed. \n");
		}
		// End of COMMAND_LED_CTRL

		// 2. Start of COMMAND_SENSOR_READ	<analog pin number(1)>

		// Wait till button is pressed
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		CommandCode = COMMAND_SENSOR_READ;
		// Send Command
		/* This transmission of 1 byte resulted 1 garbage byte collection in Rx buffer of the master
		   and RXNE flag is set. So do the dummy read and clear the flag. */
		SPI_SendData(SPI2, &CommandCode, 1U);

		// Do Dummy Read to clear of the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1U);

		// Send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1U);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1U);

		if (SPI_VerifyResponse(ackbyte))
		{
			// Send Argument
			args[0U] = ANALOG_PIN0;

			SPI_SendData(SPI2, args, 1U);

			// Do Dummy Read to clear of the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1U);

			/* Slave actually takes some time to read the analog value (Slave does ADC conversion on that pin)
			 So Master should wait for sometime before generating the dummy bits to fetch the result. */

			// Insert some delay so that slave can ready with the data
			delay();

			// Send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1U);

			// Read Analog Pin
			SPI_ReceiveData(SPI2, &analog_read, 1U);

			//printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}
		// End of COMMAND_SENSOR_READ

		// 3. Start of COMMAND_LED_READ	<pin no(1)>

		// Wait till button is pressed
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		CommandCode = COMMAND_LED_READ;
		// Send Command
		/* This transmission of 1 byte resulted 1 garbage byte collection in Rx buffer of the master
		    and RXNE flag is set. So do the dummy read and clear the flag. */
		SPI_SendData(SPI2, &CommandCode, 1U);

		// Do Dummy Read to clear of the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1U);

		// Send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1U);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1U);

		if (SPI_VerifyResponse(ackbyte))
		{
			// Send Argument
			args[0U] = LED_PIN;

			SPI_SendData(SPI2, args, 1U);

			// Do Dummy Read to clear of the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1U);

			/* Slave actually takes some time to read the analog value (Slave does ADC conversion on that pin)
			 So Master should wait for sometime before generating the dummy bits to fetch the result. */

			// Insert some delay so that slave can ready with the data
			delay();

			// Send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1U);

			// Read Led Pin
			SPI_ReceiveData(SPI2, &led_status, 1U);

			//printf("COMMAND_LED_READ %d\n",led_status);
		}
		// End of COMMAND_LED_READ

		// 4. Start of COMMAND_PRINT	<len(2)>   <message(len)>

		// Wait till button is pressed
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		CommandCode = COMMAND_PRINT;
		// Send Command
		/* This transmission of 1 byte resulted 1 garbage byte collection in Rx buffer of the master
		    and RXNE flag is set. So do the dummy read and clear the flag. */
		SPI_SendData(SPI2, &CommandCode, 1U);

		// Do Dummy Read to clear of the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1U);

		// Send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1U);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1U);

		uint8_t message[] = "Hello! How are you ??";

		if (SPI_VerifyResponse(ackbyte))
		{
			// Send Argument
			args[0U] = strlen((char*)message);

			SPI_SendData(SPI2, args, 1U);

			// Send Message
			SPI_SendData(SPI2, message, args[0U]);

			//printf("COMMAND_PRINT Executed. \n");
		}
		// End of COMMAND_PRINT

		// 5. Start of COMMAND_ID_READ

		// Wait till button is pressed
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		CommandCode = COMMAND_ID_READ;
		// Send Command
		/* This transmission of 1 byte resulted 1 garbage byte collection in Rx buffer of the master
		    and RXNE flag is set. So do the dummy read and clear the flag. */
		SPI_SendData(SPI2, &CommandCode, 1U);

		// Do Dummy Read to clear of the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1U);

		// Send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1U);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1U);

	    uint8_t id[10];
	    uint8_t i = 0U;

		if (SPI_VerifyResponse(ackbyte))
		{
			// Read 10 bytes id from the slave
			for(i = 0; i < 10U; i++)
			{
				// Send some dummy bits (1byte) to fetch data from the slave
				SPI_SendData(SPI2, &dummy_write, 1U);
				SPI_ReceiveData(SPI2, &id[i], 1U);
			}

			id[11U] = '\0';

			//printf("COMMAND ID : %s \n",id);
		}
		// End of COMMAND_ID_READ

		// Lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		//printf("SPI Communication Closed.");
	}

	return 0U;
}
