/*
 * 013i2c_slave_tx_string.c
 *
 *  Created on: Mar 29, 2025
 *      Author: HP
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_gpio_driver.h"

#define SLAVE_ADDR  0x68

#define MY_ADDR		SLAVE_ADDR

I2C_Handle_t I2C1Handle = {0U};

//some data
uint8_t Tx_buff[32] = "STM32 Slave modetesting..";

/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */

void delay(void)
{
	// This will introduce ~200ms delay when system clock is 16MHz
	for (uint32_t i = 0; i < 500000/2; i++);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins = {0U};

	memset(&I2CPins,0U,sizeof(I2CPins));

	/*Note : Internal pull-up resistors are used */

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_TYPE_OD;

	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;

	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C_Handle_t I2C1Handle = {0U};

	memset(&I2C1Handle,0U,sizeof(I2C1Handle));

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn = {0U};

	memset(&GPIOBtn,0U,sizeof(GPIOBtn));

	// This is Btn GPIO Configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main (void)
{
	GPIO_ButtonInit();

	//I2C Pin Inits
	I2C1_GPIOInits();

	//I2C Peripheral Configuration
	I2C1_Inits();

	//I2C IRQ Configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	//Enable the I2C Peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//Ack bit is made 1 after PE = 1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1);

}

void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if (I2C_EV_DATA_REQ == AppEv)
	{
		//Master wants some data. slave has to send it
		if (commandCode == 0x51)
		{
			//send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char*)Tx_buff));
		}
		else if (commandCode == 0x52)
		{
			//Send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx,Tx_buff[Cnt++]);

		}
	}
	else if (I2C_EV_DATA_RCV == AppEv)
	{
		//Data is waiting for the slave to read . slave has to read it
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}
	else if (I2C_ERROR_AF == AppEv)
	{
		//This happens only during slave txing .
		//Master has sent the NACK. so slave should understand that master doesnt need
		//more data.
		commandCode = 0xff;
		Cnt = 0;
	}
	else if (I2C_EV_STOP == AppEv)
	{
		//This happens only during slave reception .
		//Master has ended the I2C communication with the slave.
	}
}

