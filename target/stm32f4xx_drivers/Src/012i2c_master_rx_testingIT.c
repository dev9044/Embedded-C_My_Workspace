/*
 * 012i2c_master_rx_testingIT.c
 *
 *  Created on: Mar 22, 2025
 *      Author: HP
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_gpio_driver.h"

#define MY_ADDR 	0x61

#define SLAVE_ADDR  0x68

I2C_Handle_t I2C1Handle = {0U};

//Flag variable
uint8_t rxComplt = RESET;

//some data
uint8_t rcv_buff[32];

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
	uint8_t CommandCode = 0U;
	uint8_t Len = 0U;

	GPIO_ButtonInit();

	//I2C Pin Inits
	I2C1_GPIOInits();

	//I2C Peripheral Configuration
	I2C1_Inits();

	//I2C IRQ Configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	//Enable the I2C Peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//Ack bit is made 1 after PE = 1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		CommandCode = 0x51;

		while(I2C_MasterSendDataIT(&I2C1Handle,&CommandCode,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle,&Len,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		CommandCode = 0x52;

		while(I2C_MasterSendDataIT(&I2C1Handle,&CommandCode,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle,&rcv_buff[0],Len,SLAVE_ADDR,I2C_DISABLE_SR) != I2C_READY);

		rxComplt = RESET;

		//wait till rx completes
		while (rxComplt != SET)
		{

		}

		rcv_buff[Len+1] = '\0';

		rxComplt = RESET;
	}
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
	if (I2C_EV_TX_CMPLT == AppEv)
	{
		//printf("Tx is completed\n");
	}
	else if (I2C_EV_RX_CMPLT == AppEv)
	{
		//printf("Rx is completed\n");
		rxComplt = SET;
	}
	else if (I2C_ERROR_AF == AppEv)
	{
		//printf("Error : Ack failure\n");
		//in master ack failure happens when slave fails to send ack for the byte
		//sent from the master.
		I2C_CloseSendData(pI2CHandle);

		//generate the stop condition to release the bus
		I2C_GenerateStopCondition(I2C1);

		//Hang in infinite loop
		while(1);
	}
}
