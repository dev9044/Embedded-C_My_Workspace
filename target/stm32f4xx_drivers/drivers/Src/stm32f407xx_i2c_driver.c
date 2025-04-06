/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jan 25, 2025
 *      Author: HP
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/*********************************************************************
 * @fn      		  - I2C_GenerateStartCondition
 *
 * @brief             - This function generates the start condition
 *
 * @param[in]         - base address of the i2c register
 *
 * @return            - none
 *
 * @Note              - none
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1U << I2C_CR1_START);
}

/*********************************************************************
 * @fn      		  - I2C_GenerateStopCondition
 *
 * @brief             - This function generates the stop condition
 *
 * @param[in]         - base address of the i2c register
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1U << I2C_CR1_STOP);
}

/*********************************************************************
 * @fn      		  - I2C_SlaveEnableDisableCallbackEvents
 *
 * @brief             - This function enables and disables call back events for slave
 *
 * @param[in]         - base address of the i2c register
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->CR2 &= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= (1 << I2C_CR2_ITERREN);
	}
}

/*********************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseWrite
 *
 * @brief             - This function executes the address phase for write
 *
 * @param[in]         - base address of the i2c register
 * @param[in]         - SlaveAddr
 *
 * @return            - none
 *
 * @Note              - none
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1U;
	SlaveAddr &= ~(1U); // SlaveAddr is Slave address + r/nw bit = 0
	pI2Cx->DR = SlaveAddr;
}

/*********************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseRead
 *
 * @brief             - This function executes the address phase for read
 *
 * @param[in]         - base address of the i2c register
 * @param[in]         - SlaveAddr
 *
 * @return            - none
 *
 * @Note              - none
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1U;
	SlaveAddr |= (1U); // SlaveAddr is Slave address + r/nw bit = 1
	pI2Cx->DR = SlaveAddr;
}

/*********************************************************************
 * @fn      		  - I2C_ClearADDRFlag
 *
 * @brief             - This function clears the ADDR flag
 *
 * @param[in]         - base address of the i2c handler
 *
 * @return            - none
 *
 * @Note              - none
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read = 0U;

	// check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1U << I2C_SR2_MSL))
	{
		// device is in master mode
		if (I2C_BUSY_IN_RX == pI2CHandle->TxRxState)
		{
			if (1U == pI2CHandle->RxSize)
			{
				// first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				// clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			// clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		// device is in slave mode
		// clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

/*********************************************************************
 * @fn      		  - I2C_ManageAcking
 *
 * @brief             - This function manages the acking
 *
 * @param[in]         - base address of the i2c register
 * @param[in]		  - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (I2C_ACK_ENABLE == EnorDi)
	{
		// enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		// disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*
 * Peripheral Clock Setup
 */
/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C Peripheral
 *
 * @param[in]         - base address of the i2c peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	// Clock Enable
	if (ENABLE == EnorDi)
	{
		if (I2C1 == pI2Cx)
		{
		   I2C1_PCLK_EN();
		}
		else if (I2C2 == pI2Cx)
		{
		   I2C2_PCLK_EN();
		}
		else if (I2C3 == pI2Cx)
		{
		   I2C3_PCLK_EN();
		}
	}
	// Clock Disable
	else
	{
		if (I2C1 == pI2Cx)
		{
		   I2C1_PCLK_DI();
		}
		else if (I2C2 == pI2Cx)
		{
		   I2C2_PCLK_DI();
		}
		else if (I2C3 == pI2Cx)
		{
		   I2C3_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - This function perform I2C Initialization
 *
 * @param[in]         - base address of the i2c handler
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0U; //temp. register

	// enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg = 0U;
	tempreg |= RCC_GetPCLK1Value() / 1000000U ;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

    // program the device own address
	tempreg = 0U;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD71;
	tempreg |= (1U << 14U);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR (Clock Control Register) calculations
	uint16_t ccr_value = 0U;
	tempreg = 0U;

	if (I2C_SCL_SPEED_SM >= pI2CHandle->I2C_Config.I2C_SCLSpeed)
	{
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2U*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// mode is fast mode
		tempreg |= (1U << 15U);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if (I2C_FM_DUTY_2 == pI2CHandle->I2C_Config.I2C_FMDutyCycle)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3U*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25U*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration
	if (I2C_SCL_SPEED_SM >= pI2CHandle->I2C_Config.I2C_SCLSpeed)
	{
		// mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1U ;

	}
	else
	{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300U) / 1000000000U) + 1U;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - This function perform I2C Deinitialization
 *
 * @param[in]         - base address of the i2c register
 *
 * @return            - none
 *
 * @Note              - none

 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	// Reset all registers of I2Cx Port
	if (I2C1 == pI2Cx)
	{
	  I2C1_REG_RESET();
	}
	else if (I2C2 == pI2Cx)
	{
	  I2C2_REG_RESET();
	}
	else if (I2C3 == pI2Cx)
	{
	  I2C3_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - This function controls the I2C Peripheral
 *
 * @param[in]         - base address of the i2c peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (ENABLE == EnOrDi)
	{
		pI2Cx->CR1 |= (1U << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1U << 0U);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - This function sends the data from master
 *
 * @param[in]         - base address of the i2c handler
 * @param[in]         - pTxbuffer
 * @param[in]         - Len
 * @param[in]         - SlaveAddr
 * @param[in]         - Sr
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. send the data until len becomes 0
	while (0U < Len)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// 7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: generating STOP, automatically clears the BTF
	if (I2C_DISABLE_SR == Sr)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - This function receives the data from slave
 *
 * @param[in]         - base address of the i2c handler
 * @param[in]         - pRxbuffer
 * @param[in]         - Len
 * @param[in]         - SlaveAddr
 * @param[in]         - Sr
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit set to w(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if (1U == Len)
	{
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// generate STOP condition
		if (I2C_DISABLE_SR == Sr)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	// procedure to read data from slave when Len > 1
	if (1U < Len)
	{
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// read the data until Len becomes zero
		for (uint32_t i = Len; i > 0U; i--)
		{
			// wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (2U == i) //if last 2 bytes are remaining
			{
				// Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// generate STOP condition
				if (I2C_DISABLE_SR == Sr)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from data register into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// increment the buffer address
			pRxBuffer++;
		}
	}

	// re-enable Acking
	if (I2C_ACK_ENABLE == pI2CHandle->I2C_Config.I2C_AckControl)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

/********************************************************************************************
 * @fn			- I2C_GetFlagStatus
 *
 * brief		- This function returns Get Flag Status of I2C Peripheral
 *
 * @param[in]	- base address of the i2c peripheral
 * @param[in]	- FlagName
 *
 * @return		- FlagStatus
 *
 * @Note		- none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	uint8_t FlagStatus = 0U;

	if (pI2Cx->SR1 & FlagName)
	{
		FlagStatus =  FLAG_SET;
	}
	else
	{
		FlagStatus = FLAG_RESET;
	}

	return FlagStatus;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - This function sends the data IT from master
 *
 * @param[in]         - base address of the i2c handler
 * @param[in]         - pTxbuffer
 * @param[in]         - Len
 * @param[in]         - SlaveAddr
 * @param[in]         - Sr
 *
 * @return            - busystate
 *
 * @Note              - none
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((I2C_BUSY_IN_TX != busystate) && (I2C_BUSY_IN_RX != busystate))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             - This function receives the data from slave
 *
 * @param[in]         - base address of the i2c handler
 * @param[in]         - pRxbuffer
 * @param[in]         - Len
 * @param[in]         - SlaveAddr
 * @param[in]         - Sr
 *
 * @return            - busystate
 *
 * @Note              - none
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((I2C_BUSY_IN_TX != busystate) && (I2C_BUSY_IN_RX != busystate))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterHandleTXEInterrupt
 *
 * @brief             - This function handles the TXE Interrupt for master
 *
 * @param[in]         - base address of the i2c handler
 *
 * @return            - none
 *
 * @Note              - none
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (0U < pI2CHandle->TxLen)
	{
		// 1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		// 2. decrement the TxLen
		pI2CHandle->TxLen--;

		// 3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterHandleRXNEInterrupt
 *
 * @brief             - This function handles the RXNE Interrupt for master
 *
 * @param[in]         - base address of the i2c handler
 *
 * @return            - none
 *
 * @Note              - none
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	// We have to do the data reception
	if (1U == pI2CHandle->RxSize)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if (1U < pI2CHandle->RxSize)
	{
		if (2U == pI2CHandle->RxLen)
		{
			// clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

		//read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (0U == pI2CHandle->RxLen)
	{
		// close the I2C data reception and notify the application
		// 1. generate the stop condition
		if (I2C_DISABLE_SR == pI2CHandle->Sr)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// 2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - I2C_CloseReceiveData
 *
 * @brief             - This function closed the receive data
 *
 * @param[in]         - base address of the i2c handler
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (I2C_ACK_ENABLE == pI2CHandle->I2C_Config.I2C_AckControl)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
}

/*********************************************************************
 * @fn      		  - I2C_CloseSendData
 *
 * @brief             - This function closed the send data
 *
 * @param[in]         - base address of the i2c handler
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief             - This function sends slave data
 *
 * @param[in]         - base address of the i2c register
 * @param[in]         - data
 *
 * @return            - none
 *
 * @Note              - none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->DR = data;
}

/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief             - This function receives slave data
 *
 * @param[in]         - base address of the i2c register
 *
 * @return            - none
 *
 * @Note              - none
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}

/********************************************************************************************
 * @fn			- I2C_EV_IRQHandling
 *
 * brief		- This function perform I2C Event Interrupt Handling
 *
 * @param[in]	- base address of i2c handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device
	uint32_t temp1 = 0U;
	uint32_t temp2 = 0U;
	uint32_t temp3 = 0U;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	// 1. Handle For interrupt generated by SB event
	// Note : SB flag is only applicable in Master mode
	if (temp1 && temp3)
	{
		// The interrupt is generated because of SB event
		// This block will not be executed in slave mode because for slave SB is always zero
		// In this block lets executed the address phase
		if (I2C_BUSY_IN_TX == pI2CHandle->TxRxState)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (I2C_BUSY_IN_RX == pI2CHandle->TxRxState)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	// 2. Handle For interrupt generated by ADDR event
	// Note : When master mode : Address is sent
	// When Slave mode : Address matched with own address
	if (temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	// 3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if (temp1 && temp3)
	{
		// BTF flag is set
		if (I2C_BUSY_IN_TX == pI2CHandle->TxRxState)
		{
			// make sure that TXE is also set .
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				// BTF, TXE = 1
				if (0U == pI2CHandle->TxLen)
				{
					// 1. generate the STOP condition
					if (I2C_DISABLE_SR == pI2CHandle->Sr)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// 2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					// 3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		}
		else if (I2C_BUSY_IN_RX == pI2CHandle->TxRxState)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	// 4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	// The below code block will not be executed by the master since STOPF will not set in master mode
	if (temp1 && temp3)
	{
		// STOPF flag is set
		// Clear the STOPF (i.e 1)read SR1 2)Write to CR1 )
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	// 5. Handle For interrupt generated by TXE event
	if (temp1 && temp2 && temp3)
	{
		// Check for device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// TXE flag is set
			// We have to do the data transmission
			if (I2C_BUSY_IN_TX == pI2CHandle->TxRxState)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// slave
			// make sure that the slave is really in transmitter mode
		    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	// 6. Handle For interrupt generated by RXNE event
	if (temp1 && temp2 && temp3)
	{
		// check device mode .
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// The device is master
			// RXNE flag is set
			if (I2C_BUSY_IN_RX == pI2CHandle->TxRxState)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// slave
			// make sure that the slave is really in receiver mode
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/********************************************************************************************
 * @fn			- I2C_ER_IRQHandling
 *
 * brief		- This function perform I2C Error Interrupt Handling
 *
 * @param[in]	- base address of i2c handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1 = 0U;
	uint32_t temp2 = 0U;

    // Know the status of ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1<< I2C_SR1_BERR);

	if (temp1 && temp2 )
	{
		// This is Bus error

		// Implement the code to clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Implement the code to notify the application about the error
	    I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error*************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);

	if (temp1 && temp2)
	{
		// This is arbitration lost error

		// Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	/***********************Check for ACK failure error******************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);

	if (temp1 && temp2)
	{
		// This is ACK failure error

	    // Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	/***********************Check for Overrun/Underrun error**************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);

	if (temp1 && temp2)
	{
		// This is Overrun/Underrun

	    // Implement the code to clear the Overrun/Underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	/***********************Check for Time out error**********************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);

	if (temp1 && temp2)
	{
		// This is Time out error

	    // Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

/*
 * IRQ Configuration and ISR Handling
 */
/********************************************************************************************
 * @fn			- I2C_IRQInterruptConfig
 *
 * brief		- This function contains configuration of IRQNumber
 *
 * @param[in]	- IRQNumber
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		if (31 >= IRQNumber) // 0 to 31
		{
			// Program ISER0 (Interrupt Set-Enable Register 0) Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if ((31 < IRQNumber) && (64 > IRQNumber)) // 32 to 63
		{
			// Program ISER0 (Interrupt Set-Enable Register 1) Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if ((64 <= IRQNumber) && (96 > IRQNumber)) // 64 to 95
		{
			// Program ISER0 (Interrupt Set-Enable Register 2) Register
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if (31 <= IRQNumber) // 0 to 31
		{
			// Program ICER0 (Interrupt Clear-Enable Register 0) Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if ((31 < IRQNumber) && (64 > IRQNumber)) // 32 to 63
		{
			// Program ICER0 (Interrupt Clear-Enable Register 1) Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if ((64 <= IRQNumber) && (96 > IRQNumber)) // 64 to 95
		{
			// Program ICER0 (Interrupt Clear-Enable Register 2) Register
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

/********************************************************************************************
 * @fn			- I2C_IRQPriorityConfig
 *
 * brief		- This function contains configuration of IRQPriority
 *
 * @param[in]	- IRQNumber
 * @param[in]	- IRQPriority
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	// Find out the IPR (Interrupt Priority Register) Register
	uint8_t iprx = IRQNumber / 4;					// It decides that IPR (Interrupt Priority Register)
	uint8_t iprx_section  = IRQNumber %4 ;			// It decides that IPR (Interrupt Priority Register) Section

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED);	// Lower 4 bits are not applicable for IPR (Interrupt Priority Register)
	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	// This is a weak implementation. The application may override this function.
}
