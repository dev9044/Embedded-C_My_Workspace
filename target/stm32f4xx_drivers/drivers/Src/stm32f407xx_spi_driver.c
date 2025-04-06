/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Dec 15, 2024
 *      Author: HP
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock Setup
 */
/********************************************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * brief		- This function enables or disables peripheral clock for the given SPI Peripheral
 *
 * @param[in]	- base address of the spi peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	// Clock Enable
	if (ENABLE == EnorDi)
	{
		if (SPI1 == pSPIx)
		{
		  SPI1_PCLK_EN();
		}
		else if (SPI2 == pSPIx)
		{
		  SPI2_PCLK_EN();
		}
		else if (SPI3 == pSPIx)
		{
		  SPI3_PCLK_EN();
		}
		else if (SPI4 == pSPIx)
		{
		  SPI4_PCLK_EN();
		}
		else if (SPI5 == pSPIx)
		{
		  SPI5_PCLK_EN();
		}
		else if (SPI6 == pSPIx)
		{
		  SPI6_PCLK_EN();
		}
	}
	// Clock Disable
	else
	{
		if (SPI1 == pSPIx)
		{
		  SPI1_PCLK_DI();
		}
		else if (SPI2 == pSPIx)
		{
		  SPI2_PCLK_DI();
		}
		else if (SPI3 == pSPIx)
		{
		  SPI3_PCLK_DI();
		}
		else if (SPI4 == pSPIx)
		{
		  SPI4_PCLK_DI();
		}
		else if (SPI5 == pSPIx)
		{
		  SPI5_PCLK_DI();
		}
		else if (SPI6 == pSPIx)
		{
		  SPI6_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/********************************************************************************************
 * @fn			- SPI_Init
 *
 * brief		- This function perform SPI Initialization
 *
 * @param[in]	- base address of the spi handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Configure the clock for SPI Peripheral
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// First lets configure the SPI_CR1 Register
	uint32_t tempreg = 0U;	// temp. register

	// Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// Configure the bus config
	if (SPI_BUS_CONFIG_FD == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (SPI_BUS_CONFIG_HD == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		// BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (SPI_BUS_CONFIG_SIMPLEX_RXONLY == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// Configure the DFF (Data Frame Format)
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// Configure the CPOL (Clock Polarity)
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// Configure the CPHA (Clock Phase)
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/********************************************************************************************
 * @fn			- SPI_DeInit
 *
 * brief		- This function perform SPI DeInitialization
 *
 * @param[in]	- base address of the spi peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	// Reset all registers of SPIx Port
	if (SPI1 == pSPIx)
	{
	  SPI1_REG_RESET();
	}
	else if (SPI2 == pSPIx)
	{
	  SPI2_REG_RESET();
	}
	else if (SPI3 == pSPIx)
	{
	  SPI3_REG_RESET();
	}
	else if (SPI4 == pSPIx)
	{
	  SPI4_REG_RESET();
	}
	else if (SPI5 == pSPIx)
	{
	  SPI5_REG_RESET();
	}
	else if (SPI6 == pSPIx)
	{
	  SPI6_REG_RESET();
	}
}

/********************************************************************************************
 * @fn			- SPI_SendData
 *
 * brief		- This function perform Send Data to SPI Peripheral
 *
 * @param[in]	- base address of the spi peripheral
 * @param[in]	- TxBuffer
 * @param[in]	- Len
 *
 * @return		- none
 *
 * @Note		- This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (0U < Len)
	{
		// Wait until TXE is set
		while(FLAG_RESET == SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)); // Here we are polling for the TXE flag to SET

		// Check the DFF bit in CR1
		if (pSPIx->CR1 & (1U << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// Load the data into the DR (Data Register)
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2U;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			// Load the data into the DR (Data Register)
			pSPIx->DR = *pTxBuffer;
			Len -= 1U;
			pTxBuffer++;
		}
	}
}

/********************************************************************************************
 * @fn			- SPI_ReceiveData
 *
 * brief		- This function perform Receive Data from SPI Peripheral
 *
 * @param[in]	- base address of the spi peripheral
 * @param[in]	- RxBuffer
 * @param[in]	- Len
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (0U < Len)
	{
		// Wait until RXNE is set
		while(FLAG_RESET == SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)); // Here we are polling for the RXNE flag to SET

		// Check the DFF bit in CR1
		if (pSPIx->CR1 & (1U << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// Load the data from the DR (Data Register) into RX Buffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2U;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			// 8 bit DFF
			// Load the data from the DR (Data Register) into RX Buffer
			*pRxBuffer = pSPIx->DR;
			Len -= 1U;
			pRxBuffer++;
		}
	}
}

/********************************************************************************************
 * @fn			- SPI_GetFlagStatus
 *
 * brief		- This function returns Get Flag Status of SPI Peripheral
 *
 * @param[in]	- base address of the spi peripheral
 * @param[in]	- FlagName
 *
 * @return		- FlagStatus
 *
 * @Note		- none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	uint8_t FlagStatus = 0U;

	if (pSPIx->SR & FlagName)
	{
		FlagStatus =  FLAG_SET;
	}
	else
	{
		FlagStatus = FLAG_RESET;
	}

	return FlagStatus;
}

/********************************************************************************************
 * @fn			- SPI_PeripheralControl
 *
 * brief		- This function controls SPI Peripheral
 *
 * @param[in]	- base address of the spi peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		pSPIx->CR1 |= (1U << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
	}
}

/********************************************************************************************
 * @fn			- SPI_SSIConfig
 *
 * brief		- This function configure SSI (Internal Slave Select) bit
 *
 * @param[in]	- base address of the spi peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		pSPIx->CR1 |= (1U << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
	}
}

/********************************************************************************************
 * @fn			- SPI_SSOEConfig
 *
 * brief		- This function configure SSOE (Slave Select Output Enable) bit
 *
 * @param[in]	- base address of the spi peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		pSPIx->CR2 |= (1U << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1U << SPI_CR2_SSOE);
	}
}

/********************************************************************************************
 * @fn			- SPI_SendDataIT
 *
 * brief		- This function perform SPI Send Data IT
 *
 * @param[in]	- base address of the spi handler
 * @param[in]	- TxBuffer
 * @param[in]	- Len
 *
 * @return		- State
 *
 * @Note		- This is blocking call
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t State = pSPIHandle->TxState;

	if (SPI_BUSY_IN_TX != State)
	{
		// 1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		/* 2. Mark the SPI state as busy in transmission so that no other code can take over
			   same SPI peripheral until transmission is over */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return State;
}

/********************************************************************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * brief		- This function perform SPI Receive Data IT
 *
 * @param[in]	- base address of the spi handler
 * @param[in]	- RxBuffer
 * @param[in]	- Len
 *
 * @return		- State
 *
 * @Note		- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t State = pSPIHandle->RxState;

	if (SPI_BUSY_IN_RX != State)
	{
		// 1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		/* 2. Mark the SPI state as busy in reception so that no other code can take over
			   same SPI peripheral until reception is over */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return State;
}

/*
 * IRQ Configuration and ISR Handling
 */
/********************************************************************************************
 * @fn			- SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		if (31U >= IRQNumber) // 0 to 31
		{
			// Program ISER0 (Interrupt Set-Enable Register 0) Register
			*NVIC_ISER0 |= (1U << IRQNumber);
		}
		else if ((31U < IRQNumber) && (64U > IRQNumber)) // 32 to 63
		{
			// Program ISER1 (Interrupt Set-Enable Register 1) Register
			*NVIC_ISER1 |= (1U << (IRQNumber % 32U));
		}
		else if ((64U <= IRQNumber) && (96U > IRQNumber)) // 64 to 95
		{
			// Program ISER2 (Interrupt Set-Enable Register 2) Register
			*NVIC_ISER2 |= (1U << (IRQNumber % 64U));
		}
	}
	else
	{
		if (31U >= IRQNumber) // 0 to 31
		{
			// Program ICER0 (Interrupt Clear-Enable Register 0) Register
			*NVIC_ICER0 |= (1U << IRQNumber);
		}
		else if ((31U < IRQNumber) && (64U > IRQNumber)) // 32 to 63
		{
			// Program ICER1 (Interrupt Clear-Enable Register 1) Register
			*NVIC_ICER1 |= (1U << (IRQNumber % 32U));
		}
		else if ((64U <= IRQNumber) && (96U > IRQNumber)) // 64 to 95
		{
			// Program ICER2 (Interrupt Clear-Enable Register 2) Register
			*NVIC_ICER2 |= (1U << (IRQNumber % 64U));
		}
	}
}

/********************************************************************************************
 * @fn			- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find out the IPR (Interrupt Priority Register) Register
	uint8_t iprx = IRQNumber / 4U;			// It decides that IPR (Interrupt Priority Register)
	uint8_t iprx_section = IRQNumber % 4U;	// It decides that IPR (Interrupt Priority Register) Section

	uint8_t shift_amount = (8U * iprx_section) + (8U - NO_OF_PR_BITS_IMPLEMENTED); // Lower 4 bits are not applicable for IPR (Interrupt Priority Register)
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/********************************************************************************************
 * @fn			- SPI_IRQHandling
 *
 * brief		- This function perform SPI Interrupt Handling
 *
 * @param[in]	- base address of the spi handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1 = 0U;
	uint8_t temp2 = 0U;

	// First lets check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<< SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// Lets check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<< SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		// Handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// Lets check for OVR Flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<< SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		// Handle OVR Error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

// Some helper functions implementation
/********************************************************************************************
 * @fn			- spi_txe_interrupt_handle
 *
 * brief		- This function handles the TXE interrupt
 *
 * @param[in]	- base address of the spi handler
 *
 * @return		- none
 *
 * @Note		- none
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
	{
		// 16 bit DFF
		// Load the data into the DR (Data Register)
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2U;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		// 8 bit DFF
		// Load the data into the DR (Data Register)
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen -= 1U;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen)
	{
		/* TxLen is zero, so close the spi transmission and inform the application that
		    Tx is over. */

		// This prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/********************************************************************************************
 * @fn			- spi_txe_interrupt_handle
 *
 * brief		- This function handles the RXNE interrupt
 *
 * @param[in]	- base address of the spi handler
 *
 * @return		- none
 *
 * @Note		- none
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
	{
		// 16 bit DFF
		// Load the data from the DR (Data Register) into RX Buffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2U;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}
	else
	{
		// 8 bit DFF
		// Load the data from the DR (Data Register) into RX Buffer
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 1U;
		pSPIHandle->pRxBuffer--;
	}

	if (!pSPIHandle->RxLen)
	{
		// Reception is complete

		// Lets turn off the rxneie interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/********************************************************************************************
 * @fn			- spi_txe_interrupt_handle
 *
 * brief		- This function handles the OVR ERR interrupt
 *
 * @param[in]	- base address of the spi handler
 *
 * @return		- none
 *
 * @Note		- none
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp = 0U;
	// Clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void)temp;
	// Inform to application
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/********************************************************************************************
 * @fn			- SPI_CloseTransmission
 *
 * brief		- This function closed the SPI transmission
 *
 * @param[in]	- base address of the spi handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0U;
	pSPIHandle->TxState = SPI_READY;
}

/********************************************************************************************
 * @fn			- SPI_CloseReception
 *
 * brief		- This function closed the SPI Reception
 *
 * @param[in]	- base address of the spi handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_RXNEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0U;
	pSPIHandle->TxState = SPI_READY;
}

/********************************************************************************************
 * @fn			- SPI_ClearOVRFlag
 *
 * brief		- This function clears the OVR FLag
 *
 * @param[in]	- base address of the spi register
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp = 0U;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t ApplEvent)
{
	// This is a weak implementation. The application may override this function.
}
