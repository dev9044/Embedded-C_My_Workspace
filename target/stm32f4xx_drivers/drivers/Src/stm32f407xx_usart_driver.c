/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Mar 31, 2025
 *      Author: HP
 */

#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - This function sets the USART baud rate
 *
 * @param[in]         - base address of USART register
 * @param[in]         - baud rate
 *
 * @return            - none
 *
 * @Note              - none

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	// Variable to hold the APB clock
	uint32_t PCLKx = 0U;

	uint32_t usartdiv = 0U;

	// variables to hold Mantissa and Fraction values
	uint32_t M_part = 0U;
	uint32_t F_part = 0U;

	uint32_t tempreg = 0U;

	// Get the value of APB bus clock in to the variable PCLKx
	if ((USART1 == pUSARTx)|| (USART6 == pUSARTx))
	{
	   // USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
	   PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   // OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}
	else
	{
	   // over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	// Calculate the Mantissa part
	M_part = usartdiv/100;

	// Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	// Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	// Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	  // OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100) & ((uint8_t)0x07);

	}
	else
	{
	   // over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	// Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	// copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*
 * Init and De-init
 */
/********************************************************************************************
 * @fn			- USART_Init
 *
 * brief		- This function performs USART Initialization
 *
 * @param[in]	- base address of the usart handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Temporary variable
	uint32_t tempreg = 0U;

/******************************** Configuration of CR1******************************************/

	// Implement the code to enable the Clock for given USART peripheral
	 USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (USART_MODE_ONLY_RX == pUSARTHandle->USART_Config.USART_Mode)
	{
		// Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	}
	else if (USART_MODE_ONLY_TX == pUSARTHandle->USART_Config.USART_Mode)
	{
		// Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	}
	else if (USART_MODE_TXRX == pUSARTHandle->USART_Config.USART_Mode)
	{
		// Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

    // Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    // Configuration of parity control bit fields
	if (USART_PARITY_EN_EVEN == pUSARTHandle->USART_Config.USART_ParityControl)
	{
		// Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		// Implement the code to enable EVEN parity
		// Not required because by default EVEN parity will be selected once you enable the parity control
	}
	else if (USART_PARITY_EN_ODD == pUSARTHandle->USART_Config.USART_ParityControl)
	{
		// Implement the code to enable the parity control
	    tempreg |= (1 << USART_CR1_PCE);

	    // Implement the code to enable ODD parity
	    tempreg |= (1 << USART_CR1_PS);
	}

    // Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg = 0U;

	// Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	// Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg = 0U;

	// Configuration of USART hardware flow control
	if (USART_HW_FLOW_CTRL_CTS == pUSARTHandle->USART_Config.USART_HWFlowControl)
	{
		// Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if (USART_HW_FLOW_CTRL_RTS == pUSARTHandle->USART_Config.USART_HWFlowControl)
	{
		// Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if (USART_HW_FLOW_CTRL_CTS_RTS == pUSARTHandle->USART_Config.USART_HWFlowControl)
	{
		// Implement the code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	// Implement the code to configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
}

/*
 * Peripheral Clock Setup
 */
/********************************************************************************************
 * @fn			- USART_PeriClockControl
 *
 * brief		- This function enables or disables peripheral clock for the given USART Peripheral
 *
 * @param[in]	- base address of the usart peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	// Clock Enable
	if (ENABLE == EnorDi)
	{
		if (USART1 == pUSARTx)
		{
			USART1_PCLK_EN();
		}
		else if (USART2 == pUSARTx)
		{
			USART2_PCLK_EN();
		}
		else if (USART3 == pUSARTx)
		{
			USART3_PCLK_EN();
		}
		else if (UART4 == pUSARTx)
		{
			UART4_PCLK_EN();
		}
	}
	// Clock Disable
	else
	{
		if (USART1 == pUSARTx)
		{
			USART1_PCLK_DI();
		}
		else if (USART2 == pUSARTx)
		{
			USART2_PCLK_DI();
		}
		else if (USART3 == pUSARTx)
		{
			USART3_PCLK_DI();
		}
		else if (UART4 == pUSARTx)
		{
			UART4_PCLK_DI();
		}
	}
}

/********************************************************************************************
 * @fn			- USART_PeripheralControl
 *
 * brief		- This function controls USART Peripheral
 *
 * @param[in]	- base address of the usart peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		pUSARTx->CR1 |= (1 << 13);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

/********************************************************************************************
 * @fn			- USART_GetFlagStatus
 *
 * brief		- This function returns Get Flag Status of USART Peripheral
 *
 * @param[in]	- base address of the usart peripheral
 * @param[in]	- FlagName
 *
 * @return		- FlagStatus
 *
 * @Note		- none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
	uint8_t FlagStatus = 0U;

    if (pUSARTx->SR & StatusFlagName)
    {
    	FlagStatus = SET;
    }
    else
    {
    	FlagStatus = RESET;
    }

   return FlagStatus;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - This function sends the USART data
 *
 * @param[in]         - base address of usart handler
 * @param[in]         - pTxBuffer
 * @param[in]         - Len
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

    // Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0 ; i < Len; i++)
	{
		// Implement the code to wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		// Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (USART_WORDLEN_9BITS == pUSARTHandle->USART_Config.USART_WordLength)
		{
			// if 9BIT load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			// check for USART_ParityControl
			if (USART_PARITY_DISABLE == pUSARTHandle->USART_Config.USART_ParityControl)
			{
				// No parity is used in this transfer , so 9bits of user data will be sent
				// Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// Parity bit is used in this transfer . so 8bits of user data will be sent
				// The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			// This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			// Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	// Implement the code to wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - This function receives the USART data
 *
 * @param[in]         - base address of usart handler
 * @param[in]         - pRxBuffer
 * @param[in]         - Len
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   // Loop over until "Len" number of bytes are received
	for (uint32_t i = 0 ; i < Len; i++)
	{
		// Implement the code to wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		// Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (USART_WORDLEN_9BITS == pUSARTHandle->USART_Config.USART_WordLength)
		{
			// We are going to receive 9bit data in a frame

			// Now, check are we using USART_ParityControl control or not
			if (USART_PARITY_DISABLE == pUSARTHandle->USART_Config.USART_ParityControl)
			{
				// No parity is used , so all 9bits will be of user data

				// read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

				// Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				// Parity is used, so 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			// We are going to receive 8bit data in a frame

			// Now, check are we using USART_ParityControl control or not
			if (USART_PARITY_DISABLE == pUSARTHandle->USART_Config.USART_ParityControl)
			{
				// No parity is used , so all 8bits will be of user data

				// read 8 bits from DR
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				// Parity is used, so , 7 bits will be of user data and 1 bit is parity

				// read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			// Now , increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - This function sends the USART data IT
 *
 * @param[in]         - base address of usart handler
 * @param[in]         - pTxBuffer
 * @param[in]         - Len
 *
 * @return            - txstate
 *
 * @Note              - none
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if (USART_BUSY_IN_TX != txstate)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - This function receives the USART data IT
 *
 * @param[in]         - base address of usart handler
 * @param[in]         - pRxBuffer
 * @param[in]         - Len
 *
 * @return            - rxstate
 *
 * @Note              - none
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (USART_BUSY_IN_RX != rxstate)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		// Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - This fumction clears the USART Flag
 *
 * @param[in]         - base address of usart register
 * @param[in]         - StatusFlagName
 *
 * @return            - none
 *
 * @Note              - Applicable to only USART_CTS_FLAG , USART_LBD_FLAG
 * 						USART_TC_FLAG, flags
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);
}

/*
 * IRQ Configuration and ISR Handling
 */
/********************************************************************************************
 * @fn			- USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
		if (31 >= IRQNumber) // 0 to 31
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
 * @fn			- USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	// Find out the IPR (Interrupt Priority Register) Register
	uint8_t iprx = IRQNumber / 4;				// It decides that IPR (Interrupt Priority Register)
	uint8_t iprx_section  = IRQNumber %4 ;		// It decides that IPR (Interrupt Priority Register) Section

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED);	// Lower 4 bits are not applicable for IPR (Interrupt Priority Register)
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/********************************************************************************************
 * @fn			- USART_IRQHandling
 *
 * brief		- This function perform USART Interrupt Handling
 *
 * @param[in]	- base address of the usart handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 = 0U;
	uint32_t temp2 = 0U;

	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    // Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	 // Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if (temp1 && temp2)
	{
		// this interrupt is because of TC

		// close transmission and call application callback if TxLen is zero
		if (USART_BUSY_IN_TX == pUSARTHandle->TxBusyState)
		{
			// Check the TxLen . If it is zero then close the data transmission
			if (!pUSARTHandle->TxLen)
			{
				// Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				// Implement the code to clear the TCEIE control bit

				// Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				// Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				// Reset the length to zero
				pUSARTHandle->TxLen = 0U;

				// Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	// Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	// Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);


	if (temp1 && temp2)
	{
		// this interrupt is because of TXE

		if (USART_BUSY_IN_TX == pUSARTHandle->TxBusyState)
		{
			// Keep sending data until Txlen reaches to zero
			if (0U < pUSARTHandle->TxLen)
			{
				// Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if (USART_WORDLEN_9BITS == pUSARTHandle->USART_Config.USART_WordLength)
				{
					// if 9BIT load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					// check for USART_ParityControl
					if (USART_PARITY_DISABLE == pUSARTHandle->USART_Config.USART_ParityControl)
					{
						// No parity is used in this transfer , so 9bits of user data will be sent
						// Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						// Parity bit is used in this transfer. so 8bits of user data will be sent
						// The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 1;
					}
				}
				else
				{
					// This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					// Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
			}

			if (0U == pUSARTHandle->TxLen)
			{
				// TxLen is zero
				// Implement the code to clear the TXEIE bit (disable interrupt for TXE flag)
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2)
	{
		// this interrupt is because of rxne
		if (USART_BUSY_IN_RX == pUSARTHandle->RxBusyState)
		{
			if (0U < pUSARTHandle->RxLen)
			{
				// Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if (USART_WORDLEN_9BITS == pUSARTHandle->USART_Config.USART_WordLength)
				{
					// We are going to receive 9bit data in a frame

					// Now, check are we using USART_ParityControl control or not
					if (USART_PARITY_DISABLE == pUSARTHandle->USART_Config.USART_ParityControl)
					{
						// No parity is used , so all 9bits will be of user data

						// read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						// Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						// Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen -= 1;
					}
				}
				else
				{
					// We are going to receive 8bit data in a frame

					// Now, check are we using USART_ParityControl control or not
					if (USART_PARITY_DISABLE == pUSARTHandle->USART_Config.USART_ParityControl)
					{
						// No parity is used , so all 8bits will be of user data

						// read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}
					else
					{
						// Parity is used, so 7 bits will be of user data and 1 bit is parity

						// read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}

					// Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}
			}

			if (!pUSARTHandle->RxLen)
			{
				// disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}

/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	// Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	// Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	// Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	//temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if (temp1 && temp2)
	{
		// Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS);

		// this interrupt is because of CTS
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	// Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	// Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if (temp1 && temp2)
	{
		// Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		// this interrupt is because of IDLE
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	// Implement the code to check the status of ORE flag in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	// Implement the code to check the status of RXNEIE bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if (temp1 && temp2)
	{
		// Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		// this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}

/*************************Check for Error Flag ********************************************/

// Noise Flag, Overrun error and Framing Error in multibuffer communication
// We dont discuss multibuffer communication in this course. please refer to the RM
// The below code will get executed in only if multibuffer mode is used.

	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE) ;

	if (temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;

		if (temp1 & (1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if (temp1 & (1 << USART_SR_NE))
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{
	// This is a weak implementation. The application may override this function.
}
