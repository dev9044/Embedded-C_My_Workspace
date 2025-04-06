/*
 * STM32F407xx_usart_driver.h
 *
 *  Created on: 31-Mar-2025
 *      Author: Dev Tandon
 */

#ifndef STM32F446X_UART_DRIVER_H_
#define STM32F446X_UART_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for USARTx (Universal Synchronous and Asynchronous Receiver-Transmitter x) Peripheral
 */
typedef struct
{
	uint8_t USART_Mode;				/* Possible values from @USART_Mode */
	uint32_t USART_Baud;			/* Possible values from @USART_Baud */
	uint8_t USART_NoOfStopBits;		/* Possible values from @USART_NoOfStopBits */
	uint8_t USART_WordLength;		/* Possible values from @USART_WordLength */
	uint8_t USART_ParityControl;	/* Possible values from @USART_ParityControl */
	uint8_t USART_HWFlowControl;	/* Possible values from @USART_HWFlowControl */
}USART_Config_t;

/*
 * This is a Handle structure for USARTx (Universal Synchronous and Asynchronous Receiver-Transmitter x) Peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;			/* This holds the base address of the USARTx(x:1,2,3,4,5,6) Peripheral */
	USART_Config_t   USART_Config;		/* This holds USARTx Peripheral Configuration Settings */
	uint8_t *pTxBuffer;					/* To Store the app. Tx Buffer address*/
	uint8_t *pRxBuffer;					/* To Store the app. Rx Buffer address*/
	uint32_t TxLen;						/* To Store the Tx Len*/
	uint32_t RxLen;						/* To Store the Rx Len*/
	uint8_t TxBusyState;				/* To Store the Tx State*/
	uint8_t RxBusyState;				/* To Store the Rx State*/
}USART_Handle_t;

/*
 * @USART_Mode
 * Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 			(0U)
#define USART_MODE_ONLY_RX 			(1U)
#define USART_MODE_TXRX  			(2U)

/*
 * @USART_Baud
 * Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					(1200U)
#define USART_STD_BAUD_2400					(400U)
#define USART_STD_BAUD_9600					(9600U)
#define USART_STD_BAUD_19200 				(19200U)
#define USART_STD_BAUD_38400 				(38400U)
#define USART_STD_BAUD_57600 				(57600U)
#define USART_STD_BAUD_115200 				(115200U)
#define USART_STD_BAUD_230400 				(230400U)
#define USART_STD_BAUD_460800 				(460800U)
#define USART_STD_BAUD_921600 				(921600U)
#define USART_STD_BAUD_2M 					(2000000U)
#define SUART_STD_BAUD_3M 					(3000000U)

/*
 * @USART_ParityControl
 * Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD    (2U)
#define USART_PARITY_EN_EVEN   (1U)
#define USART_PARITY_DISABLE   (0U)

/*
 * @USART_WordLength
 * Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  	(0U)
#define USART_WORDLEN_9BITS  	(1U)

/*
 * @USART_NoOfStopBits
 * Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     	(0U)
#define USART_STOPBITS_0_5   	(1U)
#define USART_STOPBITS_2     	(2U)
#define USART_STOPBITS_1_5   	(3U)

/*
 * @USART_HWFlowControl
 * Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    		(0U)
#define USART_HW_FLOW_CTRL_CTS    		(1U)
#define USART_HW_FLOW_CTRL_RTS    		(2U)
#define USART_HW_FLOW_CTRL_CTS_RTS		(3U)

/*
 * USART flags
 */
#define USART_FLAG_TXE 			(1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		(1 << USART_SR_RXNE)
#define USART_FLAG_TC 			(1 << USART_SR_TC)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 		(1U)
#define USART_BUSY_IN_TX 		(2U)
#define USART_READY 			(0U)

#define USART_EVENT_TX_CMPLT   	(0U)
#define	USART_EVENT_RX_CMPLT   	(1U)
#define	USART_EVENT_IDLE      	(2U)
#define	USART_EVENT_CTS       	(3U)
#define	USART_EVENT_PE        	(4U)
#define	USART_ERR_FE     		(5U)
#define	USART_ERR_NE    	 	(6U)
#define	USART_ERR_ORE    		(7U)

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ (Interrupt Request) Configuration and ISR (Interrupt Service Routine) Handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv);

#endif /* STM32F446X_UART_DRIVER_H_ */
