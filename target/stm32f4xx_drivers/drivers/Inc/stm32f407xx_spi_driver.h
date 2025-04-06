/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Dec 15, 2024
 *      Author: HP
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for SPIx (Serial Peripheral Interface x) Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;			/* Decide the Master mode and Slave Mode. Possible values from @SPI_DeviceMode*/
	uint8_t SPI_BusConfig;			/* Decide the Bus (FullDuplex, HalfDuplex and Simplex). Possible values from @SPI_BusConfig */
	uint8_t SPI_SclkSpeed;			/* Decide the SPI Clock Speed. Possible values from @SPI_SclkSpeed */
	uint8_t SPI_DFF;				/* Decide the Data Frame Format (8bit Data or 16bit Data). Possible values from @SPI_DFF */
	uint8_t SPI_CPOL;				/* Decide the Clock Polarity. Possible values from @SPI_CPOL */
	uint8_t SPI_CPHA;				/* Decide the Clock Phase. Possible values from @SPI_CPHA */
	uint8_t SPI_SSM;				/* Decide the Slave Select Management (Hardware or Software). Possible values from @SPI_SSM */
}SPI_Config_t;

/*
 * This is a Handle structure for SPIx (Serial Peripheral Interface x) Peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;		/* This holds the base address of the SPIx(x:1,2,3,4,5,6) Peripheral */
	SPI_Config_t SPIConfig;	    /* This holds SPIx Peripheral Configuration Settings */
	uint8_t *pTxBuffer;			/* To Store the app. Tx Buffer address*/
	uint8_t *pRxBuffer;			/* To Store the app. Rx Buffer address*/
	uint8_t TxLen;				/* To Store the Tx Len*/
	uint8_t RxLen;				/* To Store the Rx Len*/
	uint8_t TxState;			/* To Store the Tx State*/
	uint8_t RxState;			/* To Store the Rx State*/
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 * SPI possible Device Mode
 */
#define SPI_DEVICE_MODE_MASTER				(1U)
#define SPI_DEVICE_MODE_SLAVE			    (0U)

/*
 * @SPI_BusConfig
 * SPI possible Bus
 */
#define SPI_BUS_CONFIG_FD					(1U)
#define SPI_BUS_CONFIG_HD					(2U)
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		(3U)

/*
 * @SPI_SclkSpeed
 * SPI possible Clock Speed
 */
#define SPI_SCLK_SPEED_DIV2					(0U)
#define SPI_SCLK_SPEED_DIV4					(1U)
#define SPI_SCLK_SPEED_DIV8					(2U)
#define SPI_SCLK_SPEED_DIV16				(3U)
#define SPI_SCLK_SPEED_DIV32				(4U)
#define SPI_SCLK_SPEED_DIV64				(5U)
#define SPI_SCLK_SPEED_DIV128				(6U)
#define SPI_SCLK_SPEED_DIV256				(7U)

/*
 * @SPI_DFF
 * SPI possible Data Frame Format
 */
#define SPI_DFF_8BITS						(0U)
#define SPI_DFF_16BITS						(1U)

/*
 * @SPI_CPOL
 * SPI possible Clock Polarity
 */
#define SPI_CPOL_HIGH						(1U)
#define SPI_CPOL_LOW						(0U)

/*
 * @SPI_CPHA
 * SPI possible Clock Phase
 */
#define SPI_CPHA_HIGH						(1U)
#define SPI_CPHA_LOW						(0U)

/*
 * @SPI_SSM
 * SPI possible Slave Select Management
 */
#define SPI_SSM_EN							(1U)
#define SPI_SSM_DI							(0U)

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG		(1U << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1U << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1U << SPI_SR_BSY)

/*
 * Possible SPI Application States
 */
#define SPI_READY				(0U)
#define SPI_BUSY_IN_RX			(1U)
#define SPI_BUSY_IN_TX			(2U)

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT		(1U)
#define SPI_EVENT_RX_CMPLT		(2U)
#define SPI_EVENT_OVR_ERR		(3U)
#define SPI_EVENT_CRC_ERR		(4U)
/*********************************************************************************************************
 * 										APIs supported by this driver
 *						For more information about the APIs check the function definitions
 *********************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ (Interrupt Request) Configuration and ISR (Interrupt Service Routine) Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application CallBack
 */
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t ApplEvent);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
