/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Dev Tandon
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO (General Purpose Input Output) Pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/* Possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* Possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;	/* Possible values from @GPIO_PIN_PULLUP_PULLDOWN */
	uint8_t GPIO_PinOPType;			/* Possible values from @GPIO_PIN_OUTPUT_TYPE */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO (General Purpose Input Output) Pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO Pin Configuration Settings */
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible numbers
 */
#define GPIO_PIN_NO_0		(0U)
#define GPIO_PIN_NO_1		(1U)
#define GPIO_PIN_NO_2		(2U)
#define GPIO_PIN_NO_3		(3U)
#define GPIO_PIN_NO_4		(4U)
#define GPIO_PIN_NO_5		(5U)
#define GPIO_PIN_NO_6		(6U)
#define GPIO_PIN_NO_7		(7U)
#define GPIO_PIN_NO_8		(8U)
#define GPIO_PIN_NO_9		(9U)
#define GPIO_PIN_NO_10		(10U)
#define GPIO_PIN_NO_11		(11U)
#define GPIO_PIN_NO_12		(12U)
#define GPIO_PIN_NO_13		(13U)
#define GPIO_PIN_NO_14		(14U)
#define GPIO_PIN_NO_15		(15U)

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		(0U)	// GPIO Mode Input
#define GPIO_MODE_OUT		(1U)	// GPIO Mode Output
#define GPIO_MODE_ALTFN		(2U)	// GPIO Mode Alternate Functionality
#define GPIO_MODE_ANALOG	(3U)	// GPIO Mode Analog
#define GPIO_MODE_IT_FT		(4U)	// GPIO Mode Interrupt Falling Edge
#define GPIO_MODE_IT_RT		(5U)	// GPIO Mode Interrupt Rising Edge
#define GPIO_MODE_IT_RFT	(6U)	// GPIO Mode Interrupt Rising and Falling Edge

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		(0U)
#define GPIO_SPEED_MEDIUM	(1U)
#define GPIO_SPEED_FAST		(2U)
#define GPIO_SPEED_HIGH		(3U)

/*
 * @GPIO_PIN_PULLUP_PULLDOWN
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		(0U)	// GPIO No PullUpPullDown
#define GPIO_PIN_PU		    (1U)	// GPIO PullUp
#define GPIO_PIN_PD			(2U)	// GPIO PullDown

/*
 * @GPIO_PIN_OUTPUT_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OPT_TYPE_PP	(0U)	// GPIO Output Type PushPull
#define GPIO_OPT_TYPE_OD	(1U)	// GPIO Output Type OpenDrain
/*********************************************************************************************************
 * 										APIs supported by this driver
 *						For more information about the APIs check the function definitions
 *********************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ (Interrupt Request) Configuration and ISR (Interrupt Service Routine) Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
