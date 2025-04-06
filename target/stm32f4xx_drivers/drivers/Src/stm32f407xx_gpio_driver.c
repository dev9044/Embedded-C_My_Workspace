/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Dev Tandon
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */
/********************************************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	// Clock Enable
	if (ENABLE == EnorDi)
	{
		if (GPIOA == pGPIOx)
		{
		  GPIOA_PCLK_EN();
		}
		else if (GPIOB == pGPIOx)
		{
		  GPIOB_PCLK_EN();
		}
		else if (GPIOC == pGPIOx)
		{
		  GPIOC_PCLK_EN();
		}
		else if (GPIOD == pGPIOx)
		{
		  GPIOD_PCLK_EN();
		}
		else if (GPIOE == pGPIOx)
		{
		  GPIOE_PCLK_EN();
		}
		else if (GPIOF == pGPIOx)
		{
		  GPIOF_PCLK_EN();
		}
		else if (GPIOG == pGPIOx)
		{
		  GPIOG_PCLK_EN();
		}
		else if (GPIOH == pGPIOx)
		{
		  GPIOH_PCLK_EN();
		}
		else if (GPIOI == pGPIOx)
		{
		  GPIOI_PCLK_EN();
		}
	}
	// Clock Disable
	else
	{
		if (GPIOA == pGPIOx)
		{
		  GPIOA_PCLK_DI();
		}
		else if (GPIOB == pGPIOx)
		{
		  GPIOB_PCLK_DI();
		}
		else if (GPIOC == pGPIOx)
		{
		  GPIOC_PCLK_DI();
		}
		else if (GPIOD == pGPIOx)
		{
		  GPIOD_PCLK_DI();
		}
		else if (GPIOE == pGPIOx)
		{
		  GPIOE_PCLK_DI();
		}
		else if (GPIOF == pGPIOx)
		{
		  GPIOF_PCLK_DI();
		}
		else if (GPIOG == pGPIOx)
		{
		  GPIOG_PCLK_DI();
		}
		else if (GPIOH == pGPIOx)
		{
		  GPIOH_PCLK_DI();
		}
		else if (GPIOI == pGPIOx)
		{
		  GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/********************************************************************************************
 * @fn			- GPIO_Init
 *
 * brief		- This function perform GPIO Initialization
 *
 * @param[in]	- base address of the gpio peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0U;	// temp. register

	// Configure the clock for GPIO Peripheral
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// Configure the mode of gpio pin
	if (GPIO_MODE_ANALOG >= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
	{
		// The non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2U * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// Multiply with 2 because one pin takes 2 bits
		pGPIOHandle->pGPIOx->MODER &= ~(0x3U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // setting
	}
	else
	{
		// The interrupt mode
		if (GPIO_MODE_IT_FT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			// Configure the FTSR (Falling Trigger Selection Register) bit
			EXTI->FTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR (Rising Trigger Selection Register) bit
			EXTI->RTSR &= ~(1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (GPIO_MODE_IT_RT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			// Configure the RTSR (Rising Trigger Selection Register) bit
			EXTI->RTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR (Falling Trigger Selection Register) bit
			EXTI->FTSR &= ~(1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (GPIO_MODE_IT_RFT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			// Configure the FTSR (Falling Trigger Selection Register) bit
			EXTI->FTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Configure the RTSR (Rising Trigger Selection Register) bit
			EXTI->RTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR (System Configuration Controller External Interrupt)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4U;	// It decides that EXTICR[x] = EXTICR[temp1]
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4U;	// It decides that Section for EXTI
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4U);	// Multiply with 4 because one pin takes 4 bit

		// Enable the EXTI Interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0U;

	// Configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2U * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// Multiply with 2 because one pin takes 2 bits
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting

	temp = 0U;

	// Configure the PullUp/PullDown settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2U * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// Multiply with 2 because one pin takes 2 bits
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; // setting

	temp = 0U;

	// Configure the OutputType
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; // setting

	temp = 0U;

	// Configure the Alternate Functionality
	if (GPIO_MODE_ALTFN == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
	{
		// Configure the alternate function registers
		uint8_t temp1 = 0U;
		uint8_t temp2 = 0U;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8U;	// It decides that which AFR will use in AFRL (Alternate Function High Register) or AFRH (Alternate Function Low Register)
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8U;	// It decides that which Pin will use
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xFU << (4U * temp2)); // clearing and Multiply with 4 because one pin takes 4 bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4U * temp2)); // setting	and Multiply with 4 because one pin takes 4 bits
	}
}

/********************************************************************************************
 * @fn			- GPIO_DeInit
 *
 * brief		- This function perform GPIO DeInitialization
 *
 * @param[in]	- base address of the gpio peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	// Reset all registers of GPIOx Port
	if (GPIOA == pGPIOx)
	{
	  GPIOA_REG_RESET();
	}
	else if (GPIOB == pGPIOx)
	{
	  GPIOB_REG_RESET();
	}
	else if (GPIOC == pGPIOx)
	{
	  GPIOC_REG_RESET();
	}
	else if (GPIOD == pGPIOx)
	{
	  GPIOD_REG_RESET();
	}
	else if (GPIOE == pGPIOx)
	{
	  GPIOE_REG_RESET();
	}
	else if (GPIOF == pGPIOx)
	{
	  GPIOF_REG_RESET();
	}
	else if (GPIOG == pGPIOx)
	{
	  GPIOG_REG_RESET();
	}
	else if (GPIOH == pGPIOx)
	{
	  GPIOH_REG_RESET();
	}
	else if (GPIOI == pGPIOx)
	{
	  GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
/********************************************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * brief		- This function reads the input from gpio pin
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- Pin Number
 *
 * @return		- 0 or 1
 *
 * @Note		- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value = 0U;

	Value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001U);	// Read only from LSB (Least Significant Bit)

	return Value;
}

/********************************************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * brief		- This function reads the input from gpio port
 *
 * @param[in]	- base address of the gpio peripheral
 *
 * @return		- uint16_t
 *
 * @Note		- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value = 0U;

	Value = (uint8_t)pGPIOx->IDR;

	return Value;
}

/********************************************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * brief		- This function writes the value on given gpio pin number
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- Pin Number
 * @param[in]	- Value
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (GPIO_PIN_SET == Value)
	{
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1U << PinNumber);
	}
	else
	{
		// write 0 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1U << PinNumber);
	}
}

/********************************************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * brief		- This function writes the value on given gpio port
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- Value
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}

/********************************************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * brief		- This function toggles the output pin
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- PinNumber
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1U << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
/********************************************************************************************
 * @fn			- GPIO_IRQInterruptConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		if (31U >= IRQNumber) // 0 to 31
		{
			// Program ISER0 (Interrupt Set-Enable Register 0)Register
			*NVIC_ISER0 |= (1U << IRQNumber);
		}
		else if ((31U < IRQNumber) && (64U > IRQNumber)) // 32 to 63
		{
			// Program ISER1 (Interrupt Set-Enable Register 1)Register
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
 * @fn			- GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find out the IPR (Interrupt Priority Register) Register
	uint8_t iprx = IRQNumber / 4U;			// It decides that IPR (Interrupt Priority Register)
	uint8_t iprx_section = IRQNumber % 4U;	// It decides that IPR (Interrupt Priority Register) Section

	uint8_t shift_amount = (8U * iprx_section) + (8U - NO_OF_PR_BITS_IMPLEMENTED); // Lower 4 bits are not applicable for IPR (Interrupt Priority Register)
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/********************************************************************************************
 * @fn			- GPIO_IRQHandling
 *
 * brief		- This function handles IRQ
 *
 * @param[in]	- PinNumber
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI PR (External Interrupt Priority Register) Register Corresponding to the pin number
	if ((EXTI->PR) & (1U << PinNumber))
	{
		// Clear
		EXTI->PR |= (1U << PinNumber);
	}
}
