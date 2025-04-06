/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Apr 5, 2025
 *      Author: Dev Tandon
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

//This returns the APB1 (Advanced Peripherals Bus 1) clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 (Advanced Peripherals Bus 2) clock value
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
