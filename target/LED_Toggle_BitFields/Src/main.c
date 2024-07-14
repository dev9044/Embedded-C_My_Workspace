/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "main.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
	RCC_AHB1ENR_t volatile *const pClkctrlreg = (RCC_AHB1ENR_t *)0x40023830;
	GPIOx_Mode_REGS_t volatile *const pPortDModeReg = (GPIOx_Mode_REGS_t *)0x40020C00;
	GPIOx_ODR_REGS_t volatile *const pPortDOutReg = (GPIOx_ODR_REGS_t *)0x40020C14;

	//1. Enable the clock for GPIOD peripheral in the AHB1ENR
    pClkctrlreg->gpiod_en = 1;

    //2. Configure the mode of the IO Pin as output
    pPortDModeReg->pin_12 = 1;

    while(1)
	{
		//3. SET 12th bit of the output data register to make I/O pin-12 as HIGH
    	pPortDOutReg->pin_12 = 1;
		// Introduce small human observable delay
		for(uint32_t i=0; i < 100000; i++);
		// Turn OFF the LED
		pPortDOutReg->pin_12 = 0;

		for(uint32_t i=0; i < 100000; i++);
	}
}
