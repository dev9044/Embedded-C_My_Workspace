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

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
 	uint32_t *pClkctrlreg = (uint32_t *)0x40023830;
	uint32_t *pPortDModeReg = (uint32_t *)0x40020C00;
	uint32_t *pPortDOutReg = (uint32_t *)0x40020C14;

	//1. Enable the clock for GPIOD peripheral in the AHB1ENR (SET the 3rd bit position)
	*pClkctrlreg |= (1 << 3);

	//2. Configure the mode of the IO Pin as output
	//a. clear the 24th and 25th bit positions (CLEAR)
	*pPortDModeReg &= ~(3 << 24);
	//b. make 24th bit position as 1 (SET)
	*pPortDModeReg |= (1 << 24);

	while(1)
	{
		//3. SET 12th bit of the output data register to make I/O pin-12 as HIGH
		*pPortDOutReg |= (1 << 12);
		// Introduce small human observable delay
		for(uint32_t i=0; i < 100000; i++);
		// Turn OFF the LED
		*pPortDOutReg &= ~(1 << 12);

		for(uint32_t i=0; i < 100000; i++);
	}
}
