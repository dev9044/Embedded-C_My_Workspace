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
	//b. clear the 26th and 27th bit positions (CLEAR)
	*pPortDModeReg &= ~(12 << 26);
	//c. clear the 28th and 29th bit positions (CLEAR)
	*pPortDModeReg &= ~(3 << 28);
	//d. clear the 30th and 31th bit positions (CLEAR)
	*pPortDModeReg &= ~(12 << 30);
	//e. make 24th bit position as 1 (SET)
	*pPortDModeReg |= (1 << 24);
	//f. make 26th bit position as 1 (SET)
	*pPortDModeReg |= (1 << 26);
	//g. make 28th bit position as 1 (SET)
	*pPortDModeReg |= (1 << 28);
	//h. make 30th bit position as 1 (SET)
	*pPortDModeReg |= (1 << 30);

	while(1)
	{
		//3. SET 12th bit of the output data register to make I/O pin-12 as HIGH
		*pPortDOutReg |= (1 << 12);
		// Introduce small human observable delay
		for(uint32_t i=0; i < 100000; i++);
		// Turn OFF the LED
		*pPortDOutReg &= ~(1 << 12);
		for(uint32_t i=0; i < 100000; i++);
		//4. SET 13th bit of the output data register to make I/O pin-13 as HIGH
		*pPortDOutReg |= (1 << 13);
		// Introduce small human observable delay
		for(uint32_t i=0; i < 100000; i++);
		*pPortDOutReg &= ~(1 << 13);
		for(uint32_t i=0; i < 100000; i++);
		//5. SET 14th bit of the output data register to make I/O pin-14 as HIGH
		*pPortDOutReg |= (1 << 14);
		// Introduce small human observable delay
		for(uint32_t i=0; i < 100000; i++);
		*pPortDOutReg &= ~(1 << 14);
		for(uint32_t i=0; i < 100000; i++);
		//6. SET 15th bit of the output data register to make I/O pin-15 as HIGH
		*pPortDOutReg |= (1 << 15);
		// Introduce small human observable delay
		for(uint32_t i=0; i < 100000; i++);
		*pPortDOutReg &= ~(1 << 15);
		for(uint32_t i=0; i < 100000; i++);
	}
}
