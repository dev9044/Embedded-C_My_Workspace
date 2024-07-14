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
	uint32_t volatile *const pClkctrlreg = (uint32_t *)0x40023830;
	uint32_t volatile *const pPortDModeReg = (uint32_t *)0x40020C00;
	uint32_t volatile *const pPortDOutReg = (uint32_t *)0x40020C14;

	uint32_t volatile *const pPortAModeReg = (uint32_t *)0x40020000;
	uint32_t const volatile *const pPortAInReg = (uint32_t *)0x40020010;

	//Enable the clock for GPIOD peripheral in the AHB1ENR (SET the 3rd bit position)
	*pClkctrlreg |= (1 << 3);
	//Enable the clock for GPIOA peripheral in the AHB1ENR (SET the 0th bit position)
	*pClkctrlreg |= (1 << 0);

	//Configure the mode of the IO Pin PD15 as output (GPIOD Mode Register)
	//a. clear the 30th and 31st bit positions (CLEAR)
	*pPortDModeReg &= ~(12 << 30);
	//b. make 30th bit position as 1 (SET)
	*pPortDModeReg |= (1 << 30);

	//Configure the mode of the IO Pin PA0 as input (GPIOA Mode Register)
	//clear the 0th and 1st bit positions (CLEAR)
	*pPortAModeReg &= ~(3 << 0);

	//Read the pin status of the pin PA0 (GPIOA Input Data Register)
	uint8_t pinStatus = (uint8_t)(*pPortAInReg & 0x1); //zero out all other bits except bit 0

	while(1)
	{
		if(pinStatus)
		{
			//SET 15th bit of the output data register to make I/O pin-15 as HIGH
			//turn on the LED
			*pPortDOutReg |= (1 << 15);
		}
		else
		{
			//SET 15th bit of the output data register to make I/O pin-15 as LOW
			//turn off the LED
			*pPortDOutReg &= ~(1 << 15);
		}
	}
}
