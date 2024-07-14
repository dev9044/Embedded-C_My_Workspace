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

	uint32_t *pPortAModeReg = (uint32_t *)0x40020000;
	uint32_t *pPortAOutReg = (uint32_t *)0x40020014;

	//Enable the clock for GPIOD peripheral in the AHB1ENR (SET the 3rd bit position)
	*pClkctrlreg |= (1 << 3);
	//Enable the clock for GPIOA peripheral in the AHB1ENR (SET the 0th bit position)
	*pClkctrlreg |= (1 << 0);

	//Configure the mode of the IO Pin PD13 as output (GPIOD Mode Register)
	//a. clear the 26th and 27th bit positions (CLEAR)
	*pPortDModeReg &= ~(12 << 26);
	//b. make 26th bit position as 1 (SET)
	*pPortDModeReg |= (1 << 26);

	//Configure the mode of the IO Pin PA0 as output (GPIOA Mode Register)
	//a. clear the 0th and 1st bit positions (CLEAR)
	*pPortAModeReg &= ~(3 << 0);
	//b. make 0th bit position as 1 (SET)
	*pPortAModeReg |= (1 << 0);

	//SET 0th bit of the output data register to make I/O pin-0 as HIGH
	*pPortAOutReg |= (1 << 0);

	//SET 0th bit of the output data register to make I/O pin-0 as LOW
    //*pPortAOutReg &= ~(1 << 0);

	uint8_t pinStatus = (uint8_t)(*pPortAOutReg);

	while(1)
	{
		if(pinStatus)
		{
			//SET 13th bit of the output data register to make I/O pin-13 as HIGH
			//turn on the LED
			*pPortDOutReg |= (1 << 13);
		}
		else
		{
			//SET 13th bit of the output data register to make I/O pin-13 as LOW
			//turn off the LED
			*pPortDOutReg &= ~(1 << 13);
		}
	}
}
