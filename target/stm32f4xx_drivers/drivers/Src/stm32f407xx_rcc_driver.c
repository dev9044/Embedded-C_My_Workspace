/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Apr 5, 2025
 *      Author: HP
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_Prescaler[8U] = {2U,4U,8U,16U,64U,128U,256U,512U};
uint16_t APB1_Prescaler[4U] = {2U,4U,8U,16U};

/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             - This function gets the pclk1 value for FREQ field of CR2
 *
 * @param[in]         - none
 *
 * @return            - pclk1
 *
 * @Note              - none
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1 = 0U;
	uint32_t SystemClk = 0U;
	uint8_t clksrc = 0U;
	uint8_t temp = 0U;
	uint8_t ahbp = 0U;
	uint8_t apb1p = 0U;

	clksrc = (RCC->CFGR >> 2U) & 0x3U;

	if (0U == clksrc)
	{
		SystemClk = 16000000;
	}
	else if (1U == clksrc)
	{
		SystemClk = 8000000;
	}

	// For AHB
	temp = (RCC->CFGR >> 4U) & 0xF;

	if (8U > temp)
	{
		ahbp = 1U;
	}
	else
	{
		ahbp = AHB_Prescaler[temp-8U];
	}

	// For APB1
	temp = (RCC->CFGR >> 10U) & 0x7;

	if (4U > temp)
	{
		apb1p = 1U;
	}
	else
	{
		apb1p = APB1_Prescaler[temp-4U];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - This function gets the pclk2 value for FREQ field of CR2
 *
 * @param[in]         - none
 *
 * @return            - pclk2
 *
 * @Note              - none
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock = 0U;
	uint8_t tmp = 0U;
	uint32_t pclk2 = 0U;
	uint8_t clk_src = 0U;
	uint8_t ahbp = 0U;
	uint8_t apb2p = 0U;

	clk_src = (RCC->CFGR >> 2) & 0X3;

	if (0U == clk_src)
	{
		SystemClock = 16000000;
	}
	else
	{
		SystemClock = 8000000;
	}

	tmp = (RCC->CFGR >> 4) & 0xF;

	if (0x08 > tmp)
	{
		ahbp = 1;
	}
	else
	{
	   ahbp = AHB_Prescaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13) & 0x7;

	if (0x04 > tmp)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB1_Prescaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp) / apb2p;

	return pclk2;
}

uint32_t  RCC_GetPLLOutputClock()
{
	return 0;
}
