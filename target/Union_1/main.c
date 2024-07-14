/*
 * main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: HP
 */

#include <stdio.h>
#include <stdint.h>

union Address
{
	uint16_t shortAddr;
	uint32_t longAddr;
};

int main (void)
{
	union Address addr;

	addr.shortAddr = 0xABCD;
	addr.longAddr = 0xEEEECCCC;

	printf("shortAddr = %#x\n",addr.shortAddr);
	printf("longAddr = %#x\n",addr.longAddr);

	while(getchar() != '\n');
	getchar();
}
