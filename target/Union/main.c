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
	uint32_t packetValue = 0;
	printf("Enter the 32bit Packet Value :");
	scanf("%X",&packetValue);

	union Address addr;

	addr.shortAddr = packetValue;
	addr.longAddr = packetValue;

	printf("shortAddr = %#x\n",addr.shortAddr);
	printf("longAddr = %#x\n",addr.longAddr);

	while(getchar() != '\n');
	getchar();
}
