/*
 * main.c
 *
 *  Created on: Jul 14, 2024
 *      Author: HP
 */

#include <stdint.h>
#include <stdio.h>

/*
 * This Union we used to store various fields of the packet in to variables
 * The variables of this union consumes 4 bytes in the memory
 */
union Packet
{
	uint32_t packetValue;

	struct
	{
		uint32_t crc 		:2;
		uint32_t status 	:1;
		uint32_t payload 	:12;
		uint32_t bat 		:3;
		uint32_t sensor 	:3;
		uint32_t longAddr 	:8;
		uint32_t shortAddr 	:2;
		uint32_t AddrMode 	:1;
	}packetFields;
};

int main(void)
{
	union Packet packet;
	printf("Enter the 32bit Packet Value :");
	scanf("%X",&packet.packetValue);

	printf("crc 		:%#x\n",packet.packetFields.crc);
	printf("status 		:%#x\n",packet.packetFields.status);
	printf("payload 	:%#x\n",packet.packetFields.payload);
	printf("bat 		:%#x\n",packet.packetFields.bat);
	printf("sensor 		:%#x\n",packet.packetFields.sensor);
	printf("longAddr 	:%#x\n",packet.packetFields.longAddr);
	printf("shortAddr 	:%#x\n",packet.packetFields.shortAddr);
	printf("AddrMode 	:%#x\n",packet.packetFields.AddrMode);

	printf("Size of union is %d\n",sizeof(packet));

	while (getchar() != '\n');
	getchar();
}
