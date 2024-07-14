/*
 * main.c
 *
 *  Created on: May 19, 2024
 *      Author: HP
 */

#include <stdio.h>

long long int g_Data = 0xFFFEabcde1234598;

int main(void)
{
	char* pAddress1;
	pAddress1 = (char*)&g_Data;
	printf("Value at address %p is = %x\n",pAddress1,*pAddress1);

	int* pAddress2;
	pAddress2 = (int*)&g_Data;
	printf("Value at address %p is = %x\n",pAddress2,*pAddress2);

	short* pAddress3;
	pAddress3 = (short*)&g_Data;
	printf("Value at address %p is = %x\n",pAddress3,*pAddress3);

	long long* pAddress4;
	pAddress4 = (long long*)&g_Data;
	printf("Value at address %p is = %I64x\n",pAddress4,*pAddress4);
}
