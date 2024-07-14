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
	printf("Address = %p\n",pAddress1);
	printf("Value = %x\n",*pAddress1);

	pAddress1 = pAddress1 + 1;

	printf("Address = %p\n",pAddress1);
	printf("Value = %x\n",*pAddress1);

	int* pAddress2;
	pAddress2 = (int*)&g_Data;
	printf("Address = %p\n",pAddress2);
	printf("Value = %x\n",*pAddress2);

	pAddress2 = pAddress2 + 1;

	printf("Address = %p\n",pAddress2);
	printf("Value = %x\n",*pAddress2);

	short* pAddress3;
	pAddress3 = (short*)&g_Data;
	printf("Address = %p\n",pAddress3);
	printf("Value = %x\n",*pAddress3);

	pAddress3 = pAddress3 + 1;

	printf("Address = %p\n",pAddress3);
    printf("Value = %x\n",*pAddress3);

	long long* pAddress4;
	pAddress4 = (long long*)&g_Data;
	printf("Address = %p\n",pAddress4);
	printf("Value = %I64x\n",*pAddress4);

	pAddress4 = pAddress4 + 1;

	printf("Address = %p\n",pAddress4);
	printf("Value = %I64x\n",*pAddress4);
}
