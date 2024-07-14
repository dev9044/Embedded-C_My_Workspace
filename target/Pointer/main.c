/*
 * main.c
 *
 *  Created on: May 19, 2024
 *      Author: HP
 */

#include <stdio.h>

int main(void)
{
	char data = 100;
	printf("Value of data is = %d\n",data);
	printf("Address of variable data is = %p\n",&data);

	char* pAddress = &data;
	char value = *pAddress;
	printf("read value is = %d\n",value);

	*pAddress = 65;
	printf("Value of data is = %d\n",data);
}
