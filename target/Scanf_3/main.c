/*
 * main.c
 *
 *  Created on: May 19, 2024
 *      Author: HP
 */

#include <stdio.h>

int main()
{
	char c1,c2,c3,c4,c5,c6;

	printf("Enter 6 characters:");
	fflush(stdout);
	c1 = getchar();
	c2 = getchar();
	c3 = getchar();
	c4 = getchar();
	c5 = getchar();
	c6 = getchar();

	printf("\nASCII Codes = %u %u %u %u %u %u",c1,c2,c3,c4,c5,c6);
}
