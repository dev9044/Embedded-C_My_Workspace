/*
 * main.c
 *
 *  Created on: May 19, 2024
 *      Author: HP
 */

#include <stdio.h>

int main()
{
	double number1, number2, number3;
	double Average;

	printf("Enter 3 numbers:");
	fflush(stdout);
	scanf("%lf %lf %lf",&number1,&number2,&number3);

	Average = (number1+number2+number3)/3;
	printf("\nAverage = %lf",Average);
}
