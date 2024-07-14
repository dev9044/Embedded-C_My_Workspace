/*
 * main.c
 *
 *  Created on: May 19, 2024
 *      Author: HP
 */

#include <stdio.h>

int main()
{
	float number1, number2, number3;
	float Average;

	printf("Enter the first number:");
	fflush(stdout);
	scanf("%f",&number1);
	printf("\nEnter the second number:");
	fflush(stdout);
    scanf("%f",&number2);
	printf("\nEnter the third number:");
	fflush(stdout);
	scanf("%f",&number3);

	Average = (number1+number2+number3)/3;
	printf("\nAverage = %f",Average);
}
