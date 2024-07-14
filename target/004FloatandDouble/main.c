/*
 * main.c
 *
 *  Created on: May 5, 2024
 *      Author: HP
 */

#include <stdio.h>

int main(void)
{
	float number = 1.34567289547e-19;
	printf("%0.9f\n",number);
	printf("%0.9e\n",number);

	double ChargeE = 1.34567289547e-19;
    printf("%0.11lf\n",ChargeE);
    printf("%0.11le\n",ChargeE);
}
