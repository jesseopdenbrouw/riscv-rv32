#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "io.h"
#include "uart.h"

/* Frequency of the DE0-CV board */
#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
/* Transmission speed */
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif


int main(void) {

	int j = 2;
	float k = 1.2f;
	double l = 0.6;
	long long int m = 0x7fffffffffffffff;

	char buffer[100] = { 0 };

	char *pc = buffer;

	uart1_init(F_CPU/BAUD_RATE-1,0x00);

	/* long long cannot be printed with the nano library */
	printf("%d %p %.20f %.20f %lld\r\n", j, pc, k, l, m);

	return 0;
}