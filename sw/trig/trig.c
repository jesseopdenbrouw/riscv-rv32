/*
 * Program to test trigoniometry functions
 * on the processor. Due to large ROM contents
 * we need to select the functions used.
 *
 * Since float/doubles are printed, we need to
 * include the linker flag -u _printf_float
 *
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

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

int main(void)
{
	char buffer[60];

	volatile float a, b, c;
	volatile float w = 0.57f;

	volatile double y = 0.57;
	volatile double k, l, m;


	uart1_init(UART_PRESCALER(BAUD_RATE), UART_CTRL_NONE);

	uart1_puts("float and double calculations\r\n");

	/* Record start time */
	clock_t start = clock();

	/* Do the calculations */
	a = sinf(w);
	b = asinf(w);
	c = logf(w);

	k = sin(y);
	l = asin(y);
	m = tan(w);

	/* Record difference */
	start = clock() - start;

	/* Print out the results */
	sprintf(buffer, "sinf(%f) = %.10f\r\n", w, a);
	uart1_puts(buffer);

	sprintf(buffer, "asinf(%f) = %.10f\r\n", w, b);
	uart1_puts(buffer);

	sprintf(buffer, "logf(%f) = %.10f\r\n", w, c);
	uart1_puts(buffer);


	sprintf(buffer, "sin(%f) = %.20f\r\n", w, k);
	uart1_puts(buffer);

	sprintf(buffer, "asin(%f) = %.20f\r\n", w, l);
	uart1_puts(buffer);

	sprintf(buffer, "tan(%f) = %.20f\r\n", w, m);
	uart1_puts(buffer);


	sprintf(buffer, "Time: %lu\r\n", start);
	uart1_puts(buffer);


	return 0;
}
