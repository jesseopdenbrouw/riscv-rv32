#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <thuasrv32.h>

/* Frequency of the DE0-CV board */
#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
/* Transmission speed */
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif


int main(void) {

	long long int m = 0xf000010010000000;
	uint64_t um = 0xf000010010000000;

	uart1_init(UART_PRESCALER(BAUD_RATE), UART_CTRL_NONE);

	uart1_puts("\r\n");
	uart1_printlonglong(m);
	uart1_puts("\r\n");
	uart1_printulonglong(um);

	return 0;
}
