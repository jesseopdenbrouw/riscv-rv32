#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* THUASRV32 */
#include <thuasrv32.h>

/* Frequency of the DE0-CV board */
#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
/* Transmission speed */
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif

/* Initialize the Baud Rate Generator and Control Register */
void uart1_init(uint32_t baudrate, uint32_t ctrl)
{
	/* Set baud rate generator */
	uint32_t speed = csr_read(0xfc1);
	speed = (speed == 0) ? F_CPU : speed;
	UART1->BAUD = speed/baudrate-1;
	/* Set control register */
	UART1->CTRL = ctrl;
}
