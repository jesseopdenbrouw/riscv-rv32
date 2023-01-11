#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* THUASRV32 */
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

/* Initialize the Baud Rate Generator */
void uart1_init(uint32_t prescaler, uint32_t ctrl)
{
	/* Set baud rate generator */
	UART1->BAUD = prescaler;
	UART1->CTRL = ctrl;
}
