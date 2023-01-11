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

/* Check if a character is available */
int uart1_available(void)
{
	return (UART1->STAT & 0x04);
}
