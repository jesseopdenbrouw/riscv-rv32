#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* THUASRV32 */
#include "io.h"
#include "uart.h"


/* __io_putchar prints a character via the UART1 */
int __io_putchar(int ch)
{
	uart1_putc(ch);
	return 1;
}
