#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "io.h"

/* Frequency of the DE0-CV board */
#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
/* Transmission speed */
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif

/* Initialize the Baud Rate Generator */
void uart1_init(uint32_t baud, uint32_t ctrl)
{
	/* Set baud rate generator */
	UART1->BAUD = baud;
	UART1->CTRL = ctrl;
}

/* Send one character over the UART1 */
void uart1_putc(int ch)
{
	/* Transmit data */
	UART1->DATA = (uint8_t) ch;

	/* Wait for transmission end */
	while ((UART1->STAT & 0x10) == 0);
}

/* Send a null-terminated string over the UART1 */
void uart1_puts(char *s)
{
	if (s == NULL)
	{
		return;
	}

	while (*s != '\0')
	{
		uart1_putc(*s++);
	}
}

/* Get one character from the UART1 in
 * blocking mode */
int uart1_getc(void)
{
	/* Wait for received character */
	while ((UART1->STAT & 0x04) == 0);

	/* Return 8-bit data */
	return UART1->DATA & 0x000000ff;
}

/* Check if a character is received */
int uart1_isreceived(void)
{
	return (UART1->STAT & 0x04);
}

/* Gets a string terminated by a newline character from usart
 * The newline character is not part of the returned string.
 * The string is null-terminated.
 * A maximum of size-1 characters are read.
 * Some simple line handling is implemented */
int uart1_gets(char buffer[], int size)
{
	int index = 0;
	char chr;

	while (1) {
		chr = uart1_getc();
		switch (chr) {
			case '\n':
			case '\r':	buffer[index] = '\0';
					uart1_puts("\r\n");
					return index;
					break;
			/* Backspace key */
			case 0x7f:
			case '\b':	if (index>0) {
						uart1_putc(0x7f);
						index--;
					} else {
						uart1_putc('\a');
					}
					break;
			/* control-U */
			case 21:	while (index>0) {
						uart1_putc(0x7f);
						index--;
					}
					break;
			/* control-C */
			case 0x03:  	uart1_puts("<break>\r\n");
					index=0;
					break;
			default:	if (index<size-1) {
						if (chr>0x1f && chr<0x7f) {
							buffer[index] = chr;
							index++;
							uart1_putc(chr);
						}
					} else {
						uart1_putc('\a');
					}
					break;
		}
	}
	return index;
}

/* __io_putchar prints a character via the UART1 */
int __io_putchar(int ch)
{
	uart1_putc(ch);
	return 1;
}

/* __io_getchar gets a character from the UART1 */
int __io_getchar(void)
{
	return uart1_getc();
}
