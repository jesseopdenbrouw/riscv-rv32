/*
 *
 * spi1softnss
 *
 * Program to read data from the 25AA010A serial EEPROM
 *
 * The program uses a software NSS (Chip Select) signal,
 * connected to POUTA pin 15. First, an 8-bit EEPROMREAD
 * code is send. Data is received, but is discarded.
 * Then an 8-bit address is send, and the received data
 * is discarded. Last, an 8-bit dummy (0x00) is send.
 * During the sending of the dummy, the 25AA010A transmits
 * the contents of the address, and this value is printed
 * to the USART. This version uses software generated NSS.
 *
 */

#include <stdint.h>
#include <ctype.h>

#include "io.h"
#include "uart.h"
#include "util.h"

/* Should be loaded by the Makefile */
#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif

int main(void)
{

	/* Deactivate device, soft NSS high */
	GPIOA->POUT |= 1<<15;

	/* CS setup, CS hold, /16, 8 bits, mode 0 */
	SPI1->CTRL = (0 << 20) | (0 << 12) | (3<<8) | (0<<4) | (0<<1);

	uart1_init(F_CPU/BAUD_RATE-1, 0x00);

	uart1_puts("\r\n");

	while (1) {
		/* Read first 16 bytes */
		for (uint32_t addr = 0x00; addr < 0x10; addr++) {

			/* Activate device, soft NSS low */
			GPIOA->POUT &= ~(1<<15);

			/* Send EEPROMREAD */
			SPI1->DATA = 0x03; 

			/* Wait for transmission complete */
			while (!(SPI1->STAT & 0x08));

			/* Send address */
			SPI1->DATA = addr; 

			/* Wait for transmission complete */
			while (!(SPI1->STAT & 0x08));

			/* Send dummy */
			SPI1->DATA = 0x00; 

			/* Wait for transmission complete */
			while (!(SPI1->STAT & 0x08));

			/* Deactivate device, soft NSS high */
			GPIOA->POUT |= 1<<15;

			/* Read out received data */
			uint32_t read = SPI1->DATA;

			/* Print out address, data and ASCII char */
			uart1_puts("Address: 0x");
			printhex(addr & 0xff, 2);
			uart1_puts(" = 0x");
			printhex(read, 2);
			uart1_puts(" ASCII: ");
			if (read >= 32 && read < 127) {
				uart1_putc((int) (read & 0xff));
			} else {
				uart1_putc('.');
			}
			uart1_puts("\r\n");
	
			/* Simple delay */
			for (volatile uint32_t i = 0; i < 5000000; i++);
		}
		uart1_puts("-----------------------------\r\n");
	}

	return 0;
}
