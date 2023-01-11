/*
 *
 * spi1readeeprom
 *
 * Program to read data from the 25AA010A serial EEPROM
 *
 * This program reads the first 16 bytes from the EEPROM and
 * displays them on the terminal program (e.g. PuTTY) using
 * USART2. The program writes a 24-bit instruction code and
 * address and a dummy byte. During this 24-bit write the
 * data from the EEPROM is read. In the least significant
 * byte is the data from the EEPROM. The EEPROM can work in
 * mode 0 and mode 3.
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

	/* CS setup, CS hold, /16, 24 bits, mode 0 */
	SPI1->CTRL = (9 << 20) | (9 << 12) | (3<<8) | (2<<4) | (0<<1);

	uart1_init(F_CPU/BAUD_RATE-1, 0x00);

	uart1_puts("\r\n");

	while (1) {
		/* Read first 16 bytes */
		for (uint32_t addr = 0x00; addr < 0x10; addr++) {

			/* EEPROMREAD + addr + dummy */
			/* During dummy, data is read from addr */
			SPI1->DATA = (0x03 << 16) | (addr << 8); 

			/* Wait for transmission complete */
			while (!(SPI1->STAT & 0x08));

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
