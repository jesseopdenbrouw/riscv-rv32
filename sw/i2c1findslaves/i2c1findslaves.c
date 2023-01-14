/*
 *
 * i2c1findslaves -- find slaves on the I2C bus
 *
 * This program searches for connected I2C slaves on the bus
 *
 *
 */

#include <stdio.h>
#include <stdint.h>

#include "io.h"
#include "uart.h"

#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif

/* Standard mode (Sm), 100 kHz */
#define TRAN_SPEED ((F_CPU/2UL/100000UL)-1)

int main(void)
{
	char buffer[20];

	uart1_init(F_CPU/BAUD_RATE-1, 0x00);

	uart1_puts("\r\nI2C1 find slaves\r\nSpeed set to: ");
	snprintf(buffer, sizeof buffer, "%lu\r\n", TRAN_SPEED);
	uart1_puts(buffer);

	I2C1->CTRL = TRAN_SPEED << 16;

	for (uint32_t i = 0x01; i < 0x78; i++) {
		/* Set START and STOP generation */
		I2C1->CTRL |= (1 << 9) | (1 << 8);

		/* Write address + write bit */
		I2C1->DATA = (i << 1) | 0;

		/* Wait for data transmission completed */
		while ((I2C1->STAT & 0x08) == 0x00);

		/* Check ACKnowledge */
		if (I2C1->STAT & (1 << 5)) {
		//	uart1_puts("ACK failed!\r\n");
		} else {
			/* Print out the data */
			uart1_puts("Slave found at address: ");
			snprintf(buffer, sizeof buffer, "0x%02lx\r\n", i);
			uart1_puts(buffer);
		}

		for (volatile uint32_t i = 0; i < 500; i++);
	}

	uart1_puts("Done\r\n");
	while (1) {}
}

