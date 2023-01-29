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

#include <thuasrv32.h>

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

	uart1_init(UART_PRESCALER(BAUD_RATE), UART_CTRL_NONE);

	uart1_puts("\r\nI2C1 find slaves\r\nSpeed set to: ");
	snprintf(buffer, sizeof buffer, "%lu\r\n", TRAN_SPEED);
	uart1_puts(buffer);

	i2c1_init(TRAN_SPEED << 16);

	for (uint32_t i = 0x01; i < 0x78; i++) {

		if (i2c1_transmit(i << 1, NULL, 0) != 0) {
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

