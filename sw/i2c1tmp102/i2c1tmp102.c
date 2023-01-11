/*
 *
 * i2c1tmp102 -- read temperature with I2C1 and TMP102
 *
 * This program reads out the TMP102 digital temperature
 * sensor using the I2C1 peripheral. The target has I2C
 * address 0x48, but this may be changed (on the target)
 * using the ADD0 pin.
 *
 * First, register number 0x00 is send to the TMP102. Then
 * two bytes are read from the TMP102. The TMP102 sends a
 * 12-bit temperature info, left adjusted to 16 bits. Format
 * is 0xhhl0 (hh is high byte, l is low nibble, 0 is 0). The
 * 12-bit temperature is in 1/16th of a degree Celsius. So
 * 0x13b0 indicates 19.6875 degree Celsius. Note that the
 * temperature is in two's complement, so negative values may
 * be read out.
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

#define FAST_MODE (1)
#define TMP102_ADDR (0x48)


#if FAST_MODE == 1
/* Fast mode (Fm), 400 kHz */
#define TRAN_SPEED (((F_CPU/3UL/400000UL)-1)+1)
#define FAST_MODE_BIT (1 << 2)
#else
/* Standard mode (Sm), 100 kHz */
#define TRAN_SPEED (((F_CPU/2UL/400000UL)-1)+1)
#define FAST_MODE_BIT (0 << 2)
#endif


int main(void)
{
	uint32_t temphi = 0;
	uint32_t templo = 0;
	char buffer[40];

	uart1_init(F_CPU/BAUD_RATE-1, 0x00);

	uart1_puts("I2C1 with TMP102\r\nSpeed set to: ");
	snprintf(buffer, sizeof buffer, "%lu\r\n", TRAN_SPEED);
	uart1_puts(buffer);

	I2C1->CTRL = (TRAN_SPEED << 16) | FAST_MODE_BIT;

	while(1) {

		/* Set START generation */
		I2C1->CTRL |= (1 << 9);

		/* Write address + write bit */
		I2C1->DATA = (TMP102_ADDR << 1) | 0;

		/* Wait for data transmission completed */
		while ((I2C1->STAT & 0x08) == 0x00);

		/* Check ACKnowledge */
		if (I2C1->STAT & (1 << 5)) {
			uart1_puts("ACK failed!\r\n");
		} else {

			/* Write register number and STOP generation */
			I2C1->CTRL |= (1 << 8);
			I2C1->DATA = 0x00;
			while ((I2C1->STAT & 0x08) == 0x00);


			/* Read in response */

			/* Set START generation */
			I2C1->CTRL |= (1 << 9);

			/* Write address + read bit */
			I2C1->DATA = (TMP102_ADDR << 1) | 1;

			/* Wait for data transmission completed */
			while ((I2C1->STAT & 0x08) == 0x00);

			/* I2C1 device now switched to reception */

			/* Wait for data transmission completed */
			while ((I2C1->STAT & 0x10) == 0x00);

			temphi = I2C1->DATA;

			/* Set STOP generation */
			I2C1->CTRL |= (1 << 8);
			/* Wait for data transmission completed */
			while ((I2C1->STAT & 0x10) == 0x00);

			templo = I2C1->DATA;

			/* Print out the data */
			snprintf(buffer, sizeof buffer, "HI: 0x%02lx, LO: 0x%02lx\r\n", temphi, templo);
			uart1_puts(buffer);
		}

		for (volatile uint32_t i = 0; i < 5000000; i++);
	}
}

