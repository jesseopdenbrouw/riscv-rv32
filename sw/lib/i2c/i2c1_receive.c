#include <thuasrv32.h>

uint32_t i2c1_receive(uint8_t address, uint8_t *buf, uint32_t len)
{
	/* If length is 0, only transmit address + START + STOP */
	if (len == 0) {
		return i2c1_transmit_address_only(address);
	}

	/* Transmit address + START */
	uint32_t ret = i2c1_transmit_address(address);

	/* If error, return */
	if (ret) {
		return ret;
	}

	/* Receive all bytes, first set master ACK */
	I2C1->CTRL |= I2C_MACK;
	for (int i = 0; i < len; i++) {
		/* Set STOP generation on last byte */
		if (i == len-1) {
			I2C1->CTRL |= I2C_STOP;
		}
		/* Transfer byte, return if error */
		*buf++ = i2c1_receive_byte();
	}
	/* Disable master ACK */
	I2C1->CTRL &= ~I2C_MACK;

	return 0;	
}
