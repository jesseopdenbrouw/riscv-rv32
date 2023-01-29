#include <thuasrv32.h>

uint32_t i2c1_transmit(uint8_t address, uint8_t *buf, uint32_t len)
{
	/* If lenght is 0, only transmit address + START + STOP */
	if (len == 0) {
		return i2c1_transmit_address_only(address);
	}

	/* Transmit address + START */
	register uint32_t ret = i2c1_transmit_address(address);

	/* If error, return */
	if (ret) {
		return ret;
	}

	/* Transmit all bytes */
	for (int i = 0; i < len; i++) {
		/* Set STOP generation on last byte */
		if (i == len-1) {
			I2C1->CTRL |= (1 << 8);
		}
		/* Transfer byte, return if error */
		ret = i2c1_transmit_byte(*buf++);
		if (ret) {
			return ret;
		}
	}

	return 0;	
}
