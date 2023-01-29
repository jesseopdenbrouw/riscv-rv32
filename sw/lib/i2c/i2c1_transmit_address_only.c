#include <thuasrv32.h>

uint32_t i2c1_transmit_address_only(uint8_t address)
{
	/* Set start and stop bit generation */
	I2C1->CTRL |= (1 << 9) | (1 << 8);

	/* Send address and R/W bit */
	I2C1->DATA = address;

	/* Wait for transmission to end */
	while ((I2C1->STAT & 0x08) == 0x00);

	return I2C1->STAT & (1 << 5);
} 
