#include <thuasrv32.h>

uint8_t i2c1_receive_byte(void)
{
	/* Wait for data transmission completed */
	while ((I2C1->STAT & 0x10) == 0x00);

	return I2C1->DATA;
} 
