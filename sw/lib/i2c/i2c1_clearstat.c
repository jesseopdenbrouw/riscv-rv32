#include <thuasrv32.h>

void i2c1_clearstat(void)
{
	I2C1->STAT = 0x00;
} 
