#include <stdint.h>

int main(void) {

	volatile uint32_t x = 1;
	volatile int32_t y = -31;

	x = x << 4;
	y = y >> 4;
}
