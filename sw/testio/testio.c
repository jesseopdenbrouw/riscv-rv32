#include <stdint.h>

#include "io.h"

int main(void) {

	while (1) {
		GPIOA_POUT = GPIOA_PIN;
	}
}
