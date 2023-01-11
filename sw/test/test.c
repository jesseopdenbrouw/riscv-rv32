#include <stdio.h>
#include <stdarg.h>

#include <thuasrv32.h>

/* 256 characters should be enough */
#define BUFFER_SIZE (256)

int main(void)
{
	char str[] = "Hello";
	int x = 3;
	uint32_t y = 0xff;

	uart1_init(UART_PRESCALER(BAUD_RATE), 0x00);
	uart1_printf("%s, %d, %p, %lu\r\n", str, x, str, y);
}
