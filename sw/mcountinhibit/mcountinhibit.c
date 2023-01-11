#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include <thuasrv32.h>

#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif

void print_cycle(void)
{
	char buffer[60];
	uint64_t instret = csr_get_cycle();

	snprintf(buffer, sizeof buffer, "cycle: %lu\r", (uint32_t)instret);
	uart1_puts(buffer);
	
}

void print_instret(void)
{
	char buffer[60];
	uint64_t instret = csr_get_instret();

	snprintf(buffer, sizeof buffer, "instret: %lu | ", (uint32_t)instret);
	uart1_puts(buffer);
	
}

int main(int argc, char *argv[])
{
	char buffer[60];
	uint32_t start;

	uart1_init(UART_PRESCALER(BAUD_RATE), 0x00);
	uart1_puts("\r\n");
	uart1_puts(argv[0]);
	uart1_puts("\r\n");
	start = clock();
	snprintf(buffer, sizeof buffer, "Clock: %lu\r\n", start);
	uart1_puts(buffer);

	/* Inhibit all counters */
	csr_write(mcountinhibit, -1);

	/* Show the result */
	uart1_puts("Counters should be stopped.\r\n");
	for (int i = 0; i < 1000; i++) {
		print_instret();
		print_cycle();
	}
	uart1_puts("\r\n");

	/* Start all counters */
	csr_write(mcountinhibit, 0);

	uart1_puts("Counters should be running.\r\n");
	/* Show the result */
	for (int i = 0; i < 1000; i++) {
		print_instret();
		print_cycle();
	}
	uart1_puts("\r\n");

	return 0;
}
