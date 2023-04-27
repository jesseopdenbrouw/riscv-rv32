/*
 * ctors.c - Test to see if constructors and destructors work
 */

#include <thuasrv32.h>

__attribute__ ((constructor)) void foo(void)
{
	uart1_init(UART_PRESCALER(BAUD_RATE), UART_CTRL_NONE);
	uart1_puts("foo\r\n");
}

__attribute__ ((constructor)) void foo2(void)
{
	uart1_puts("foo2\r\n");
}

__attribute__ ((destructor)) void bar(void)
{
	uart1_puts("bar\r\n");
}

__attribute__ ((destructor)) void bar2(void)
{
	uart1_puts("bar2\r\n");
}


typedef void (*func_ptr_t)(void);
 
extern func_ptr_t __init_array_start[1], __init_array_end[1];
extern func_ptr_t __fini_array_start[1], __fini_array_end[1];

int main(int argc, char *argv[], char *env[])
{
	uart1_init(UART_PRESCALER(BAUD_RATE), UART_CTRL_NONE);

	uart1_puts("From linker symbols:\r\n");
	uart1_printf(">> %08x %p\r\n", *__init_array_start, __init_array_start);
	uart1_printf(">> %08x %p\r\n", *__init_array_end, __init_array_end);
	uart1_printf(">> %08x %p\r\n", *__fini_array_start, __fini_array_start);
	uart1_printf(">> %08x %p\r\n", *__fini_array_end, __fini_array_end);

	uart1_puts("Constructors:\r\n");
	for ( func_ptr_t* func = __init_array_start; func != __init_array_end; func++ ) {
		uart1_printf("-> Function address: %p, @%p\r\n", *func, func);
	}

	uart1_puts("Destructors:\r\n");
	for ( func_ptr_t* func = __fini_array_start; func != __fini_array_end; func++ ) {
		uart1_printf("-> Function address: %p, @%p\r\n", *func, func);
	}
}
