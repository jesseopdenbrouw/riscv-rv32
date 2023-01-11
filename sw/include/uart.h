/*
 * uart.h -- definitions for the UARTs
 */

#ifndef _uart1_H
#define _uart1_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize UART1 */
void uart1_init(uint32_t prescaler, uint32_t ctrl);
/* Write one character to UART1 */
void uart1_putc(int ch);
/* Write null-terminated string to UART1 */
void uart1_puts(char *s);
/* Get one character from UART1 */
int uart1_getc(void);
/* Check if character is available */
int uart1_available(void);
/* Get maximum size-1 characters in string buffer from UART1 */
int uart1_gets(char buffer[], int size);
/* Print formatted to the uart */
int uart1_printf(const char *format, ...);

#define UART_PRESCALER(A) ((F_CPU)/(A)-1)

#ifdef __cplusplus
}
#endif

#endif
