/*
 * uart.h -- definitions for the UARTs
 */

#ifndef _UART_H
#define _UART_H

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

#define UART_STOP1 (0 << 0)
#define UART_STOP2 (1 << 0)
#define UART_SIZE7 (3 << 2)
#define UART_SIZE8 (0 << 2)
#define UART_SIZE9 (2 << 2)
#define UART_PARITY_NONE (0 << 4)
#define UART_PARITY_EVEN (2 << 4)
#define UART_PARITY_ODD (3 << 4)
#define UART_RCIE (1 << 6)
#define UART_TCIE (1 << 7)

#define UART_FE (1 << 0)
#define UART_RF (1 << 1)
#define UART_RC (1 << 2)
#define UART_PE (1 << 3)
#define UART_TC (1 << 4)

#define UART_CTRL_NONE (0)

#ifdef __cplusplus
}
#endif

#endif
