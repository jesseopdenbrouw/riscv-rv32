/*
 *     io.h - definitions for the I/O of the
 *            THUAS RISC-V processor
 */

#ifndef _IO_H
#define _IO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Base address of the I/O */
#define IO_BASE (0xf0000000UL)


/*
 * General purpose I/O
 */
typedef struct {
        volatile uint32_t PIN;
        volatile uint32_t POUT;
} GPIO_struct_t;

#define GPIOA_BASE (IO_BASE+0x00000000UL)
#define GPIOA ((GPIO_struct_t *) GPIOA_BASE)

#define GPIOA_PIN  (*(volatile uint32_t*)(GPIOA_BASE+0x00000000UL))
#define GPIOA_POUT (*(volatile uint32_t*)(GPIOA_BASE+0x00000004UL))


/* UART1 */
typedef struct {
	volatile uint32_t DATA;
	volatile uint32_t BAUD;
	volatile uint32_t CTRL;
	volatile uint32_t STAT;
} UART_struct_t;

#define UART_BASE (IO_BASE+0x00000020UL)
#define UART1 ((UART_struct_t *) UART_BASE)

#define UART1_DATA (*(volatile uint32_t*)(UART_BASE+0x00000000UL))
#define UART1_BAUD (*(volatile uint32_t*)(UART_BASE+0x00000004UL))
#define UART1_CTRL (*(volatile uint32_t*)(UART_BASE+0x00000008UL))
#define UART1_STAT (*(volatile uint32_t*)(UART_BASE+0x0000000cUL))


/*
 * I2C1
*/
typedef struct {
	volatile uint32_t CTRL;
	volatile uint32_t STAT;
	volatile uint32_t DATA;
} I2C_struct_t;

#define I2C_BASE (IO_BASE+0x00000040UL)
#define I2C1 ((I2C_struct_t *) I2C_BASE)

#define I2C1_CTRL (*(volatile uint32_t*)(I2C_BASE+0x00000000UL))
#define I2C1_STAT (*(volatile uint32_t*)(I2C_BASE+0x00000004UL))
#define I2C1_DATA (*(volatile uint32_t*)(I2C_BASE+0x00000008UL))


/*
 * SPI1, SPI2
 */
typedef struct {
	volatile uint32_t CTRL;
	volatile uint32_t STAT;
	volatile uint32_t DATA;
} SPI_struct_t;

#define SPI_BASE (IO_BASE+0x00000060UL)
#define SPI1 ((SPI_struct_t *) SPI_BASE)
#define SPI2 ((SPI_struct_t *) (SPI_BASE + 0x10))

#define SPI1_CTRL (*(volatile uint32_t*)(SPI_BASE+0x00000000UL))
#define SPI1_STAT (*(volatile uint32_t*)(SPI_BASE+0x00000004UL))
#define SPI1_DATA (*(volatile uint32_t*)(SPI_BASE+0x00000008UL))
#define SPI2_CTRL (*(volatile uint32_t*)(SPI_BASE+0x00000010UL))
#define SPI2_STAT (*(volatile uint32_t*)(SPI_BASE+0x00000014UL))
#define SPI2_DATA (*(volatile uint32_t*)(SPI_BASE+0x00000018UL))


/*
 * TIMER1
 */
typedef struct {
	volatile uint32_t CTRL;
	volatile uint32_t STAT;
	volatile uint32_t CNTR;
	volatile uint32_t CMPT;
} TIMER1_struct_t;

#define TIMER1_BASE (IO_BASE+0x00000080UL)
#define TIMER1 ((TIMER1_struct_t *) TIMER1_BASE)

#define TIMER1_CTRL (*(volatile uint32_t*)(TIMER1_BASE+0x00000000UL))
#define TIMER1_STAT (*(volatile uint32_t*)(TIMER1_BASE+0x00000004UL))
#define TIMER1_CNTR (*(volatile uint32_t*)(TIMER1_BASE+0x00000008UL))
#define TIMER1_CMPT (*(volatile uint32_t*)(TIMER1_BASE+0x0000000cUL))


/*
 * TIMER2
 */
typedef struct {
	volatile uint32_t CTRL;
	volatile uint32_t STAT;
	volatile uint32_t CNTR;
	volatile uint32_t CMPT;
	volatile uint32_t PRSC;
	volatile uint32_t CMPA;
	volatile uint32_t CMPB;
	volatile uint32_t CMPC;
} TIMER2_struct_t;

#define TIMER2_BASE (IO_BASE+0x000000a0UL)
#define TIMER2 ((TIMER2_struct_t *) TIMER2_BASE)

#define TIMER2_CTRL (*(volatile uint32_t*)(TIMER2_BASE+0x00000000UL))
#define TIMER2_STAT (*(volatile uint32_t*)(TIMER2_BASE+0x00000004UL))
#define TIMER2_CNTR (*(volatile uint32_t*)(TIMER2_BASE+0x00000008UL))
#define TIMER2_CMPT (*(volatile uint32_t*)(TIMER2_BASE+0x0000000cUL))
#define TIMER2_PRSC (*(volatile uint32_t*)(TIMER2_BASE+0x00000010UL))
#define TIMER2_CMPA (*(volatile uint32_t*)(TIMER2_BASE+0x00000014UL))
#define TIMER2_CMPB (*(volatile uint32_t*)(TIMER2_BASE+0x00000018UL))
#define TIMER2_CMPC (*(volatile uint32_t*)(TIMER2_BASE+0x0000001cUL))


/*
 * RISC-V system timer (in I/O)i
 */
#define MTIME (*(volatile uint32_t*)(IO_BASE+0x000000f0UL))
#define MTIMEH (*(volatile uint32_t*)(IO_BASE+0x000000f4UL))
#define MTIMECMP (*(volatile uint32_t*)(IO_BASE+0x000000f8UL))
#define MTIMECMPH (*(volatile uint32_t*)(IO_BASE+0x000000fcUL))

typedef struct {
	volatile uint32_t time;
	volatile uint32_t timeh;
} MTIME_struct_t;

typedef struct {
	volatile uint32_t timecmp;
	volatile uint32_t timecmph;
} MTIMECMP_struct_t;

#ifdef __cplusplus
}
#endif

#endif
