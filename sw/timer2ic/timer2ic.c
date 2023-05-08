#include <stdint.h>

#include <thuasrv32.h>

/* Should be loaded by the Makefile */
#ifndef F_CPU
#define F_CPU (50000000UL)
#endif
#ifndef BAUD_RATE
#define BAUD_RATE (9600UL)
#endif

int main(void)
{
	uint32_t cmpt;
	uint32_t speed = csr_read(0xfc1);

	uart1_init(BAUD_RATE, UART_CTRL_NONE);
	uart1_puts("TIMER2 Input Capture\r\n");

	speed = (speed == 0) ? F_CPU : speed;

	/* Activate TIMER2 with a cycle of 100 Hz */
	TIMER2->CMPT = cmpt = 0xffffUL; //speed/2000UL-1UL;
	uart1_printf("CMPT: %d\r\n", cmpt);
	/* Prescaler 0 */
	TIMER2->PRSC = 19UL;
	/* Timer2 OCA is PWM, 0%, adjusted in loop */
	TIMER2->CMPA = 0UL;
	/* Timer2 OCB is PWM, 50% */
	//TIMER2->CMPB = 2UL*speed/40000UL;
	/* Timer2 OCC is PWM, 75% */
	//TIMER2->CMPC = 3UL*speed/40000UL;
	/* Enable timer 2
	 * CMPT compare match, start phase low
	 * CMPA/B/C PWM, start phase low
	 * Preload enable on all count registers
	 * 31: FOCC is off (force OC)
	 * 30: FOCB is off
	 * 29: FOCA is off
	 * 28: FOCT is off
	 * 27: PHAC is 0 (start phase)
	 * 26-24: MODEC is PWM
	 * 23: PHAB is 0
	 * 22-20: MODEB is PWM
	 * 19: PHAA is 0
	 * 18-16: MODEA is IC
	 * 15: PHAT is 0
	 * 14-12: MODET is off
	 * 11: PREC is 0 (preload)
	 * 10: PREB is 0
	 *  9: PREA is 0
	 *  8: PRET is 0
	 *  7: CIE is 0 (interrupt enable)
	 *  6: BIE is 0
	 *  5: AIE is 0
	 *  4: TIE is 0
	 *  3: OS is 0 (one-shot)
	 *  2-1: reserved
	 *  0: EN is 1 (timer enable) */
	TIMER2->CTRL = 0x00060001UL;

	while (1) {
		/* Clear counter */
		TIMER2->CNTR = 0UL;
		/* Clear flag */
		TIMER2->STAT = 0x00;
		/* Wait for first Input Capture */
		while ((TIMER2->STAT & (1 << 5)) == 0x00) {}
		/* Get value */
		uint32_t first = TIMER2->CMPA;
		/* Clear flag */
		TIMER2->STAT = 0x00;
		/* Wait for second Input Capture */
		while ((TIMER2->STAT & (1 << 5)) == 0x00) {}
		/* Get value */
		uint32_t second = TIMER2->CMPA;
		uart1_printf("First: %d, second: %d, diff: %d\r\n", first, second, second-first);
		delayms(1000);
	}

	return 0;
}