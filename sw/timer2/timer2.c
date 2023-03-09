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

	/* Activate TIMER2 with a cycle of 1000 Hz */
	TIMER2->CMPT = cmpt = F_CPU/1000UL-1UL;
	/* Prescaler 0 */
	TIMER2->PRSC = 0UL;
	/* Timer2 OCA is PWM, 25% */
	TIMER2->CMPA = F_CPU/4000UL;
	/* Timer2 OCB is PWM, 50% */
	TIMER2->CMPB = 2UL*F_CPU/4000UL;
	/* Timer2 OCC is PWM, 75% */
	TIMER2->CMPC = 3UL*F_CPU/4000UL;
	/* Enable timer 2
	 * CMPT compare match, start phase high
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
	 * 18-16: MODEA is PWM
	 * 15: PHAT is 1
	 * 14-12: MODET is OC toggle
	 * 11: PREC is 1 (preload)
	 * 10: PREB is 1
	 *  9: PREA is 1
	 *  8: PRET is 1
	 *  7: CIE is 0 (interrupt enable)
	 *  6: BIE is 0
	 *  5: AIE is 0
	 *  4: TIE is 0
	 *  3: OS is 0 (one-shot)
	 *  2-1: reserved
	 *  0: EN is 1 (timer enable) */
	TIMER2->CTRL = 0x04449f01UL;

	while (1) {
		/* Set the PWM DC to i/CMPT*100% */
		for (uint32_t i = 0; i <= cmpt; i++) {
			TIMER2->CMPA = i;
			/* Small delay loop */
			for (volatile uint32_t w = 0; w < 500; w++);
		}	
	}

	return 0;
}
