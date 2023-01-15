/*
Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Original Author: Shay Gal-on
*/

#include "coremark.h"
#include "core_portme.h"

/* Ported to THUASRV32 by Jesse op den Brouw */
/* Based on NEORV32 by S.T. Nolting */

#include <thuasrv32.h>
#include <stdio.h>
/* Currenly. the [m]instret counter cannot be cleared */
uint64_t start_instret = 0;


#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;
/* Porting : Timing functions
	      How to capture time and convert to seconds must be ported to whatever is
	 supported by the platform. e.g. Read value from on board RTC, read value from
	 cpu clock cycles performance counter etc. Sample implementation for standard
	 time.h and windows.h definitions included.
*/
CORETIMETYPE
barebones_clock()
{
/*
#error \
	  "You must implement a method to measure time in barebones_clock()! This function should return current time.\n"
*/
	return 1;
}
/* Define : TIMER_RES_DIVIDER
	      Divider to trade off timer resolution and total time that can be
	 measured.

	      Use lower values to increase resolution, but make sure that overflow
	 does not occur. If there are issues with the return value overflowing,
	 increase this value.
	      */
#define GETMYTIME(_t)              (*_t = (CORETIMETYPE)csr_get_cycle())
#define MYTIMEDIFF(fin, ini)       ((fin) - (ini))
#define TIMER_RES_DIVIDER          1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC           (CLOCKS_PER_SEC / TIMER_RES_DIVIDER)

/** Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;

/* Function : start_time
	      This function will be called right before starting the timed portion of
	 the benchmark.

	      Implementation may be capturing a system timer (as implemented in the
	 example code) or zeroing some system parameters - e.g. setting the cpu clocks
	 cycles to 0.
*/
void
start_time(void)
{
	GETMYTIME(&start_time_val);
	/* Enable all counters */
	csr_write(mcountinhibit, 0);

}
/* Function : stop_time
	      This function will be called right after ending the timed portion of the
	 benchmark.

	      Implementation may be capturing a system timer (as implemented in the
	 example code) or other system parameters - e.g. reading the current value of
	 cpu cycles counter.
*/
void
stop_time(void)
{

	/* Stop all counters */
	csr_write(mcountinhibit, -1);
	GETMYTIME(&stop_time_val);
	
}
/* Function : get_time
	      Return an abstract "ticks" number that signifies time on the system.

	      Actual value returned may be cpu cycles, milliseconds or any other
	 value, as long as it can be converted to seconds by <time_in_secs>. This
	 methodology is taken to accommodate any hardware or simulated platform. The
	 sample implementation returns milliseconds by default, and the resolution is
	 controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS
get_time(void)
{
	  CORE_TICKS elapsed
	      = (CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
	  return elapsed;
}
/* Function : time_in_secs
	      Convert the value returned by get_time to seconds.

	      The <secs_ret> type is used to accomodate systems with no support for
	 floating point. Default implementation implemented by the EE_TICKS_PER_SEC
	 macro above.
*/
secs_ret
time_in_secs(CORE_TICKS ticks)
{
	  /* THUASRV32-specific */
	  secs_ret retval = (secs_ret)(((CORE_TICKS)ticks) / ((CORE_TICKS)F_CPU));
	  return retval;
}

ee_u32 default_num_contexts = 1;


/* Function : portable_init
	      Target specific initialization code
	      Test for some common mistakes.
*/
#ifndef RUN_COREMARK
void
__attribute__((__noreturn__))
portable_init(core_portable *p, int *argc, char *argv[])
#else
void
portable_init(core_portable *p, int *argc, char *argv[])
#endif
{
	/* THUASRV32-specific */
	char buffer[80];
	disable_irq(); // no interrupt, thanks
	uart1_init(F_CPU/BAUD_RATE-1, 0x00);

	uart1_puts("\r\n\r\nTHUASRV32: starting CoreMark\r\n");

// Disable coremark compilation by default
#ifndef RUN_COREMARK
	#warning COREMARK HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_COREMARK clean_all exe<< to compile it.

	// inform the user if you are actually executing this
	uart1_puts("ERROR! CoreMark has not been compiled. Use >>make USER_FLAGS+=-DRUN_COREMARK clean_all exe<< to compile it.\r\n");

	while(1);
#endif

	snprintf(buffer, sizeof buffer, "THUASRV32: Processor running at %lu Hz\r\n", (uint32_t)F_CPU);
	uart1_puts(buffer);
	snprintf(buffer, sizeof buffer, "THUASRV32: Executing coremark (%lu iterations). This may take some time...\r\n\r\n", (uint32_t)ITERATIONS);
	uart1_puts(buffer);

/*
#error \
	  "Call board initialization routines in portable init (if needed), in particular initialize UART!\n"
*/
	  if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
	  {
	      ee_printf(
	          "ERROR! Please define ee_ptr_int to a type that holds a "
	          "pointer!\n");
	  }
	  if (sizeof(ee_u32) != 4)
	  {
	      ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
	  }
	  p->portable_id = 1;

	/* [m]instret is not set to 0 if we start */
	/* Currently, [m]instret cannot be cleared, so we have to get
	 * a starting point for later */
	start_instret = csr_get_instret();

#ifndef RUN_COREMARK
	while(1);
#endif
}


/* Function : portable_fini
	      Target specific final code
*/
void
portable_fini(core_portable *p)
{
	char buffer[80];
	  p->portable_id = 0;

	// show executed instructions, required cycles and resulting average CPI
	union {
	  uint64_t uint64;
	  uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
	} exe_instructions, exe_time;

	exe_time.uint64 = (uint64_t)get_time();
	exe_instructions.uint64 = csr_get_instret() - start_instret;

	snprintf(buffer, sizeof buffer, "THUASRV32: Executed instructions:       0x%08lx%08lx\r\n", (uint32_t)exe_instructions.uint32[1], (uint32_t)exe_instructions.uint32[0]);
	uart1_puts(buffer);
	snprintf(buffer, sizeof buffer, "THUASRV32: CoreMark core clock cycles:  0x%08lx%08lx\r\n", (uint32_t)exe_time.uint32[1], (uint32_t)exe_time.uint32[0]);
	uart1_puts(buffer);

	snprintf(buffer, sizeof buffer, "THUASRV32: Avg CPI: %f clock/instr\r\n", (double)exe_time.uint64/(double)exe_instructions.uint64);
	uart1_puts(buffer);
	snprintf(buffer, sizeof buffer, "THUASRV32: Avg IPC: %f instr/clock\r\n", (double)exe_instructions.uint64/(double)exe_time.uint64);
	uart1_puts(buffer);

}
