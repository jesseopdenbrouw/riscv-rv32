
/*
 * csr.h -- some common routines for CSR handling
 */


#ifndef _CSR_H
#define _CSR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Get the number of clock cycles */
uint64_t csr_get_cycle(void);
/* Get the time in micro seconds */
uint64_t csr_get_time(void);
/* Get the number of retired instructions */
uint64_t csr_get_instret(void);

/* Some macros to read/write CSRs, based on */
/* https://github.com/torvalds/linux/blob/master/arch/riscv/include/asm/csr.h */
#define csr_read(csr)  			     		\
({											\
	register uint32_t __v;					\
	__asm__ __volatile__ ("csrr %0, " #csr	\
						  : "=r" (__v) :	\
						  : "memory");		\
	__v;									\
})

#define csr_write(csr, val)						\
({												\
	uint32_t __v = (uint32_t)(val);				\
	__asm__ __volatile__ ("csrw " #csr ", %0"	\
			      : : "rK" (__v)				\
			      : "memory");					\
})

#define csr_read_set(csr, val)						\
({													\
	uint32_t __v = (uint32_t)(val);					\
	__asm__ __volatile__ ("csrrs %0, " #csr ", %1"	\
			      : "=r" (__v) : "rK" (__v)			\
			      : "memory");						\
	__v;											\
})

#define csr_set(csr, val)						\
({												\
	uint32_t __v = (uint32_t)(val);i			\
	__asm__ __volatile__ ("csrs " #csr ", %0"	\
			      : : "rK" (__v)				\
			      : "memory");					\
})

#define csr_read_clear(csr, val)					\
({													\
	uint32_t __v = (uint32_t)(val);					\
	__asm__ __volatile__ ("csrrc %0, " #csr ", %1"	\
			      : "=r" (__v) : "rK" (__v)			\
			      : "memory");						\
	__v;											\
})

#define csr_clear(csr, val)						\
({								                \
	uint32_t __v = (uint32_t)(val);				\
	__asm__ __volatile__ ("csrc " #csr ", %0"	\
			      : : "rK" (__v)				\
			      : "memory");					\
})

#define csr_swap(csr, val)							\
({													\
	uint32_t __v = (uint32_t)(val);					\
	__asm__ __volatile__ ("csrrw %0, " #csr ", %1"	\
			      : "=r" (__v) : "rK" (__v)			\
			      : "memory");						\
	__v;											\
})

#ifdef __cplusplus
}
#endif

#endif
