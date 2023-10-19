
# This is the startup section and must be the first to be called
  .section .text.start_up_code
#  .text
  .global _start
  .global Universal_Handler
  .type   _start, @function
  .type   Universal_Hander, @function
_start:
  # Initialize global pointer
.option push
.option norelax

	la gp, __global_pointer$
	la sp, __stack_pointer$ 

	# Register trap hanlder and enable
	# interrupts (direct mode)

	la t0, Universal_Handler
	csrw mtvec,t0
	li t0, 0x8 | (0x3<<11)
	csrw mstatus,t0

	nop
	nop
	la    x7,0xf0000000
	li    x8,(1 << 1)
	sw    x8,24(x7)

	li    x8,1000
	li    x9,3
	nop
	nop
	nop
	nop
	nop
	nop
	div   x10,x8,x9
	#csrrw x8,mscratch,x9
	nop
	nop

einde:	j einde

	nop
	nop
	nop
	nop
	nop
	nop

	nop
	nop
	nop

	lw x5,0(x0)

	nop
	nop


Universal_Handler:
	nop
	nop
	nop
	# Load mcause to check for interrupt
	csrr x10,mcause
	blt  x10,x0,over

	# Add 4 to MEPC to skip the offending instruction
	csrr x10,mepc
	addi x10,x10,4
	csrw mepc,x9

over:
	nop

	sw x0,28(x7)
	nop

	mret
#	j Universal_Handler

.option pop

  .size  _start, .-_start
