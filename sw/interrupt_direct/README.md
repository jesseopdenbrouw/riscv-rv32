# Interrupt

Simple code to test if ECALL, EBREAK and
hardware interrupts work.
Can only be used on riscv-pipe3-csr-md-lic.bootloader
This version uses mtvec direct mode

The External Timer interrupt is set to 10 Hz.
The TIMER1 interrupt is set to 2 Hz.

## Note

This example uses a different startup.c file because
it is using interrupts.

## Status

Works on the board.
