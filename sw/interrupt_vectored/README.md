# Interrupt

Simple code to test if ECALL, EBREAK and
hardware interrupts work.
This version uses mtvec vectored mode.

The External Timer interrupt is set to 1000 Hz.
The TIMER1 interrupt is set to 100 Hz.

## Note

This example uses a different startup.c because
it uses interrupts

## Status

Works on the DE0-CV board.
