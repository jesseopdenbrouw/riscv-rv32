# Interrupt

Simple code to test if ECALL, EBREAK and hardware interrupts work.
This version uses mtvec vectored mode.

The External Timer interrupt is set to 1000 Hz.
The TIMER1 interrupt is set to 100 Hz.
The TIMER2 interrupt is set to 1 Hz
SPI1 and I2C1 interrupts are triggered once in 10 sec, by executing a transmit action
The UART receive interrupt is also active.

The low switch of the DE0-CV board triggers a default handler, which holds the processor

## Status

Works on the DE0-CV board.
