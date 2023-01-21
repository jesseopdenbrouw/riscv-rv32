# Hardware

This directory contains the hardware description of the
THUAS RISC-V RV32IM 32-bit processor.


## riscv-pipe3-csr-md-lic

This is a version of the three-stage pipelined processor
and incorporates a hardcoded bootloader (can be switched off).
The bootloader is located at address 0x10000000. The bootloader
is able to load an S-record file into the ROM at address
0x00000000 using the `upload` program.

When the processor starts, the bootloader waits for about
5 seconds for a keboard press (using the UART). If not
within this 5 seconds, the bootloader starts the main
program at address 0x00000000. If pressed, the bootloader
enters a simple monitor program. Type 'h' for help.

A S-record file can be uploaded by the `upload` program.
If `upload` contacts the bootloader within the 5 second
delay, the S-record file is transmitted to the processorr
and the instructions are placed in the ROM (or RAM). Make
sure that NO terminal connection (e.g. Putty) is active.

This version includes a registers-in-RAM option. The clock
frequency will be lower that 80 MHz, but saves about 1000
ALM flip-flops.

## Status

Works on the DE0-CV board.
