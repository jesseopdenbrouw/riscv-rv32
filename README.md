# riscv-rv32im

A RISC-V 32-bit microcontroller written in VHDL targeted
for an FPGA.

## Description

The RISC-V microcontroller uses the RV32IM instruction set
and the Zicsr and Zicntr extensions. The microcontroller
supports exceptions and interrupts. `ECALL`, `EBREAK`
and `MRET` are supported. `WFI` acts as a no-operation
(`NOP`). Currently only machine mode is supported. We
successfully tested a complex program with interrupts
and exceptions and implemented a basic syscall library usable
with the `ECALL` instruction as provided by the GNU C
compiler for RISC-V. `sbrk`, `read`, `write`, `times` and
`gettimeofday` are supported. The External (system) Timer
is implemented and generates an interrupt if `time` >=
`timecmp`. The processor can handle up to 16 fast local
interrupts. Reads from ROM, RAM and I/O require 2 clock
cycles. Writes require 1 clock cycles. Multiplications
require 3 clock cycles, divisions require 16+2 clock cycles,
CSR accesses take 1 clock cycle.
Jumps/calls/branches taken require 3 clock cycles, the
processor does not implement branch prediction. Interrupts
are direct or vectored. Current Coremark testbench shows
a CPI of 1.53.

Software is written in C, (C++ is supported but there are
some limitations) and compiled using the RISC-V GNU C/C++
compiler.

## Current flavor

The design is equipped with a bootloader program and registers in onboard RAM.
The bootloader can be removed from synthesis. The registers can be placed in
logic cells. The design runs at a speed of approximately 75 MHz.
 
## Memory

The microcontroller uses FPGA onboard RAM blocks to emulate RAM
and program ROM. There is no suport for external RAM. Programs
are compiled with the GNU C compiler for RISC-V and the resulting
executable is transformed to a VHDL synthesizable ROM table.

* ROM: a ROM of 64 kB is available (placed in onboard RAM, may be extended).
* BOOT: a bootloader ROM of 4 kB (placed in onboard RAM).
* RAM: a RAM of 32 kB using onboard RAM block available (may be extended).
* I/O: a simple 32-bit input and 32-bit output is available, as
is a simple 7/8/9-bit UART with interrupt capabilities. Two SPI devices are
available, with one device used for SD card socket (no interrupt) and a
general purpose SPI device with hardware NSS. One I2C device is
available. A simple timer
with interrupt is provided. A more elaborate timer is included and can
generate waveforms (Output Compare and PWM). The External (system) Timer is
located in the I/O so it's memory mapped.

ROM starts at 0x00000000, BOOT (if available) starts at 0x10000000,
RAM starts at0x20000000, I/O starts at 0xF0000000. May be changed
on 256 MB (top 4 bits) sections.

## CSR

A number CSR registers are implemented: `time`, `timeh`, `cycle`, `cycleh`,
`instret`, `instreth`, `mvendorid`, `marchid`, `mimpid`, `mhartid`, `mstatus`,
`mstatush`, `misa`, `mie`, `mtvec`, `mscratch`, `mepc`, `mcause`, `mip`,
`mcountinhibit`. Some of these CSRs are hardwired. Others will be implemented
when needed. The `time` and `timeh` CSRs produces the time since reset in
microseconds, shadowed from the External Timer memory mapped registers.

## Software

A number of C programs have been tested, created by the GNU C/C++ Compiler for
RISC-V. We tested the use of (software) floating point operations (both
float and double) and tested the mathematical library (sin, cos, et al.).
Traps (interrupts and exceptions) are tested and work.
Assembler programs can be compiled by the C/C++ compiler. We provide a CRT
(C startup) and linker file. C++ is supported but many language concepts
(e.g. cout with iostream) create a binary that is too big to fit in the
ROM.

We provide a basic set of systems call, trapped (ECALL) and non-trapped
(functions overriding the C library functions). Trapped system calls
are by default set up by the RISC-V C/C++ compiler, so no extra handling
is needed.

## FPGA

The microcontroller is developed on a Cyclone V FPGA (5CEBA4F23C7)
with the use of the DE0-CV board by Terasic and Intel Quartus Prime
Lite 22.0. Simulation is possible with QuestaSim Intel Starter Edition.
You need a (free) license for that. The processor uses about
2800 ALM (cells) of 18480, depending on the settings. In the default
settings, ROM, BOOT, RAM and registers uses 43% of the available RAM blocks.

## Plans (or not) and issues

* We are *not* planning the C standard.
* Implement clock stretching and arbitration in the I2C1 peripheral.
* Adding input synchronization for SPI1/SPI2 peripherals.
* Adding Input Capture for TIMER2.
* Implement an I/O input/output multiplexer for pina and pouta. This will enable I/O functions to be multiplexed with normal port I/O.
* Smaller (in cells) divide unit.
* Test more functions of the standard and mathematical libraries.
* It is not possible to print `long long` (i.e. 64-bit) using `printf` et al. When using the format specifier `%lld`, `printf` just prints `ld`. This due to lack of support in the `nano` library.
* Further optimize the ALU for size and speed.
* The `time` (TIMEH:TIME) registers are currently read only, but should be writable.
* Move CSR and LIC into the core.
* A new signal `I_memvma` and `O_memvma` are introduced to grant read and write access if no interrupt is pending. This lowers the overall clock speed to 75 MHz.

## Disclaimer

This microcontroller is for educational purposes only.
Work in progress. Things might change. Use with care.

