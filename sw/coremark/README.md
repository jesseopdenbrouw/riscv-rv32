
# Introduction

CoreMark's primary goals are simplicity and providing a method for testing only a processor's core features. For more information about EEMBC's comprehensive embedded benchmark suites, please see www.eembc.org.

For a more compute-intensive version of CoreMark that uses larger datasets and execution loops taken from common applications, please check out EEMBC's [CoreMark-PRO](https://www.github.com/eembc/coremark-pro) benchmark, also on GitHub.

This project folder is a port of CoreMark (from the official [GitHub repository](https://github.com/eembc/coremark)) for the processor. Based on the NEORV32 version.

# Building

To build the executable (`coremark.srec`) of the benchmark, type:

`> make clean all`

To build the executable for a certain CPU configuration and a certain optimization level of the benchmark, type (`Ofast` in this example):

`> make EFFORT=-Ofast clean all`


# Running

Upload the generated executable `coremark.srec` with the `upload` program: `make upload`

```
THUAS RISC-V Bootloader v0.4
Clock frequency: 50000000
**********

THUASRV32: starting CoreMark
THUASRV32: Processor running at 50000000 Hz
THUASRV32: Executing coremark (4000 iterations). This may take some time...

2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 1806326 k
Total time (secs): 36
Iterations/Sec   : 111
Iterations       : 4000
Compiler version : GCC12.2.0
Compiler flags   : see makefile
Memory location  : STATIC
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
Correct operation validated. See README.md for run and reporting rules.
THUASRV32: Executed instructions: 1180857655
THUASRV32: CoreMark core clock cycles: 1806326068
THUASRV32: Avg CPI: 1.529673 clock/instr
THUASRV32: Avg IPC: 0.653734 instr/clock
```
