
# Introduction

CoreMark's primary goals are simplicity and providing a method for testing only a processor's core features. For more information about EEMBC's comprehensive embedded benchmark suites, please see www.eembc.org.

For a more compute-intensive version of CoreMark that uses larger datasets and execution loops taken from common applications, please check out EEMBC's [CoreMark-PRO](https://www.github.com/eembc/coremark-pro) benchmark, also on GitHub.

This project folder is a port of CoreMark (from the official [GitHub repository](https://github.com/eembc/coremark)) for the NEORV32 processor.

# Building

To build the executable (`coremark.srec`) of the benchmark, type:

`> make USER_FLAGS+=-DRUN_COREMARK clean_all exe`

Make sure to define `RUN_COREMARK` *when invoking* `make` (via `USER_FLAGS+=-DRUN_COREMARK`).

To build the executable for a certain CPU configuration and a certain optimization level of the benchmark, type (`Ofast` in this example):

`> make USER_FLAGS+=-DRUN_COREMARK EFFORT=-Ofast clean_all exe`


# Running

Upload the generated executable `coremark.srec` with the `upload` program: `make upload`

```
THUAS RISC-V Bootloader v0.2                                           
**********
                                                                                      
THUASRV32: starting CoreMark
THUASRV32: Processor running at 50000000 Hz
THUASRV32: Executing coremark (2000 iterations). This may take some time...

2K performance run parameters for coremark. 
CoreMark Size    : 666
Total ticks      : 903162 k
Total time (secs): 18
Iterations/Sec   : 111
Iterations       : 2000
Compiler version : GCC12.1.0
Compiler flags   : see makefile
Memory location  : STATIC
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x4983
Correct operation validated. See README.md for run and reporting rules.
                                                                      
THUASRV32: All reported numbers only show the integer part.
THUASRV32: Executed instructions:       0x000000002331266e
THUASRV32: CoreMark core clock cycles:  0x0000000035d52c54
THUASRV32: Avg CPI: 1.529686 clock/instr
THUASRV32: Avg IPC: 0.653729 instr/clock
