#  Common settings for the programs

# Compiler defaults
CC = riscv32-unknown-elf-gcc
OBJCOPY = riscv32-unknown-elf-objcopy
AR = riscv32-unknown-elf-ar
SIZE = riscv32-unknown-elf-size
SREC2VHDL = ../bin/srec2vhdl
UPLOAD = ../bin/upload

# The clock frequency of the system
ifndef F_CPU
F_CPU = "(50000000UL)"
endif

# The default baud rate of the USART
ifndef BAUD_RATE
BAUD_RATE = "(115200UL)"
endif

# Set to program name in Makefile
ifndef PROG_NAME
PROG_NAME = \"mad_cow\"
endif

# Set to the startup file
CRT_PATH = ../crt
CRT = startup.c

# Needed for binutils >= 2.39, set to empty otherwise
EXTRA_LINKER_FLAGS=-Wl,--no-warn-rwx-segments

# Linker script
LD_SCRIPT = ../ldfiles/riscv.ld

# Include dir
INCPATH = ../include

# THUAS RISCV library and set search path
LIBTHUASRV32STRING = -lthuasrv32 -L../lib

# Architecture and ABI
MARCHABISTRING = -march=rv32im -mabi=ilp32

# Linker specs files
SPECSSTRING = --specs=../lib/libsys.specs --specs=../lib/nano.specs
