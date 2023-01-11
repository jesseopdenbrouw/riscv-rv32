#
# Makefile for creating the system call + utilities library
# for the THUAS RISC-V Minimal project.
# See: https://github.com/jesseopdenbrouw/riscv-minimal
#

CC = riscv32-unknown-elf-gcc
AR = riscv32-unknown-elf-ar

CFLAGS = -I../include -Wall -O2 -g -march=rv32im -mabi=ilp32

LIBTHUASRV32 = libthuasrv32.a
SRC = $(wildcard */*.c)
OBJ = $(patsubst %.c, %.o, $(SRC))

all:
	make -C syscalls all
	make -C uart all
	make -C util all
	make -C csr all
	$(AR) rsv $(LIBTHUASRV32) $(OBJ)


clean:
	make -C syscalls clean
	make -C uart clean
	make -C util clean
	make -C csr clean
	rm -f $(LIBTHUASRV32)
