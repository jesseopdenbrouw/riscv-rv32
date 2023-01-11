#
# Makefile for creating the system call + utilities library
# for the THUAS RISC-V Minimal project.
# See: https://github.com/jesseopdenbrouw/riscv-minimal
#

CC = riscv32-unknown-elf-gcc
AR = riscv32-unknown-elf-ar

CFLAGS = -I../../include -Wall -O2 -g -march=rv32im -mabi=ilp32

SRC = $(wildcard *.c)
OBJ = $(patsubst %.c, %.o, $(SRC))

all: $(OBJ) 

%o: %c
	$(CC) -c $< -o $@

clean:
	rm -f $(OBJ)
