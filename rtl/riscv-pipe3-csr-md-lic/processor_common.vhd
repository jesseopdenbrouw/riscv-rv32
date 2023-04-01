
-- #################################################################################################
-- # processor_common.vhd - Common types and constants                                             #
-- # ********************************************************************************************* #
-- # This file is part of the THUAS RISCV RV32 Project                                             #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Jesse op den Brouw. All rights reserved.                                  #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # https:/github.com/jesseopdenbrouw/riscv-minimal                                               #
-- #################################################################################################

-- This file contains the used data types and build options for the design

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package processor_common is

    -- Customization constants
    -- Set to board frequency
    constant SYSTEM_FREQUENCY : integer := 50000000;
    -- Set to 1 MHz, so elapsed time is in micro seconds, keep this!
    constant CLOCK_FREQUENCY : integer := 1000000;
    
    -- Make use of the E standard (reduced register set),
    -- set to 16. For the I standard, use 32.
    constant NUMBER_OF_REGISTERS : integer := 32;
    
    -- Do we have the integer multiply/divide unit?
    constant HAVE_MULDIV : boolean := TRUE;
    -- Fast divide (needs more area)?
    constant FAST_DIVIDE : boolean := TRUE;
    
    -- Do we enable vectored mode for mtvec?
    constant VECTORED_MTVEC : boolean := TRUE;

    -- mstatus.MIE disables only interrupts (true) or all traps (false)
    constant MSTATUS_MIE_DISABLES_INTERRUPTS : boolean := TRUE;
    
    -- Do we have a bootloader ROM?
    constant HAVE_BOOTLOADER_ROM : boolean := TRUE;
    
    -- Do we have registers is RAM?
    constant HAVE_REGISTERS_IN_RAM : boolean := TRUE;
    
    -- Do we have UART1?
    constant HAVE_UART1 : boolean := TRUE;
    
    -- Do we have SPI1?
    constant HAVE_SPI1 : boolean := TRUE;
    
    -- Do we have SPI2?
    constant HAVE_SPI2 : boolean := TRUE;

    -- Do we have I2C1?
    constant HAVE_I2C1 : boolean := TRUE;
    
    -- Do we have TIMER1?
    constant HAVE_TIMER1 : boolean := TRUE;
    
    -- Do we have TIMER2?
    constant HAVE_TIMER2 : boolean := TRUE;

    
    -- Used data types
    -- The common data type is 32 bits wide
    subtype data_type is std_logic_vector(31 downto 0);
    
    -- For shifts with immediate operand
    subtype shift_type is std_logic_vector(4 downto 0);
    
    -- For selecting registers
    subtype reg_type is std_logic_vector(4 downto 0);
    
    -- Opcode is 7 bits in instruction
    subtype opcode_type is std_logic_vector(6 downto 0);
    
    -- Func3 extra function bits in instruction
    subtype func3_type is std_logic_vector(2 downto 0);

    -- Func7 extra function bits in instruction
    subtype func7_type is std_logic_vector(6 downto 0);
    
    -- Size of memory access
    type memsize_type is (memsize_unknown, memsize_byte, memsize_halfword, memsize_word);
    
    -- Memory access type
    type memaccess_type is (memaccess_nop, memaccess_write, memaccess_read);
    
    -- ALU operations
    type alu_op_type is (alu_unknown, alu_nop,
                         alu_add, alu_sub, alu_and, alu_or, alu_xor,
                         alu_slt, alu_sltu,
                         alu_addi, alu_andi, alu_ori, alu_xori,
                         alu_slti, alu_sltiu,
                         alu_sll, alu_srl, alu_sra,
                         alu_slli, alu_srli, alu_srai,
                         alu_lui, alu_auipc,
                         alu_lw, alu_lh, alu_lhu, alu_lb, alu_lbu,
                         alu_sw, alu_sh, alu_sb,
                         alu_jal_jalr,
                         alu_beq, alu_bne, alu_blt, alu_bge, alu_bltu, alu_bgeu,
                         alu_csr,
                         alu_multiply,
                         alu_divrem,
                         alu_trap, alu_mret
                        );
                        
    -- Control and State register operations
    type csr_op_type is (csr_nop, csr_rw, csr_rs, csr_rc, csr_rwi, csr_rsi, csr_rci);

    -- Traps
    type interrupt_request_type is (irq_none, irq_hard);
--    subtype interrupt_request_type is std_logic;
--    constant irq_none : interrupt_request_type := '0';
--    constant irq_hard : interrupt_request_type := '1';

    -- The ROM
    -- NOTE: the ROM is word (32 bits) size.
    -- NOTE: data is in Little Endian format (as by the toolchain)
    --       for half word and word entities
    --       Set rom_size_bits as if it were bytes
    --       default is 64 kB data
    constant rom_size_bits : integer := 16;
    constant rom_size : integer := 2**(rom_size_bits-2);
    type rom_type is array(0 to rom_size-1) of data_type;
    -- The contents of the ROM is loaded by processor_common_rom.vhd
    
    -- The bootloader ROM
    -- NOTE: the bootloader ROM is word (32 bits) size.
    -- NOTE: data is in Little Endian format (as by the toolchain)
    --       for half word and word entities
    --       Set bootloader rom_size_bits as if it were bytes
    --       default is 4 kB data
    constant bootloader_size_bits : integer := 12;
    constant bootloader_size : integer := 2**(bootloader_size_bits-2);
    type bootloader_type is array(0 to bootloader_size-1) of data_type;
    -- The contents of the bootloader ROM is loaded by bootloader.vhd
    
    -- The RAM
    -- NOTE: the RAM is 4x byte (8 bits) size, supporting
    --       32-bit Big Endian storage,
    --       so we have to recode to support Little Endian.
    --       Set ram_size_bits as if it were bytes
    -- Default is 32 kB data
    constant ram_size_bits : integer := 15;
    constant ram_size : integer := 2**(ram_size_bits-2);
    -- The type of the RAM block
    type ram_type is array (0 to ram_size-1) of std_logic_vector(7 downto 0);
                        
    -- The I/O
    -- NOTE: the I/O is word (32 bits) size, Big Endian
    --       there is no need to recode the data
    --       The I/O can only handle word size access
    --       Set io_size_bits as if it were bytes
    -- Default 128 bytes data
    constant io_size_bits : integer := 8;
    constant io_size : integer := 2**(io_size_bits-2);
    type io_type is array (0 to io_size-1) of data_type;
    
    -- The Control and Status Registers
    -- Keep csr_size_bits to 12!!!
    -- The CSRs have their own address space, it is not
    -- visible on the 4 GB normal address space.
    constant csr_size_bits : integer := 12;
    constant csr_size : integer := 2**csr_size_bits;
    subtype csraddr_type is std_logic_vector(csr_size_bits-1 downto 0);
    subtype csrimmrs1_type is std_logic_vector(4 downto 0);

    -- The highest nibble (4 bits) of the ROM, bootloader, RAM and I/O
    -- This will set the memories at 256 MB intervals
    constant rom_high_nibble : std_logic_vector(3 downto 0) := x"0";
    constant bootloader_high_nibble : std_logic_vector(3 downto 0) := x"1";
    constant ram_high_nibble : std_logic_vector(3 downto 0) := x"2";
    constant io_high_nibble : std_logic_vector(3 downto 0) := x"F";

end package processor_common;
