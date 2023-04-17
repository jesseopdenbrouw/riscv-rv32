-- #################################################################################################
-- # lic.vhd - The Local Interrupt Controller                                                      #
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
-- # https:/github.com/jesseopdenbrouw/riscv-rv32                                                  #
-- #################################################################################################

-- This is the Local Interrupt Controller. Currently this is
-- an all combinational circuit. The LIC determines the 
-- current interrupt status based on priority. Local hardware I/O
-- interrupts have the highest priorities, the system timer has the
-- lowest hardware interrupt priority. Then the synchronous exceptions
-- (and ECALL/EBREAK) have priority. Interrupts will only occur if
-- mstatus.MIE is active.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.processor_common.all;

entity lic is
    port (I_clk: in std_logic;
          I_areset : in std_logic;
          -- mstatus.MIE Interrupt Enable bit
          I_mstatus_mie : in std_logic;
          -- mie.MTIE external timer interrupt enable
          I_mie_mtie : in std_logic;
          -- Max 16 external/hardware interrupts + System Timer
          I_intrio : in data_type;
          -- Synchronous exceptions
          I_ecall_request : in std_logic;
          I_ebreak_request : in std_logic;
          I_illegal_instruction_error_request : in std_logic;
          I_instruction_misaligned_error_request : in std_logic;
          I_load_access_error_request : in std_logic;
          I_store_access_error_request : in std_logic;
          I_load_misaligned_error_request : in std_logic;
          I_store_misaligned_error_request : in std_logic;
          -- MRET instruction detected
          I_mret_request : in std_logic;
          -- mcause value to CSR
          O_mcause : out data_type;
          -- Advertise interrupt request
          O_interrupt_request : out interrupt_request_type;
          -- Advertise interrupt release
          O_interrupt_release : out std_logic
         );
end entity;

architecture rtl of lic is
constant zeros : data_type := (others => '0');
begin

    -- mstatus.MIE enables only interrupts, exceptions are always enabled
    mstatus_mie_all: if MSTATUS_MIE_DISABLES_INTERRUPTS generate
        process (I_clk, I_areset, I_mstatus_mie, I_intrio, I_ecall_request,
                 I_ebreak_request, I_illegal_instruction_error_request,
                 I_instruction_misaligned_error_request, 
                 I_load_access_error_request,
                 I_store_access_error_request,
                 I_load_misaligned_error_request,
                 I_store_misaligned_error_request,
                 I_mret_request, I_mie_mtie) is
        variable interrupt_request_int : interrupt_request_type;
        begin
            interrupt_request_int := irq_none;
            O_interrupt_release <= '0';
            O_mcause <= (others => '0');
            
            -- Priority as of Table 3.7 of "Volume II: RISC-V Privileged Architectures V20211203"
            -- Local hardware interrupts take priority over exceptions, the RISC-V system timer
            -- has the lowest hardware interrupt priority. Not all exceptions are implemented
            -- SPI1 transmission complete interrupt
            if I_intrio(21) = '1' and I_mstatus_mie = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(21, O_mcause'length));
                O_mcause(31) <= '1';
            -- I2C1 transmit/receive complete interrupt
            elsif I_intrio(20) = '1' and I_mstatus_mie = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(20, O_mcause'length));
                O_mcause(31) <= '1';
            -- TIMER2 compare T/A/B/C interrupt
            elsif I_intrio(19) = '1' and I_mstatus_mie = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(19, O_mcause'length));
                O_mcause(31) <= '1';
            -- USART interrupt
            elsif I_intrio(18) = '1' and I_mstatus_mie = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(18, O_mcause'length));
                O_mcause(31) <= '1';
            -- TIMER1 compare T interrupt
            elsif I_intrio(17) = '1' and I_mstatus_mie = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(17, O_mcause'length));
                O_mcause(31) <= '1';
            -- For testing only, will be removed/changed
            elsif I_intrio(16) = '1' and I_mstatus_mie = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(16, O_mcause'length));
                O_mcause(31) <= '1';
            -- External timer interrupt
            elsif I_intrio(7) = '1' and I_mstatus_mie = '1' and I_mie_mtie = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(7, O_mcause'length));
                O_mcause(31) <= '1';
            -- Exceptions from here.
            elsif I_illegal_instruction_error_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(2, O_mcause'length));
            elsif I_instruction_misaligned_error_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(0, O_mcause'length));
            elsif I_ecall_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(11, O_mcause'length));
            elsif I_ebreak_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(3, O_mcause'length));
            elsif I_load_access_error_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(5, O_mcause'length));
            elsif I_store_access_error_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(7, O_mcause'length));
            elsif I_load_misaligned_error_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(4, O_mcause'length));
            elsif I_store_misaligned_error_request = '1' then
                interrupt_request_int := irq_hard;
                O_mcause <= std_logic_vector(to_unsigned(6, O_mcause'length));
            end if;
            
            O_interrupt_request <= interrupt_request_int;
            
            -- Signal interrupt release
            if I_mret_request = '1' then
                O_interrupt_release <= '1';
            end if;
        end process;
    end generate;

    -- mstatus.MIE enables all traps, so also exceptions
    mstatus_mie_intr: if not MSTATUS_MIE_DISABLES_INTERRUPTS generate
        process (I_clk, I_areset, I_mstatus_mie, I_intrio, I_ecall_request,
                 I_ebreak_request, I_illegal_instruction_error_request,
                 I_instruction_misaligned_error_request, 
                 I_load_access_error_request,
                 I_store_access_error_request,
                 I_load_misaligned_error_request,
                 I_store_misaligned_error_request,
                 I_mret_request, I_mie_mtie) is
        variable interrupt_request_int : interrupt_request_type;
        begin
            interrupt_request_int := irq_none;
            O_interrupt_release <= '0';
            O_mcause <= (others => '0');
            
            -- Not conform RISC-V: mstatus.MIE disables all traps
            if I_mstatus_mie = '1' then
                -- Priority as of Table 3.7 of "Volume II: RISC-V Privileged Architectures V20211203"
                -- Local hardware interrupts take priority over exceptions, the RISC-V system timer
                -- has the lowest hardware interrupt priority. Not all exceptions are implemented.
                -- SPI1 transmission complete interrupt
                if I_intrio(21) = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(21, O_mcause'length));
                    O_mcause(31) <= '1';
                -- I2C1 transmit/receive complete interrupt
                elsif I_intrio(20) = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(20, O_mcause'length));
                    O_mcause(31) <= '1';
                -- TIMER2 compare T/A/B/C interrupt
                elsif I_intrio(19) = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(19, O_mcause'length));
                    O_mcause(31) <= '1';
                -- USART interrupt
                elsif I_intrio(18) = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(18, O_mcause'length));
                    O_mcause(31) <= '1';
                -- TIMER1 interrupt
                elsif I_intrio(17) = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(17, O_mcause'length));
                    O_mcause(31) <= '1';
                -- For testing only, will be removed/changed
                elsif I_intrio(16) = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(16, O_mcause'length));
                    O_mcause(31) <= '1';
                -- External timer interrupt
                elsif I_intrio(7) = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(7, O_mcause'length));
                    O_mcause(31) <= '1';
                -- Exceptions from here.
                elsif I_illegal_instruction_error_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(2, O_mcause'length));
                elsif I_instruction_misaligned_error_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(0, O_mcause'length));
                elsif I_ecall_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(11, O_mcause'length));
                elsif I_ebreak_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(3, O_mcause'length));
                elsif I_load_access_error_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(5, O_mcause'length));
                elsif I_store_access_error_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(7, O_mcause'length));
                elsif I_load_misaligned_error_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(4, O_mcause'length));
                elsif I_store_misaligned_error_request = '1' then
                    interrupt_request_int := irq_hard;
                    O_mcause <= std_logic_vector(to_unsigned(6, O_mcause'length));
                end if;
            end if;    
            
            O_interrupt_request <= interrupt_request_int;
            
            -- Signal interrupt release
            if I_mret_request = '1' then
                O_interrupt_release <= '1';
            end if;
        end process;
    end generate;
    
end architecture rtl;