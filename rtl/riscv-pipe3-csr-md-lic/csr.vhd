-- #################################################################################################
-- # csr.vhd - The CSR                                                                             #
-- # ********************************************************************************************* #
-- # This file is part of the THUAS RISCV Minimal Project                                          #
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

-- Implementation of the Control and Status Registers.
-- The following registers are implemented:
--  cycle, cyclih -- copy of mcycle, mcycleh_addr
--  time, timeh -- copy of memory-mapped time, timeh I/O registers
--  instret, instreth -- copy of minstret, minstreth_addr
--  mstatus -- r/w
--  misa -- r/- (hardwired)
--  mie -- r/w, set all bits hard to 0 except MTIE (7), MSIE (3)
--  mtvec -- r/w
--  mcountinhibit -- r/w, only bits 0 and 2
--  mscratch -- r/w
--  mepc -- r/w
--  mcause -- r/w
--  mtval -- r/w
--  mip -- r/o
--  mvendorid -- r/o (hardwired to 0x00000000)
--  marchid -- r/o (hardwired to 0x00000000)
--  mimpid -- r/o (hardwired to 0x00000000)
--  mhartid -- r/o (hardwired to 0x00000000)
--  mconfigptr_addr -- r/o (hardwired to 0x00000000)


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.processor_common.all;

entity csr is
    port (I_clk : in std_logic;
          I_areset : in std_logic;
          -- Common signals for CSR instructions
          I_csr_op : in csr_op_type;
          I_csr_addr : in csraddr_type;
          I_csr_datain : in data_type;
          I_csr_immrs1 : in csrimmrs1_type;
          I_csr_instret : in std_logic;
          O_csr_dataout : out data_type;
          -- Exceptions/interrupts
          I_interrupt_request : in interrupt_request_type;
          I_interrupt_release : in std_logic;
          -- For use in mip
          I_intrio : in data_type;
          -- Global interrupt enable status
          O_mstatus_mie : out std_logic;
          -- mie.MTIE external timer interrupt enable
          O_mie_mtie : out std_logic;
          -- mcause reported by LIC
          I_mcause : in data_type;
          -- The trap vector
          O_mtvec : out data_type;
          -- The saved PC (not always what mepc tells us!)
          O_mepc : out data_type;
          -- PC to save in mepc
          I_pc : in data_type;
          -- Address on address bus, for mtval
          I_address : in data_type;
          -- TIME and TIMEH
          I_time : in data_type;
          I_timeh : in data_type
         );
end entity csr;

architecture rtl of csr is
signal csr : csr_type;
signal csr_addr_int : integer range 0 to 2**csr_size_bits-1;

-- Read only clock cycles, time in us and retired instructions, 
constant cycle_addr : integer := 16#c00#;
constant time_addr : integer := 16#c01#;
constant instret_addr : integer := 16#c02#;
constant cycleh_addr : integer := 16#c80#;
constant timeh_addr : integer := 16#c81#;
constant instreth_addr : integer := 16#c82#;

-- Read only
constant mvendorid_addr : integer := 16#f11#;
constant marchid_addr : integer := 16#f12#;
constant mimpid_addr : integer := 16#f13#;
constant mhartid_addr : integer := 16#f14#;
constant mconfigptr_addr : integer := 16#f15#;

-- Registers for interrupts/exceptions
constant mstatus_addr : integer := 16#300#; -- 768
-- misa should be read/write, but here it is read only
constant misa_addr : integer := 16#301#;
constant mie_addr : integer := 16#304#;
constant mtvec_addr : integer := 16#305#; -- 773
constant mcounteren_addr : integer := 16#306#; -- 774
constant mstatush_addr : integer := 16#310#;
constant mcountinhibit_addr : integer := 16#320#;
constant mscratch_addr : integer := 16#340#;
constant mepc_addr : integer := 16#341#; -- 833
constant mcause_addr : integer := 16#342#; -- 834
constant mtval_addr : integer := 16#343#;
constant mip_addr : integer := 16#344#;

constant mcycle_addr : integer := 16#b00#; --
constant minstret_addr : integer := 16#b02#; --
constant mcycleh_addr : integer := 16#b80#; --
constant minstreth_addr : integer := 16#b82#; --

begin

    -- Fetch CSR address
    csr_addr_int <= to_integer(unsigned(I_csr_addr));
    
    -- Output the pointed CSR
    process (csr_addr_int, csr) is
    begin
        case csr_addr_int is
            when cycle_addr => O_csr_dataout <= csr(cycle_addr);
            when time_addr => O_csr_dataout <= csr(time_addr);
            when instret_addr => O_csr_dataout <= csr(instret_addr);
            when cycleh_addr => O_csr_dataout <= csr(cycleh_addr);
            when timeh_addr => O_csr_dataout <= csr(timeh_addr);
            when instreth_addr => O_csr_dataout <= csr(instreth_addr);
            when mvendorid_addr => O_csr_dataout <= csr(mvendorid_addr);
            when marchid_addr => O_csr_dataout <= csr(marchid_addr);
            when mimpid_addr => O_csr_dataout <= csr(mimpid_addr);
            when mhartid_addr => O_csr_dataout <= csr(mhartid_addr);
            when mstatus_addr => O_csr_dataout <= csr(mstatus_addr);
            when mstatush_addr => O_csr_dataout <= csr(mstatush_addr);
            when misa_addr => O_csr_dataout <= csr(misa_addr);
            when mie_addr => O_csr_dataout <= csr(mie_addr);
            when mtvec_addr => O_csr_dataout <= csr(mtvec_addr);
            -- mcounteren is not implemented, because we only support M mode
            --when mcounteren_addr => O_csr_dataout <= csr(mcounteren_addr);
            when mcycle_addr => O_csr_dataout <= csr(cycle_addr);
            when minstret_addr => O_csr_dataout <= csr(instret_addr);
            when mcycleh_addr => O_csr_dataout <= csr(cycleh_addr);
            when minstreth_addr => O_csr_dataout <= csr(instreth_addr);
            when mcountinhibit_addr => O_csr_dataout <= csr(mcountinhibit_addr);
            when mscratch_addr => O_csr_dataout <= csr(mscratch_addr);
            when mepc_addr => O_csr_dataout <= csr(mepc_addr);
            when mcause_addr => O_csr_dataout <= csr(mcause_addr);
            when mtval_addr => O_csr_dataout <= csr(mtval_addr);
            when mip_addr => O_csr_dataout <= csr(mip_addr);
            when mconfigptr_addr => O_csr_dataout <= csr(mconfigptr_addr);
            when others => O_csr_dataout <= (others => '-');
        end case;
    end process;

    -- [M]CYCLE --- count the number of clock cycles
    -- These are read-only registers, should be writable
    process (I_clk, I_areset) is
    variable mcycle_reg : unsigned(63 downto 0);
    begin
        if I_areset = '1' then
            mcycle_reg := (others => '0');
        elsif rising_edge(I_clk) then
            --  Do we count cycles?
            if csr(mcountinhibit_addr)(0) = '0' then
                mcycle_reg := mcycle_reg + 1;
            end if;
        end if;
        csr(cycle_addr) <= std_logic_vector(mcycle_reg(31 downto 0));
        csr(cycleh_addr) <= std_logic_vector(mcycle_reg(63 downto 32));
    end process;
    
    -- TIME --- count the number of microseconds
    -- These are read-only registers
    -- They are shadowed from the MTIME/MTIMEH registers in the I/O
    csr(time_addr) <= I_time;
    csr(timeh_addr) <= I_timeh;
    
    -- INSTRET --- instructions retired
    -- These are read-only registers
    process (I_clk, I_areset) is
    variable minstret_reg : unsigned(63 downto 0);
    begin
        if I_areset = '1' then
            minstret_reg := (others => '0');
        elsif rising_edge(I_clk) then
            -- Instruction retired?
            if I_csr_instret = '1' then
                -- Do we count instructions retired?
                if csr(mcountinhibit_addr)(2) = '0' then
                    minstret_reg := minstret_reg + 1;
                end if;
            end if;
        end if;
        csr(instret_addr) <= std_logic_vector(minstret_reg(31 downto 0));
        csr(instreth_addr) <= std_logic_vector(minstret_reg(63 downto 32));
    end process;

    -- Data to process in other registers
    -- Ignore the misa, mip, these are hard wired
    process (I_clk, I_areset) is
    variable csr_content : data_type;
    begin
        if I_areset = '1' then
            -- Reset the lot
            csr(mstatus_addr) <= (others => '0');
            -- misa is hard wired
            csr(mie_addr) <= (others => '0');
            csr(mtvec_addr) <= (others => '0');
            -- mcounteren does not exists, because we have no U mode
            --csr(mcounteren_addr) <= (0 => '1', 1 => '1', 2 => '1', others => '0');
            csr(mcountinhibit_addr) <= (others => '0');
            csr(mscratch_addr) <= (others => '0');
            csr(mepc_addr) <= (others => '0');
            csr(mcause_addr) <= (others => '0');
            csr(mtval_addr) <= (others => '0');
            -- mip is hardcoded, read only
            --csr(mip_addr) <= (others => '0');
            -- mtval - trap value = address on address bus
            csr(mtval_addr) <= (others => '0');
        elsif rising_edge(I_clk) then
            -- If no interrupt is pending, then update the selected CSR.
            -- Needed because the instruction is restarted on MRET
            if I_interrupt_request = irq_none and I_csr_op /= csr_nop then
                -- Select the CSR
                case csr_addr_int is
                    when mstatus_addr => csr_content := csr(mstatus_addr);
                    -- misa is hardwired
                    --when misa_addr => csr_content := csr(misa_addr);
                    when mie_addr => csr_content := csr(mie_addr);
                    when mtvec_addr => csr_content := csr(mtvec_addr);
                    -- mcounteren not available since we have M mode only
                    --when mcounteren_addr => csr_content := csr(mcounteren_addr);
                    when mcountinhibit_addr => csr_content := csr(mcountinhibit_addr);
                    when mscratch_addr => csr_content := csr(mscratch_addr);
                    when mepc_addr => csr_content := csr(mepc_addr);
                    when mcause_addr => csr_content := csr(mcause_addr);
                    when mtval_addr => csr_content := csr(mtval_addr);
                    -- mip is hardcoded, read only
                    --when mip_addr => csr_content := csr(mip_addr);
                    when others => csr_content := (others => '-');
                end case;
                -- Do the operation
                -- Some bits should be ignored or hard wired to 0
                -- but we just ignore them
                case I_csr_op is
                    when csr_rw =>
                        csr_content := I_csr_datain;
                    when csr_rs =>
                        csr_content := csr_content or I_csr_datain;
                    when csr_rc =>
                        csr_content := csr_content and not I_csr_datain;
                    when csr_rwi =>
                        csr_content(31 downto 5) := (others => '0');
                        csr_content(4 downto 0) := I_csr_immrs1;
                    when csr_rsi =>
                        csr_content(4 downto 0) := csr_content(4 downto 0) or I_csr_immrs1(4 downto 0);
                    when csr_rci =>
                        csr_content(4 downto 0) := csr_content(4 downto 0) and not I_csr_immrs1(4 downto 0);
                    when others =>
                        null;
                end case;
                -- Write back
                case csr_addr_int is
                    when mstatus_addr => csr(mstatus_addr) <= csr_content;
                    -- misa is hardwired
                    --when misa_addr => csr(misa_addr) <= csr_content;
                    when mie_addr => csr(mie_addr) <= csr_content;
                    when mtvec_addr => csr(mtvec_addr) <= csr_content;
                    -- mcounteren does not exists
                    --when mcounteren_addr => csr(mcounteren_addr) <= csr_content;
                    when mcountinhibit_addr => csr(mcountinhibit_addr) <= csr_content;
                    when mscratch_addr => csr(mscratch_addr) <= csr_content;
                    when mepc_addr => csr(mepc_addr) <= csr_content;
                    when mcause_addr => csr(mcause_addr) <= csr_content;
                    when mtval_addr => csr(mtval_addr) <= csr_content;
                    -- mip is hardcoded, read only
                    --when mip_addr => csr(mip_addr) <= csr_content;
                    when others => null;
                end case;
            end if;
            
            -- Interrupt handling takes priority over possible user
            -- update of the CSRs.
            -- The LIC checks if exceptions/interrupts are enabled.
            if I_interrupt_request = irq_hard or I_interrupt_request = irq_soft then
                -- Copy mie to mpie
                csr(mstatus_addr)(7) <= csr(mstatus_addr)(3);
                -- Set M mode
                csr(mstatus_addr)(12 downto 11) <= "11";
                -- Disable interrupts
                csr(mstatus_addr)(3) <= '0';
                -- Copy mcause
                csr(mcause_addr) <= I_mcause;
                -- Save PC at the point of interrupt
                csr(mepc_addr) <= I_pc;
                -- The real PC to save, needed to (re)start an instruction
                --pc_to_save_int <= I_pc_to_save;
                -- Latch address from address bus
                csr(mtval_addr) <= I_address;
            elsif I_interrupt_release = '1' then
                -- Copy mpie to mie
                csr(mstatus_addr)(3) <= csr(mstatus_addr)(7);
                -- ??
                csr(mstatus_addr)(7) <= '1';
                -- Keep M mode
                csr(mstatus_addr)(12 downto 11) <= "11";
                -- mcause reset
                csr(mcause_addr) <= (others => '0');
                -- mepc reset
                csr(mepc_addr) <= (others => '0');
                -- mtval
                csr(mtval_addr) <= (others => '0');
            end if;

            -- Set all bits hard to 0 except MTIE (7), MSIE (3)
            csr(mie_addr)(31 downto 8) <= (others => '0');
            csr(mie_addr)(6 downto 4) <= (others => '0');
            csr(mie_addr)(2 downto 0) <= (others => '0');
            
            -- Set most bits of mstatus, and mstatush to 0
            csr(mstatus_addr)(31 downto 13) <= (others => '0');
            csr(mstatus_addr)(10 downto 8) <= (others => '0');
            csr(mstatus_addr)(4 downto 4) <= (others => '0');
            csr(mstatus_addr)(2 downto 0) <= (others => '0');
            
            -- Set most bits of mcountinhibit to 0
            csr(mcountinhibit_addr)(31 downto 3) <= (others => '0');
            -- TI bit always 0
            csr(mcountinhibit_addr)(1) <= '0';
            
            -- MCAUSE doesn't use that many bits...
            csr(mcause_addr)(30 downto 10) <= (others => '0');
        end if;
    end process;

    -- Present the saved PC. This is not necessarily the PC in the mtvec CSR.
    O_mepc <= std_logic_vector(unsigned(csr(mepc_addr))+4) when csr(mcause_addr) = x"0000000b" or csr(mcause_addr) = x"00000003" else std_logic_vector(unsigned(csr(mepc_addr)));
    
    -- Advertise the interrupt enable status
    O_mstatus_mie <= csr(mstatus_addr)(3);
    
    -- Advertise the M mode timer interrupt enable
    O_mie_mtie <= csr(mie_addr)(7);
    
    -- The interrupt/exception vector address
    -- Vectored mode only for interrupts. You need to
    -- create a jump table at the indicated mtvec address
    process (csr) is
    begin
        if VECTORED_MTVEC and csr(mtvec_addr)(0) = '1' and csr(mcause_addr)(31) = '1' then
            O_mtvec <= std_logic_vector(unsigned(csr(mtvec_addr)(31 downto 2)) + unsigned(csr(mcause_addr)(5 downto 0))) & "00";
        else
            O_mtvec <= csr(mtvec_addr)(31 downto 2) & "00";
        end if;
    end process;

    -- mip is hardwired to I/O interrupts and System Timer interrupt
    csr(mip_addr) <= I_intrio;

    -- Hard wired CSR's
    csr(mvendorid_addr) <= (others => '0'); --
    csr(marchid_addr) <= (others => '0');
    csr(mimpid_addr) <= (others => '0');
    csr(mhartid_addr) <= (others => '0');
    csr(misa_addr) <= x"40001100" when NUMBER_OF_REGISTERS = 32 else x"40001010";
    -- mstatush is hardcoded to all zero
    csr(mstatush_addr) <= (others => '0');
    csr(mconfigptr_addr) <= (others => '0');
    
end architecture rtl;
