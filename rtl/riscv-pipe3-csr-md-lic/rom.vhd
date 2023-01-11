-- #################################################################################################
-- # rom.vhd - The ROM                                                                             #
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

-- This file contains the description of the ROM. The ROM
-- is placed in mutable onboard RAM blocks and can be changed
-- by writing to it. A read takes two clock cycles, for both
-- instruction and data. The ROM contents is placed in file
-- processor_common_rom.vhd.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.processor_common.all;
use work.processor_common_rom.all;

entity rom is
    port (I_clk : in std_logic;
          I_areset : in std_logic;
          I_pc : in data_type;
          I_address : in data_type;
          I_csrom : in std_logic;
          I_wren : in std_logic;
          I_size : in memsize_type;
          I_stall : in std_logic;
          O_instr : out data_type;
          I_datain : in data_type;
          O_data_out : out data_type;
          --
          O_instruction_misaligned_error : out std_logic;
          O_load_misaligned_error : out std_logic;
          O_store_misaligned_error : out std_logic
         );
end entity rom;

architecture rtl of rom is

-- The ROM itself
signal rom : rom_type := rom_contents;

begin

    O_instruction_misaligned_error <= '0' when I_pc(1 downto 0) = "00" else '1';        

    -- ROM, for both instructions and read-write data
    process (I_clk, I_areset, I_pc, I_address, I_csrom, I_size, I_wren, I_datain) is
    variable address_instr : integer range 0 to rom_size-1;
    variable address_data : integer range 0 to rom_size-1;
    variable instr_var : data_type;
    variable instr_recode : data_type;
    variable romdata_var : data_type;
    constant x : std_logic_vector(7 downto 0) := (others => 'X');
    begin
        -- Calculate addresses
        address_instr := to_integer(unsigned(I_pc(rom_size_bits-1 downto 2)));
        address_data := to_integer(unsigned(I_address(rom_size_bits-1 downto 2)));
 
        -- Set store misaligned error
        if I_csrom = '1' and I_wren = '1' and I_size /= memsize_word then
            O_store_misaligned_error <= '1';
        else
            O_store_misaligned_error <= '0';
        end if;
        
        -- Quartus will detect ROM table and uses onboard RAM
        -- Do NOT use reset
        if rising_edge(I_clk) then
            -- Read the instruction
            if I_stall = '0' then
                instr_var := rom(address_instr);
            end if;
            -- Read the data
            romdata_var := rom(address_data);
            if HAVE_BOOTLOADER_ROM then
                -- Write the ROM ;-)
                if I_wren = '1' and I_size = memsize_word then
                    rom(address_data) <= I_datain(7 downto 0) & I_datain(15 downto 8) & I_datain(23 downto 16) & I_datain(31 downto 24);
                end if;
            end if;
        end if;

        -- Recode instruction
        O_instr <= instr_var(7 downto 0) & instr_var(15 downto 8) & instr_var(23 downto 16) & instr_var(31 downto 24);
        
        O_load_misaligned_error <= '0';
        
        -- By natural size, for data
        if I_csrom = '1' then
            if I_size = memsize_word and I_address(1 downto 0) = "00" then
                O_data_out <= romdata_var(7 downto 0) & romdata_var(15 downto 8) & romdata_var(23 downto 16) & romdata_var(31 downto 24);
            elsif I_size = memsize_halfword and I_address(1 downto 0) = "00" then
                O_data_out <= x & x & romdata_var(23 downto 16) & romdata_var(31 downto 24);
            elsif I_size = memsize_halfword and I_address(1 downto 0) = "10" then
                O_data_out <= x & x & romdata_var(7 downto 0) & romdata_var(15 downto 8);
            elsif I_size = memsize_byte then
                case I_address(1 downto 0) is
                    when "00" => O_data_out <= x & x & x & romdata_var(31 downto 24);
                    when "01" => O_data_out <= x & x & x & romdata_var(23 downto 16);
                    when "10" => O_data_out <= x & x & x & romdata_var(15 downto 8);
                    when "11" => O_data_out <= x & x & x & romdata_var(7 downto 0);
                    when others => O_data_out <= x & x & x & x; O_load_misaligned_error <= '1';
                end case;
            else
                -- Chip select, but not aligned
                O_data_out <= x & x & x & x;
                O_load_misaligned_error <= '1';
            end if;
        else
            -- No chip select, so no data
            O_data_out <= x & x & x & x;
        end if;
    end process;

end architecture rtl;
