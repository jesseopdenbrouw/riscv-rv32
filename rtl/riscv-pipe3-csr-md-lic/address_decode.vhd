-- #################################################################################################
-- # address_decode.vhd - Address Decoder and Data Router between the core and the memory          #
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

-- This file contains the description of address decoder and
-- data router, it interconnects the core with memory (ROM, RAM
-- and I/O).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.processor_common.all;
use work.processor_common_rom.all;

entity address_decode is
    port (I_clk : in std_logic;
          I_areset : in std_logic;
          -- to core
          I_memaccess : in memaccess_type;
          I_memaddress : in data_type;
          O_waitfordata : out std_logic;
          O_dataout : out data_type; 
          -- to memory
          O_wrrom : out std_logic;
          O_wrram : out std_logic;
          O_wrio : out std_logic;
          O_csrom : out std_logic;
          O_csboot : out std_logic;
          O_csram : out std_logic;
          O_csio : out std_logic;
          I_romdatain : in data_type;
          I_bootdatain : in data_type;
          I_ramdatain : in data_type;
          I_iodatain : in data_type;
          O_load_access_error : out std_logic;
          O_store_access_error : out std_logic
         );
end entity address_decode;

architecture rtl of address_decode is
begin

    -- Address decoder and data router (may be forward from RS1)
    process (I_memaccess, I_memaddress, I_romdatain, I_bootdatain, I_ramdatain, I_iodatain) is
    begin
        
        O_wrrom <= '0';
        O_wrram <= '0';
        O_wrio <= '0';
        
        O_waitfordata <= '0';
        
        O_csrom <= '0';
        O_csboot <= '0';
        O_csram <= '0';
        O_csio <= '0';
        
        -- Currently not used. May be used to fault unimplemented memory.
        O_load_access_error <= '0';
        O_store_access_error <= '0';
        
        -- ROM @ 0xxxxxxx, 256M space, read-write
        if I_memaddress(31 downto 28) = rom_high_nibble then
            if I_memaccess = memaccess_read or I_memaccess = memaccess_write then
                O_csrom <= '1';
            end if;
            if I_memaccess = memaccess_write then
                O_wrrom <= '1';
            end if;
            if I_memaccess = memaccess_read then
                O_waitfordata <= '1';
            end if;
            O_dataout <= I_romdatain;
        -- Bootloader ROM @ 1xxxxxxx, 256M space, read only
        elsif I_memaddress(31 downto 28) = bootloader_high_nibble then
            if I_memaccess = memaccess_read then
                O_csboot <= '1';
                O_waitfordata <= '1';
            end if;
            O_dataout <= I_bootdatain;
        -- RAM @ 2xxxxxxx, 256M space
        elsif I_memaddress(31 downto 28) = ram_high_nibble then
            if I_memaccess = memaccess_read or I_memaccess = memaccess_write then
                O_csram <= '1';
            end if;
            if I_memaccess = memaccess_write then
                O_wrram <='1';
            end if;
            if I_memaccess = memaccess_read then
                O_waitfordata <= '1';
            end if;
            O_dataout <= I_ramdatain;
        -- I/O @ Fxxxxxxx, 256M space
        elsif I_memaddress(31 downto 28) = io_high_nibble then
            if I_memaccess = memaccess_read or I_memaccess = memaccess_write then
                O_csio <= '1';
            end if;
            if I_memaccess = memaccess_write then
                O_wrio <='1';
            end if;
            if I_memaccess = memaccess_read then
                O_waitfordata <= '1';
            end if;
            O_dataout <= I_iodatain;
        else
            if I_memaccess = memaccess_write then
                O_store_access_error <= '1';
            end if;
            if I_memaccess = memaccess_read then
                O_load_access_error <= '1';
            end if;
            O_dataout <= (others => 'X');
        end if;
    end process;

end architecture rtl;

