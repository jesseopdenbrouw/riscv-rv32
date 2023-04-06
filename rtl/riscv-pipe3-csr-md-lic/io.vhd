-- #################################################################################################
-- # io.vhd - The I/O                                                                              #
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

-- The I/O consists of:
-- A single 32 bits input register and a single 32 bit output
-- register. There is no data direction register. This may be
-- updated in the future.
-- One UART with 7/8/9 data bits, N/E/O parity and 1/2 stop
-- bits. Several UART flags are available: transmit complete,
-- receive complete, parity error, receive failed and framing
-- error. Interrupts on Transmit complete and Receive complete.
-- A simple timer TIMER1 is provided, has no prescaler and
-- generates an interrupt when the CMPT register is equal to
-- or greater than the TCNT register.
-- A more elaborate timer TIMER2 is provided with a 16-bit prescaler
-- PRSC and a 16-bit counter CMPT. Three PWM/OC outputs (A,B,C)
-- are provided. Interrupts available on T/A/B/C.
-- A minimal I2C master-only device capable of using Standard mode
-- (Sm) and Fast mode (Fm). START and STOP conditions generated
-- by software settings. Transmit complete interrupt available.
-- A SPI master device in available, with hardware active low
-- slave select, transmitting 8/16/24/32 bits in one transmission
-- and has interrupt capabilities.
-- A second simple SPI master is included without hardware slave
-- select and no interrupt capabilities.
-- The TIME and TIMECMP registers are provided and are memory
-- mapped and available as output. The TIME registers are currently
-- not writable. 

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.processor_common.all;

entity io is
    generic (freq_sys : integer := SYSTEM_FREQUENCY;
             freq_count : integer := CLOCK_FREQUENCY
            );
    port (I_clk : in std_logic;
          I_areset : in std_logic;
          I_csio : in std_logic;
          I_memaddress : in data_type;
          I_memsize : memsize_type;
          I_memvma : std_logic;
          I_wren : in std_logic;
          I_datain : in data_type;
          O_dataout : out data_type;
          O_load_misaligned_error : out std_logic;
          O_store_misaligned_error : out std_logic;
          -- Connection with outside world
          I_gpioapin : in data_type;
          O_gpioapout : out data_type;
          I_uart1rxd : in std_logic;
          O_uart1txd : out std_logic;
          IO_i2c1scl : inout std_logic;
          IO_i2c1sda : inout std_logic;
          O_spi1sck : out std_logic;
          O_spi1mosi : out std_logic;
          I_spi1miso : in std_logic;
          O_spi1nss : out std_logic;
          O_spi2sck : out std_logic;
          O_spi2mosi : out std_logic;
          I_spi2miso : in std_logic;
          O_timer2oct : out std_logic;
          O_timer2oca : out std_logic;
          O_timer2ocb : out std_logic;
          O_timer2occ : out std_logic;
          -- Hardware interrupt request
          O_intrio : out data_type;
          -- MTIME and MTIMEH
          O_mtime : out data_type;
          O_mtimeh : out data_type
         );
end entity io;
    
architecture rtl of io is

-- The I/O register file
signal io : io_type;
-- The I/O register number from address
signal reg_int : integer range 0 to io_size-1;
-- Boolean TRUE if the address is on word boundary
signal isword : boolean;
-- Read access granted
signal read_access_granted : std_logic;
-- Write access granted
signal write_access_granted : std_logic;


-- Port input and output
constant gpioapin_addr : integer := 0;   -- 0x00.b
constant gpioapout_addr : integer := 1;  -- 0x04.b
alias gpioapin_int : data_type is io(gpioapin_addr);
alias gpioapout_int : data_type is io(gpioapout_addr);
signal gpioapin_sync : data_type;

-- registers 2 - 7 not used -- reserved


-- UART1
constant uart1ctrl_addr : integer := 8;   -- 0x20.b
constant uart1stat_addr : integer := 9;   -- 0x24.b
constant uart1data_addr : integer := 10;  -- 0x28.b
constant uart1baud_addr : integer := 11;  -- 0x2c.b
alias uart1ctrl_int : data_type is io(uart1ctrl_addr);
alias uart1stat_int : data_type is io(uart1stat_addr);
alias uart1data_int : data_type is io(uart1data_addr);
alias uart1baud_int : data_type is io(uart1baud_addr);
-- Transmit signals
signal uart1txbuffer : std_logic_vector(11 downto 0);
signal uart1txstart : std_logic;
type uarttxstate_type is (tx_idle, tx_iter, tx_ready);
signal uart1txstate : uarttxstate_type;
signal uart1txbittimer : integer range 0 to 65535;
signal uart1txshiftcounter : integer range 0 to 15;
--Receive signals
signal uart1rxbuffer : std_logic_vector(11 downto 0);
type uartrxstate_type is (rx_idle, rx_wait, rx_iter, rx_parity, rx_parity2, rx_ready, rx_fail);
signal uart1rxstate : uartrxstate_type;
signal uart1rxbittimer : integer range 0 to 65535;
signal uart1rxshiftcounter : integer range 0 to 15;
signal uart1rxd_sync : std_logic;

-- Register 12 - 15 not used, reserved


-- I2C1 - minimalistic I2C master-only
constant i2c1ctrl_addr : integer := 16;   -- 0x40.b
constant i2c1stat_addr : integer := 17;   -- 0x44.b
constant i2c1data_addr : integer := 18;   -- 0x48.b
alias i2c1ctrl_int : data_type is io(i2c1ctrl_addr);
alias i2c1stat_int : data_type is io(i2c1stat_addr);
alias i2c1data_int : data_type is io(i2c1data_addr);
signal i2c1txbuffer : std_logic_vector(8 downto 0);
signal i2c1rxbuffer : std_logic_vector(8 downto 0);
type i2c1state_type is (idle, send_startbit, send_data_first, send_data_second, leadout,
                        send_stopbit_first, send_stopbit_second, send_stopbit_third);
signal i2c1state : i2c1state_type;
signal i2c1bittimer : integer range 0 to 65535;
signal i2c1shiftcounter : integer range 0 to 9;
signal i2c1startstransmission : std_logic;
signal i2c1sda_out : std_logic;
signal i2c1scl_out : std_logic;
signal i2c1sdasync : std_logic_vector(1 downto 0);
signal i2c1sclsync : std_logic_vector(1 downto 0);
alias i2c1mack : std_logic is i2c1ctrl_int(11);
alias i2c1startbit : std_logic is i2c1ctrl_int(9);
alias i2c1stopbit : std_logic is i2c1ctrl_int(8);
alias i2c1fastmode : std_logic is i2c1ctrl_int(2);
alias i2c1softreset : std_logic is i2c1ctrl_int(1);
alias i2c1istransmitting : std_logic is i2c1stat_int(2);
alias i2c1tc : std_logic is i2c1stat_int(3); 
alias i2c1ackfail : std_logic is i2c1stat_int(5);
alias i2c1busy : std_logic is i2c1stat_int(6);

-- Register 19 - 23 not used, reserved


-- SPI1 - full SPI master with hardware NSS
constant spi1ctrl_addr : integer := 24; -- 0x60.b
constant spi1stat_addr : integer := 25; -- 0x64.b
constant spi1data_addr : integer := 26; -- ox68.b
-- Register 27 not used
alias spi1ctrl_int : data_type is io(spi1ctrl_addr);
alias spi1stat_int : data_type is io(spi1stat_addr);
alias spi1data_int : data_type is io(spi1data_addr);
signal spi1start : std_logic;
type spistate_type is (idle, cssetup, first, second, leadout, cshold);
signal spi1state : spistate_type;
signal spi1txbuffer : data_type;
signal spi1bittimer : integer range 0 to 127;
signal spi1shiftcounter : integer range 0 to 32;
signal spi1mosi : std_logic;
signal spi1rxbuffer : data_type;
signal spi1sck : std_logic;
constant spi1mosidefault : std_logic := 'Z';

-- Register 27 not used - reserved


-- SPI2 - simple SPI master, software NSS
constant spi2ctrl_addr : integer := 28; -- 0x70.b
constant spi2stat_addr : integer := 29; -- 0x74.b
constant spi2data_addr : integer := 30; -- ox78.b
-- Register 31 not used
alias spi2ctrl_int : data_type is io(spi2ctrl_addr);
alias spi2stat_int : data_type is io(spi2stat_addr);
alias spi2data_int : data_type is io(spi2data_addr);
signal spi2start : std_logic;
type spi2state_type is (idle, first, second, leadout);
signal spi2state : spi2state_type;
signal spi2txbuffer : data_type;
signal spi2bittimer : integer range 0 to 127;
signal spi2shiftcounter : integer range 0 to 32;
signal spi2mosi : std_logic;
signal spi2rxbuffer : data_type;
signal spi2sck : std_logic;
constant spi2mosidefault : std_logic := '1';

-- Register 31 not used - reserved


-- Timer/Counters
-- TIMER1
constant timer1ctrl_addr : integer := 32; -- 0x80.b
constant timer1stat_addr : integer := 33; -- 0x84.b
constant timer1cntr_addr : integer := 34; -- 0x88.b
constant timer1cmpt_addr : integer := 35; -- 0x8c.b
alias timer1ctrl_int : data_type is io(timer1ctrl_addr);
alias timer1stat_int : data_type is io(timer1stat_addr);
alias timer1cntr_int : data_type is io(timer1cntr_addr);
alias timer1cmpt_int : data_type is io(timer1cmpt_addr);
-- registers 36 - 39 not used -- reserved


-- TIMER2
constant timer2ctrl_addr : integer := 40; -- 0xa0.b
constant timer2stat_addr : integer := 41; -- 0xa4.b
constant timer2cntr_addr : integer := 42; -- 0xa8.b
constant timer2cmpt_addr : integer := 43; -- 0xac.b
constant timer2prsc_addr : integer := 44; -- 0xb0.b
constant timer2cmpa_addr : integer := 45; -- 0xb4.b
constant timer2cmpb_addr : integer := 46; -- 0xb8.b
constant timer2cmpc_addr : integer := 47; -- 0xbc.b
alias timer2ctrl_int : data_type is io(timer2ctrl_addr);
alias timer2stat_int : data_type is io(timer2stat_addr);
alias timer2cntr_int : data_type is io(timer2cntr_addr);
alias timer2cmpt_int : data_type is io(timer2cmpt_addr);
alias timer2prsc_int : data_type is io(timer2prsc_addr);
alias timer2cmpa_int : data_type is io(timer2cmpa_addr);
alias timer2cmpb_int : data_type is io(timer2cmpb_addr);
alias timer2cmpc_int : data_type is io(timer2cmpc_addr);
-- internal prescaler
signal timer2prescaler_int : data_type;
-- shadow registers
signal timer2cmptshadow_int : data_type;
signal timer2cmpashadow_int : data_type;
signal timer2cmpbshadow_int : data_type;
signal timer2cmpcshadow_int : data_type;
signal timer2prscshadow_int : data_type;
signal timer2oct_int : std_logic;
signal timer2oca_int : std_logic;
signal timer2ocb_int : std_logic;
signal timer2occ_int : std_logic;

-- Registers 50 - 59 not used - reserved


-- RISC-V system timer TIME and TIMECMP
constant mtime_addr : integer := 60;      -- 0xf0.b
constant mtimeh_addr : integer := 61;     -- 0xf4.b
constant mtimecmp_addr : integer := 62;   -- 0xf8.b
constant mtimecmph_addr : integer := 63;  -- 0xfc.b
alias mtime_int : data_type is io(mtime_addr);
alias mtimeh_int : data_type is io(mtimeh_addr);
alias mtimecmp_int : data_type is io(mtimecmp_addr);
alias mtimecmph_int : data_type is io(mtimecmph_addr);


begin

    -- Fetch internal register of io_size_bits bits minus 2
    -- because we will use word size only
    reg_int <= to_integer(unsigned(I_memaddress(io_size_bits-1 downto 2)));
    
    -- Check if an access is on a 4-byte boundary AND is word size
    isword <= TRUE when I_memsize = memsize_word and I_memaddress(1 downto 0) = "00" else FALSE;

    -- Misaligned error, when (not on a 4-byte boundary OR not word size) AND chip select
    O_load_misaligned_error <= '1' when isword = FALSE and I_csio = '1' and I_wren = '0' else '0';
    O_store_misaligned_error <= '1' when isword = FALSE and I_csio = '1' and I_wren = '1' else '0';
    
    -- Read or write access, but only if no interrupt is pending
    read_access_granted <= '1' when isword and I_csio = '1' and I_wren = '0' and I_memvma = '1' else '0';
    write_access_granted <= '1' when isword and I_csio = '1' and I_wren = '1' and I_memvma = '1' else '0';
    
    --
    -- Data out to ALU
    --
    process (I_clk, I_areset) is --, io, isword, reg_int, I_csio, I_wren) is
    begin
        -- Only at word boundaries AND chip select
        if I_areset = '1' then
            O_dataout <= (others => '0');
        elsif rising_edge(I_clk) then
            if read_access_granted = '1' then
                case reg_int is
                    when gpioapin_addr   => O_dataout <= gpioapin_int;
                    when gpioapout_addr  => O_dataout <= gpioapout_int;
                    when uart1ctrl_addr  => O_dataout <= uart1ctrl_int;
                    when uart1stat_addr  => O_dataout <= uart1stat_int;
                    when uart1data_addr  => O_dataout <= uart1data_int;
                    when uart1baud_addr  => O_dataout <= uart1baud_int;
                    when i2c1ctrl_addr   => O_dataout <= i2c1ctrl_int;
                    when i2c1stat_addr   => O_dataout <= i2c1stat_int;
                    when i2c1data_addr   => O_dataout <= i2c1data_int;
                    when spi1ctrl_addr   => O_dataout <= spi1ctrl_int;
                    when spi1stat_addr   => O_dataout <= spi1stat_int;
                    when spi1data_addr   => O_dataout <= spi1data_int;
                    when spi2ctrl_addr   => O_dataout <= spi2ctrl_int;
                    when spi2stat_addr   => O_dataout <= spi2stat_int;
                    when spi2data_addr   => O_dataout <= spi2data_int;
                    when timer1ctrl_addr => O_dataout <= timer1ctrl_int;
                    when timer1stat_addr => O_dataout <= timer1stat_int;
                    when timer1cntr_addr => O_dataout <= timer1cntr_int;
                    when timer1cmpt_addr => O_dataout <= timer1cmpt_int;
                    when timer2ctrl_addr => O_dataout <= timer2ctrl_int;
                    when timer2stat_addr => O_dataout <= timer2stat_int;
                    when timer2cntr_addr => O_dataout <= timer2cntr_int;
                    when timer2cmpt_addr => O_dataout <= timer2cmpt_int;
                    when timer2prsc_addr => O_dataout <= timer2prsc_int;
                    when timer2cmpa_addr => O_dataout <= timer2cmpa_int;
                    when timer2cmpb_addr => O_dataout <= timer2cmpb_int;
                    when timer2cmpc_addr => O_dataout <= timer2cmpc_int;
                    when mtime_addr      => O_dataout <= mtime_int;
                    when mtimeh_addr     => O_dataout <= mtimeh_int;
                    when mtimecmp_addr   => O_dataout <= mtimecmp_int;
                    when mtimecmph_addr  => O_dataout <= mtimecmph_int;
                    when others => O_dataout <= (others => '-');
                end case;
            end if;
        end if;
    end process;

    --
    -- GPIO A pin en pout    
    -- General purpose I/O ports, one 32-bit input and one 32-bit output
    --
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            gpioapin_int <= (others => '0');
            gpioapout_int <= (others => '0');
            gpioapin_sync <= (others => '0');
        elsif rising_edge(I_clk) then
            -- Read data in from outside world
            -- Synchronizer register
            gpioapin_sync <= I_gpioapin; 
            gpioapin_int <= gpioapin_sync;
            -- Only write to I/O when write is enabled AND size is word
            -- Only write to the outputs, not the inputs
            -- Only write if on 4-byte boundary
            -- Only write when Chip Select (cs)
            if write_access_granted = '1' then
                if reg_int = gpioapout_addr then
                    gpioapout_int <= I_datain;
                end if;
            end if;
        end if;
    end process;
     -- Data to outside world
    O_gpioapout <= gpioapout_int;
    
    
    --
    -- UART1
    --
    uart1gen: if HAVE_UART1 generate
        process (I_clk, I_areset) is
        variable uart1txshiftcounter_var : integer range 0 to 15;
        begin
            -- Common resets et al.
            if I_areset = '1' then
                uart1data_int <= (others => '0');
                uart1baud_int <= (others => '0');
                uart1ctrl_int <= (others => '0');
                uart1stat_int <= (others => '0');
                uart1txstart <= '0';
                uart1txstate <= tx_idle;
                uart1txbuffer <= (others => '0');
                uart1txbittimer <= 0;
                uart1txshiftcounter <= 0;
                O_uart1txd <= '1';
                uart1rxbuffer <= (others => '0');
                uart1rxstate <= rx_idle;
                uart1rxbittimer <= 0;
                uart1rxshiftcounter <= 0;
                uart1rxd_sync <= '1';
            elsif rising_edge(I_clk) then
                -- Default for start transmission
                uart1txstart <= '0';
                -- Common register writes
                if write_access_granted = '1' then
                    if reg_int = uart1ctrl_addr then
                        -- A write to the control register
                        uart1ctrl_int <= I_datain;
                    elsif reg_int = uart1stat_addr then
                        -- A write to the status register
                        uart1stat_int <= I_datain;
                    elsif reg_int = uart1baud_addr then
                        -- A write to the baud rate register
                        -- Use only 16 bits for baud rate
                        uart1baud_int <= I_datain;
                    elsif reg_int = uart1data_addr then
                        -- A write to the data register triggers a transmission
                        -- Signal start
                        uart1txstart <= '1';
                        -- Load transmit buffer with 7/8/9 data bits, parity bit and
                        -- a start bit
                        -- Stop bits will be automatically added since the remaining
                        -- bits are set to 1. Most right bit is start bit.
                        uart1txbuffer <= (others => '1');
                        if uart1ctrl_int(3 downto 2) = "10" then
                            -- 9 bits data
                            uart1txbuffer(9 downto 0) <= I_datain(8 downto 0) & '0';
                            -- Have parity
                            if uart1ctrl_int(5) = '1' then
                                uart1txbuffer(10) <= I_datain(8) xor I_datain(7) xor I_datain(6) xor I_datain(5) xor I_datain(4)
                                                xor I_datain(3) xor I_datain(2) xor I_datain(1) xor I_datain(0) xor uart1ctrl_int(4);
                            end if;
                        elsif uart1ctrl_int(3 downto 2) = "11" then
                            -- 7 bits data
                            uart1txbuffer(7 downto 0) <= I_datain(6 downto 0) & '0';
                            -- Have parity
                            if uart1ctrl_int(5) = '1' then
                                uart1txbuffer(8) <= I_datain(6) xor I_datain(5) xor I_datain(4) xor I_datain(3)
                                             xor I_datain(2) xor I_datain(1) xor I_datain(0) xor uart1ctrl_int(4);
                            end if;
                        else
                            -- 8 bits data
                            uart1txbuffer(8 downto 0) <= I_datain(7 downto 0) & '0';
                            -- Have parity
                            if uart1ctrl_int(5) = '1' then
                                uart1txbuffer(9) <= I_datain(7) xor I_datain(6) xor I_datain(5) xor I_datain(4) xor I_datain(3)
                                             xor I_datain(2) xor I_datain(1) xor I_datain(0) xor uart1ctrl_int(4);
                            end if;
                        end if;
                        -- Signal that we are sending
                        uart1stat_int(4) <= '0'; 
                    end if;
                end if;
                
                -- If data register is read...
                if read_access_granted = '1' then
                    if reg_int = uart1data_addr then
                        -- Clear the received status bits
                        -- PE, RC, RF, FE
                        uart1stat_int(3) <= '0';
                        uart1stat_int(2) <= '0';
                        uart1stat_int(1) <= '0';
                        uart1stat_int(0) <= '0';
                    end if;
                end if;
                
                -- Transmit a character
                case uart1txstate is
                    -- Tx idle state, wait for start
                    when tx_idle =>
                        O_uart1txd <= '1';
                        -- If start triggered...
                        if uart1txstart = '1' then
                            -- Load the prescaler, set the number of bits (including start bit)
                            uart1txbittimer <= to_integer(unsigned(uart1baud_int));
                            if uart1ctrl_int(3 downto 2) = "10" then
                                uart1txshiftcounter_var := 10;
                            elsif uart1ctrl_int(3 downto 2) = "11" then
                                uart1txshiftcounter_var := 8;
                            else
                                uart1txshiftcounter_var := 9;
                            end if;
                            -- Add up posibly parity bit and posibly second stop bit
                            uart1txshiftcounter <= uart1txshiftcounter_var + to_integer(unsigned(uart1ctrl_int(5 downto 5))) + to_integer(unsigned(uart1ctrl_int(0 downto 0)));
                            uart1txstate <= tx_iter;
                        else
                            uart1txstate <= tx_idle;
                        end if;
                    -- Transmit the bits
                    when tx_iter =>
                        -- Cycle through all bits in the transmit buffer
                        -- First in line is the start bit
                        O_uart1txd <= uart1txbuffer(0);
                        if uart1txbittimer > 0 then
                            uart1txbittimer <= uart1txbittimer - 1;
                        elsif uart1txshiftcounter > 0 then
                            uart1txbittimer <= to_integer(unsigned(uart1baud_int));
                            uart1txshiftcounter <= uart1txshiftcounter - 1;
                            -- Shift in stop bit
                            uart1txbuffer <= '1' & uart1txbuffer(uart1txbuffer'high downto 1);
                        else
                            uart1txstate <= tx_ready;
                        end if;
                    -- Signal ready
                    when tx_ready =>
                        O_uart1txd <= '1';
                        uart1txstate <= tx_idle;
                        -- Signal character transmitted
                        uart1stat_int(4) <= '1'; 
                    when others =>
                        O_uart1txd <= '1';
                        uart1txstate <= tx_idle;
                end case;
                
                -- Receive character
                -- Input synchronizer
                uart1rxd_sync <= I_uart1rxd;
                case uart1rxstate is
                    -- Rx idle, wait for start bit
                    when rx_idle =>
                        -- If detected a start bit ...
                        if uart1rxd_sync = '0' then
                            -- Set half bit time ...
                            uart1rxbittimer <= to_integer(unsigned(uart1baud_int))/2;
                            uart1rxstate <= rx_wait;
                        else
                            uart1rxstate <= rx_idle;
                        end if;
                    -- Hunt for start bit, check start bit at half bit time
                    when rx_wait =>
                        if uart1rxbittimer > 0 then
                            uart1rxbittimer <= uart1rxbittimer - 1;
                        else
                            -- At half bit time...
                            -- Start bit is still 0, so continue
                            if uart1rxd_sync = '0' then
                                uart1rxbittimer <= to_integer(unsigned(uart1baud_int));
                                -- Set reception size
                                if uart1ctrl_int(3 downto 2) = "10" then
                                    -- 9 bits
                                    uart1rxshiftcounter <= 9;
                                elsif uart1ctrl_int(3 downto 2) = "11" then
                                    -- 7 bits
                                    uart1rxshiftcounter <= 7;
                                else
                                    -- 8 bits
                                    uart1rxshiftcounter <= 8;
                                end if;
                                uart1rxbuffer <= (others => '0');
                                uart1rxstate <= rx_iter;
                            else
                                -- Start bit is not 0, so invalid transmission
                                uart1rxstate <= rx_fail;
                            end if;
                        end if;
                    -- Shift in the data bits
                    -- We sample in the middle of a bit time...
                    when rx_iter =>
                        if uart1rxbittimer > 0 then
                            -- Bit timer not finished, so keep counting...
                            uart1rxbittimer <= uart1rxbittimer - 1;
                        elsif uart1rxshiftcounter > 0 then
                            -- Bit counter not finished, so restart timer and shift in data bit
                            uart1rxbittimer <= to_integer(unsigned(uart1baud_int));
                            uart1rxshiftcounter <= uart1rxshiftcounter - 1;
                            if uart1ctrl_int(3 downto 2) = "10" then
                                -- 9 bits
                                uart1rxbuffer(8 downto 0) <= uart1rxd_sync & uart1rxbuffer(8 downto 1);
                            elsif uart1ctrl_int(3 downto 2) = "11" then
                                -- 7 bits
                                uart1rxbuffer(6 downto 0) <= uart1rxd_sync & uart1rxbuffer(6 downto 1);
                            else
                                -- 8 bits
                                uart1rxbuffer(7 downto 0) <= uart1rxd_sync & uart1rxbuffer(7 downto 1);
                            end if;
                        else
                            -- Do we have a parity bit?
                            if uart1ctrl_int(5) = '1' then
                                uart1rxstate <= rx_parity;
                            else
                                uart1rxstate <= rx_ready;
                            end if;
                        end if;
                    -- Check parity, we already there...
                    when rx_parity =>
                        if uart1ctrl_int(3 downto 2) = "10" then
                            uart1stat_int(3) <= uart1rxbuffer(8) xor uart1rxbuffer(7) xor uart1rxbuffer(6) xor uart1rxbuffer(5)
                                                xor uart1rxbuffer(4) xor uart1rxbuffer(3) xor uart1rxbuffer(2)
                                                xor uart1rxbuffer(1) xor uart1rxbuffer(0) xor uart1rxd_sync xor uart1ctrl_int(4);
                        elsif uart1ctrl_int(3 downto 2) = "11" then
                            uart1stat_int(3) <= uart1rxbuffer(6) xor uart1rxbuffer(5)
                                                xor uart1rxbuffer(4) xor uart1rxbuffer(3) xor uart1rxbuffer(2)
                                                xor uart1rxbuffer(1) xor uart1rxbuffer(0) xor uart1rxd_sync xor uart1ctrl_int(4);
                        else
                            uart1stat_int(3) <= uart1rxbuffer(7) xor uart1rxbuffer(6) xor uart1rxbuffer(5)
                                                xor uart1rxbuffer(4) xor uart1rxbuffer(3) xor uart1rxbuffer(2)
                                                xor uart1rxbuffer(1) xor uart1rxbuffer(0) xor uart1rxd_sync xor uart1ctrl_int(4);
                        end if;
                        uart1rxbittimer <= to_integer(unsigned(uart1baud_int));
                        uart1rxstate <= rx_parity2;
                    -- Wait to middle of stop bit
                    when rx_parity2 =>
                        if uart1rxbittimer > 0 then
                            uart1rxbittimer <= uart1rxbittimer - 1;
                        else
                            uart1rxstate <= rx_ready;
                        end if;
                    -- When ready, all bits are shifted in
                    -- Even if we use two stop bits, we only check one and
                    -- signal reception. This leave some computation time
                    -- before the next reception occurs.
                    when rx_ready =>
                        -- Test for a stray 0 in position of (first) stop bit
                        if uart1rxd_sync = '0' then
                            -- Signal frame error
                            uart1stat_int(0) <= '1';
                        end if;
                        -- Any way, copy the received data to the data register
                        uart1data_int <= (others => '0');
                        if uart1ctrl_int(3 downto 2) = "10" then
                            -- 9 bits
                            uart1data_int(8 downto 0) <= uart1rxbuffer(8 downto 0);
                        elsif uart1ctrl_int(3 downto 2) = "11" then
                            -- 7 bits
                            uart1data_int(6 downto 0) <= uart1rxbuffer(6 downto 0);
                        else
                            -- 8 bits
                            uart1data_int(7 downto 0) <= uart1rxbuffer(7 downto 0);
                        end if;
                        -- signal reception
                        uart1stat_int(2) <= '1';
                        uart1rxstate <= rx_idle;
                    -- Wrong start bit detected, no data present
                    when rx_fail =>
                        -- Failed to receive a correct start bit...
                        uart1rxstate <= rx_idle;
                        uart1stat_int(1) <= '1';
                    when others =>
                        uart1rxstate <= rx_idle;
                end case;
                uart1baud_int(31 downto 16) <= (others => '0');
                uart1data_int(31 downto 9) <= (others => '0');
                uart1ctrl_int(31 downto 8) <= (others => '0');
                uart1stat_int(31 downto 5) <= (others => '0');
            end if;
        end process;
    end generate;
    uart1gen_not: if not HAVE_UART1 generate
        uart1baud_int <= (others => '0');
        uart1data_int <= (others => '0');
        uart1ctrl_int <= (others => '0');
        uart1stat_int <= (others => '0');
        O_uart1txd <= 'Z';
    end generate;

    --
    -- I2C1
    --
    i2c1gen : if HAVE_I2C1 generate
        process (I_clk, I_areset) is
        begin
            if I_areset = '1' then
                i2c1ctrl_int <= (others => '0');
                i2c1stat_int <= (others => '0');
                i2c1data_int <= (others => '0');
                i2c1scl_out <= '1';
                i2c1sda_out <= '1';
                i2c1state <= idle;
                i2c1bittimer <= 0;
                i2c1shiftcounter <= 0;
                i2c1txbuffer <= (others => '0');
                i2c1rxbuffer <= (others => '0');
                i2c1startstransmission <= '0';
                i2c1sclsync <= (others => '1');
                i2c1sdasync <= (others => '1');
            elsif rising_edge(I_clk) then
                i2c1startstransmission <= '0';
                -- Common register writes
                if write_access_granted = '1' then
                    if reg_int = i2c1ctrl_addr then
                        i2c1ctrl_int <= I_datain;
                    elsif reg_int = i2c1stat_addr then
                        i2c1stat_int <= I_datain;
                    elsif reg_int = i2c1data_addr then
                        -- Latch data, if startbit set, end with master Nack
                        i2c1txbuffer <= I_datain(7 downto 0) & (i2c1startbit or i2c1stopbit or not i2c1mack);
                        -- Signal that we are sending data
                        i2c1startstransmission <= '1';
                        -- Reset both Transmission Complete and Ack Failed
                        i2c1tc <= '0';
                        i2c1ackfail <= '0';
                    end if;
                end if;
                -- If read data register, clear the TC and AF flag
                if read_access_granted = '1' then
                    if reg_int = i2c1data_addr then
                        i2c1tc <= '0';
                        i2c1ackfail <= '0';
                    end if;
                end if;

                -- Check for I2C bus is busy.
                -- If SCL or SDA is/are low...
                if i2c1sclsync(1) = '0' or i2c1sdasync(1) = '0' then
                    -- I2C bus is busy
                    i2c1busy <= '1';
                end if;
                -- SCL is high and rising edge on SDA...
                if i2c1sclsync(0) /= '0' and i2c1sdasync(1) = '0' and i2c1sdasync(0) /= '0' then
                    -- signals a STOP, so bus is free
                    i2c1busy <= '0';
                end if;
                
                -- Input synchronizer
                i2c1sdasync <= i2c1sdasync(0) & IO_i2c1sda;
                i2c1sclsync <= i2c1sclsync(0) & IO_i2c1scl;

                -- The I2C1 state machine
                case i2c1state is
                    when idle =>
                        -- Clock == !state_of_transmitting, SDA = High-Z (==1)
                        -- If transmitting, the clock is held low. If not
                        -- transmitting, the clock is held high (high-Z). After
                        -- STOP, the state of transmitting is reset. This keeps
                        -- the bus occupied between START and STOP.
                        i2c1scl_out <= not i2c1istransmitting;
                        i2c1sda_out <= '1';
                        -- Idle situation, load the counters and set SCL/SDA to High-Z
                        if i2c1fastmode = '1' then
                            i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)))*2;
                        else
                            i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)));
                        end if;
                        i2c1shiftcounter <= 8;
                        -- Is data register written?
                        if i2c1startstransmission = '1' then
                            -- Register that we are transmitting
                            i2c1istransmitting <= '1';
                            -- Data written to data register, check for start condition
                            if i2c1startbit = '1' then
                                -- Start bit is seen, so clear it.
                                i2c1startbit <= '0';
                                -- Send a START bit, so address comes next
                                i2c1state <= send_startbit;
                            else
                                -- Regular data
                                i2c1state <= send_data_first;
                            end if;
                        end if;
                    when send_startbit =>
                        -- Generate start condition
                        i2c1scl_out <= '1';
                        i2c1sda_out <= '0';
                        if i2c1bittimer > 0 then
                            i2c1bittimer <= i2c1bittimer - 1;
                        else
                            if i2c1fastmode = '1' then
                                i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)))*2;
                            else
                                i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)));
                            end if;
                            i2c1state <= send_data_first;
                        end if;
                    when send_data_first =>
                        -- SCL low == 0, SDA 0 or High-Z (== 1)
                        i2c1scl_out <= '0';
                        i2c1sda_out <= i2c1txbuffer(8);
                        
                        -- Count bit time
                        if i2c1bittimer > 0 then
                            i2c1bittimer <= i2c1bittimer - 1;
                        else
                            i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)));
                            i2c1state <= send_data_second;
                        end if;
                    when send_data_second =>
                        -- SCL High-Z == 1, SDA 0 or High-Z (== 1)
                        i2c1scl_out <= '1';
                        i2c1sda_out <= i2c1txbuffer(8);

                        -- Count bit time
                        if i2c1bittimer > 0 then
                            i2c1bittimer <= i2c1bittimer - 1;
                        else
                            if i2c1fastmode = '1' then
                                i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)))*2;
                            else
                                i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)));
                            end if;
                            -- Check if more bits
                            if i2c1shiftcounter > 0 then
                                -- More bits to send...
                                i2c1shiftcounter <= i2c1shiftcounter - 1;
                                i2c1state <= send_data_first;
                                -- Shift next bit, hold time is 0 ns as per spec
                                i2c1txbuffer <= i2c1txbuffer(7 downto 0) & '1';
                            else
                                -- No more bits, then goto STOP or leadout
                                if i2c1stopbit = '1' then
                                    i2c1state <= send_stopbit_first;
                                else
                                    i2c1state <= leadout;
                                end if;
                            end if;
                        end if;
                        -- If detected rising edge on external SCL, clock in SDA.
                        if i2c1sclsync(1) = '0' and i2c1sclsync(0) /= '0' then
                            i2c1rxbuffer <= i2c1rxbuffer(7 downto 0) & i2c1sdasync(1);
                        end if;
                    when leadout =>
                        -- SCL low, SDA high
                        i2c1scl_out <= '0';
                        i2c1sda_out <= '1';
                        -- Count bit time
                        if i2c1bittimer > 0 then
                            i2c1bittimer <= i2c1bittimer - 1;
                        else
                            --i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)));
                            i2c1state <= idle;
                            i2c1tc <= '1';
                            i2c1data_int(7 downto 0) <= i2c1rxbuffer(8 downto 1);
                            i2c1ackfail <= i2c1rxbuffer(0);
                        end if;
                    when send_stopbit_first =>
                        -- SCL low, SDA low
                        i2c1scl_out <= '0';
                        i2c1sda_out <= '0';
                        -- Count bit time
                        if i2c1bittimer > 0 then
                            i2c1bittimer <= i2c1bittimer - 1;
                        else
                            i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)));
                            i2c1state <= send_stopbit_second;
                        end if;
                    when send_stopbit_second =>
                        -- SCL high, SDA low
                        i2c1scl_out <= '1';
                        i2c1sda_out <= '0';
                        -- Count bit timer
                        if i2c1bittimer > 0 then
                            i2c1bittimer <= i2c1bittimer - 1;
                        else
                            if i2c1fastmode = '1' then
                                i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)))*2;
                            else
                                i2c1bittimer <= to_integer(unsigned(i2c1ctrl_int(31 downto 16)));
                            end if;
                            i2c1state <= send_stopbit_third;
                        end if;
                    when send_stopbit_third =>
                        -- SCL high, SCL low, will be set high in idle,
                        -- so there is no need to set SDA high here.
                        i2c1scl_out <= '1';
                        i2c1sda_out <= '1';
                        -- Count bit timer
                        if i2c1bittimer > 0 then
                            i2c1bittimer <= i2c1bittimer - 1;
                        else
                            -- Transmission conplete
                            i2c1tc <= '1';
                            -- Remove STOP bit
                            i2c1stopbit <= '0';
                            -- and goto IDLE
                            i2c1state <= idle;
                            -- Copy data to data register and flag ACK
                            i2c1data_int(7 downto 0) <= i2c1rxbuffer(8 downto 1);
                            i2c1ackfail <= i2c1rxbuffer(0);
                            -- Unregister that we are transmitting
                            i2c1istransmitting <= '0';
                        end if;
                    when others =>
                        i2c1state <= idle;
                end case;
                -- Clear unusd bits
                i2c1data_int(31 downto 8) <= (others => '0');
                i2c1ctrl_int(15 downto 12) <= (others => '0');
                i2c1ctrl_int(7 downto 4) <= (others => '0');
                i2c1stat_int(31 downto 12) <= (others => '0');
            end if;
        end process;
        -- Drive the clock and data lines
        IO_i2c1scl <= '0' when i2c1scl_out = '0' else 'Z';
        IO_i2c1sda <= '0' when i2c1sda_out = '0' else 'Z';
    end generate;
    i2c1gen_not : if not HAVE_I2C1 generate
        i2c1data_int <= (others => '0');
        i2c1ctrl_int <= (others => '0');
        i2c1stat_int <= (others => '0');
        IO_i2c1scl <= 'Z';
        IO_i2c1sda <= 'Z';
    end generate;

    --
    -- SPI1
    --
    spi1gen : if HAVE_SPI1 generate
        process (I_clk, I_areset) is
        variable spi1txshiftcounter_var : integer range 0 to 31;
        variable spi1prescaler_var : integer range 0 to 255;
        begin
            -- Common resets et al.
            if I_areset = '1' then
                spi1data_int <= (others => '0');
                spi1ctrl_int <= (others => '0');
                spi1stat_int <= (others => '0');
                spi1start <= '0';
                spi1state <= idle;
                spi1txbuffer <= (others => '0');
                spi1bittimer <= 0;
                spi1shiftcounter <= 0;
                spi1mosi <= spi1mosidefault;
                spi1rxbuffer <= (others => '0');
                --spi1miso_sync <= '0';
                spi1sck <= '0';
                O_spi1nss <= '1';
            elsif rising_edge(I_clk) then
                -- Default for start transmission
                spi1start <= '0';
                -- Common register writes
                if write_access_granted = '1' then
                    if reg_int = spi1ctrl_addr then
                        -- A write to the control register
                        spi1ctrl_int <= I_datain;
                        -- Set clock polarity
                        spi1sck <= I_datain(2);
                    elsif reg_int = spi1stat_addr then
                        -- A write to the status register
                        spi1stat_int <= I_datain;
                    elsif reg_int = spi1data_addr then
                        -- A write to the data register triggers a transmission
                        -- Signal start
                        spi1start <= '1';
                        -- Load transmit buffer with 8/16/24/32 data bits
                        spi1txbuffer <= (others => '0');
                        spi1data_int <= (others => '0');
                        -- Load the desired bits to transfer
                        case spi1ctrl_int(5 downto 4) is
                            when "00" =>   spi1txbuffer(31 downto 24) <= I_datain(7 downto 0);
                                           spi1shiftcounter <= 7;
                            when "01" =>   spi1txbuffer(31 downto 16) <= I_datain(15 downto 0);
                                           spi1shiftcounter <= 15;
                            when "10" =>   spi1txbuffer(31 downto 8) <= I_datain(23 downto 0);
                                           spi1shiftcounter <= 23;
                            when "11" =>   spi1txbuffer <= I_datain;
                                           spi1shiftcounter <= 31;
                            when others => spi1txbuffer <= (others => '-');
                                           spi1shiftcounter <= 0;
                        end case;
                        -- Signal that we are sending
                        spi1stat_int(3) <= '0'; 
                    end if;
                end if;
                -- Zero out bits not needed
                spi1ctrl_int(31 downto 28) <= (others => '0');
                spi1ctrl_int(11) <= '0';
                spi1ctrl_int(7 downto 6) <= (others => '0');
                spi1ctrl_int(0) <= '0';
                spi1stat_int(31 downto 4) <= (others => '0');
                spi1stat_int(2 downto 0) <= (others =>'0');

                -- Calculate prescaler
                case spi1ctrl_int(10 downto 8) is
                    when "000" => spi1prescaler_var := 0;
                    when "001" => spi1prescaler_var := 1;
                    when "010" => spi1prescaler_var := 3;
                    when "011" => spi1prescaler_var := 7;
                    when "100" => spi1prescaler_var := 15;
                    when "101" => spi1prescaler_var := 31;
                    when "110" => spi1prescaler_var := 63;
                    when "111" => spi1prescaler_var := 127;
                    when others => spi1prescaler_var  := 127;
                end case;

                -- If data register is read...
                if read_access_granted = '1' then
                    if reg_int = spi1data_addr then
                        -- Clear the received status bit
                        spi1stat_int(3) <= '0';
                    end if;
                end if;
                
                -- Transmit/receive
                case spi1state is
                    when idle =>
                        -- Clear receive buffer
                        spi1rxbuffer <= (others => '0');
                        -- If start is active (data written)
                        if spi1start = '1' then
                            -- Activate the NSS (slave select)
                            O_spi1nss <= '0';
                            spi1state <= cssetup;
                            spi1sck <= spi1ctrl_int(2);
                            if spi1ctrl_int(1) = '0' then
                                spi1mosi <= spi1txbuffer(31);
                            else
                                spi1mosi <= spi1mosidefault;
                            end if;
                        else
                            spi1mosi <= spi1mosidefault;
                        end if;
                        -- Load CS setup time before first clock
                        spi1bittimer <= to_integer(unsigned(spi1ctrl_int(27 downto 20)));
                    when cssetup =>
                        -- Wait CS setup time (+1 system clocks)
                        if spi1bittimer > 0 then
                            spi1bittimer <= spi1bittimer - 1;
                        else
                            spi1bittimer <= spi1prescaler_var;
                            spi1state <= first;
                        end if;
                    when first =>
                        if spi1bittimer > 0 then
                            spi1bittimer <= spi1bittimer - 1;
                        else
                            spi1bittimer <= spi1prescaler_var;
                            spi1state <= second;
                            spi1sck <= not spi1ctrl_int(2);
                            -- If CPHA is 0 ...
                            if spi1ctrl_int(1) = '0' then
                                -- Clock in data from slave
                                spi1rxbuffer <= spi1rxbuffer(30 downto 0) & I_spi1miso;
                            else
                                -- CPHA = 1, write out data
                                spi1txbuffer <= spi1txbuffer(30 downto 0) & '0';
                                spi1mosi <= spi1txbuffer(31);
                            end if;
                        end if;
                    when second =>
                        if spi1bittimer > 0 then
                            spi1bittimer <= spi1bittimer - 1;
                        else
                            spi1bittimer <= spi1prescaler_var;
                            spi1sck <= spi1ctrl_int(2);
                            -- If CPHA is 0 ...
                            if spi1ctrl_int(1) = '0' then
                                -- Clock out data
                                spi1txbuffer <= spi1txbuffer(30 downto 0) & '0';
                                spi1mosi <= spi1txbuffer(30);
                            else
                                -- Read in data from slave
                                spi1rxbuffer <= spi1rxbuffer(30 downto 0) & I_spi1miso;
                            end if;
                            -- Are still bits left to transmit?
                            if spi1shiftcounter > 0 then
                                spi1shiftcounter <= spi1shiftcounter - 1;
                                spi1state <= first;
                            else
                                -- All bits transferred
                                if spi1ctrl_int(1) = '1' then
                                    -- CPHA = 1, half SPI clock leadout
                                    spi1state <= leadout;
                                else
                                    -- Copy to data register
                                    spi1data_int <= spi1rxbuffer;
                                    -- Load CS hold time after last clock
                                    spi1bittimer <= to_integer(unsigned(spi1ctrl_int(19 downto 12)));
                                    -- Goto cshold
                                    spi1state <= cshold;
                                end if;
                            end if;
                        end if;
                    when leadout =>
                        if spi1bittimer > 0 then
                            spi1bittimer <= spi1bittimer - 1;
                        else
                            -- Load CS hold time after last clock
                            spi1bittimer <= to_integer(unsigned(spi1ctrl_int(19 downto 12)));
                            spi1state <= cshold;
                        end if;
                        -- Copy to data register
                        spi1data_int <= spi1rxbuffer;
                    when cshold =>
                        -- Wait CS hold time
                        spi1sck <= spi1ctrl_int(2);
                        if spi1bittimer > 0 then
                            spi1bittimer <= spi1bittimer - 1;
                        else
                            -- Disable NSS
                            O_spi1nss <= '1';
                            spi1mosi <= spi1mosidefault;
                            -- Set the received status bit
                            spi1stat_int(3) <= '1';
                            spi1state <= idle;
                        end if;
                    when others => null;
                end case;
            end if; -- rising_edge
        end process;
        O_spi1sck <= spi1sck;
        O_spi1mosi <= spi1mosi;
    end generate;
    spi1gen_not : if not HAVE_SPI1 generate
        spi1ctrl_int <= (others => '0');
        spi1stat_int <= (others => '0');
        spi1data_int <= (others =>'0');
        O_spi1sck <= 'Z';
        O_spi1mosi <= 'Z';
        O_spi1nss <= 'Z';
    end generate;


    --
    -- SPI2
    --
    spi2gen : if HAVE_SPI2 generate
        process (I_clk, I_areset) is
        variable spi2txshiftcounter_var : integer range 0 to 31;
        variable spi2prescaler_var : integer range 0 to 255;
        begin
            -- Common resets et al.
            if I_areset = '1' then
                spi2data_int <= (others => '0');
                spi2ctrl_int <= (others => '0');
                spi2stat_int <= (others => '0');
                spi2start <= '0';
                spi2state <= idle;
                spi2txbuffer <= (others => '0');
                spi2bittimer <= 0;
                spi2shiftcounter <= 0;
                spi2mosi <= spi2mosidefault;
                spi2rxbuffer <= (others => '0');
                --spi2miso_sync <= '0';
                spi2sck <= '0';
            elsif rising_edge(I_clk) then
                -- Default for start transmission
                spi2start <= '0';
                -- Common register writes
                if write_access_granted = '1' then
                    if reg_int = spi2ctrl_addr then
                        -- A write to the control register
                        spi2ctrl_int <= I_datain;
                        -- Set clock polarity
                        spi2sck <= I_datain(2);
                    elsif reg_int = spi2stat_addr then
                        -- A write to the status register
                        spi2stat_int <= I_datain;
                    elsif reg_int = spi2data_addr then
                        -- A write to the data register triggers a transmission
                        -- Signal start
                        spi2start <= '1';
                        -- Load transmit buffer with 8/16/24/32 data bits
                        spi2txbuffer <= (others => '0');
                        spi2data_int <= (others => '0');
                        -- Load the desired bits to transfer
                        case spi2ctrl_int(5 downto 4) is
                            when "00" =>   spi2txbuffer(31 downto 24) <= I_datain(7 downto 0);
                                           spi2shiftcounter <= 7;
                            when "01" =>   spi2txbuffer(31 downto 16) <= I_datain(15 downto 0);
                                           spi2shiftcounter <= 15;
                            when "10" =>   spi2txbuffer(31 downto 8) <= I_datain(23 downto 0);
                                           spi2shiftcounter <= 23;
                            when "11" =>   spi2txbuffer <= I_datain;
                                           spi2shiftcounter <= 31;
                            when others => spi2txbuffer <= (others => '-');
                                           spi2shiftcounter <= 0;
                        end case;
                        -- Signal that we are sending
                        spi2stat_int(3) <= '0'; 
                    end if;
                end if;
                -- Zero out bits not needed
                spi2ctrl_int(31 downto 11) <= (others => '0');
                spi2ctrl_int(7 downto 6) <= (others => '0');
                spi2ctrl_int(3) <= '0';
                spi2ctrl_int(0) <= '0';
                spi2stat_int(31 downto 4) <= (others => '0');
                spi2stat_int(2 downto 0) <= (others =>'0');

                -- Calculate prescaler, 2 to 256 in powers of 2
                case spi2ctrl_int(10 downto 8) is
                    when "000" =>  spi2prescaler_var := 0;
                    when "001" =>  spi2prescaler_var := 1;
                    when "010" =>  spi2prescaler_var := 3;
                    when "011" =>  spi2prescaler_var := 7;
                    when "100" =>  spi2prescaler_var := 15;
                    when "101" =>  spi2prescaler_var := 31;
                    when "110" =>  spi2prescaler_var := 63;
                    when "111" =>  spi2prescaler_var := 127;
                    when others => spi2prescaler_var  := 127;
                end case;

                -- If data register is read...
                if read_access_granted = '1' then
                    if reg_int = spi2data_addr then
                        -- Clear the received status bit
                        spi2stat_int(3) <= '0';
                    end if;
                end if;
                
                -- Transmit/receive
                case spi2state is
                    when idle =>
                        -- Clear receive buffer
                        spi2rxbuffer <= (others => '0');
                        -- Load prescaler value
                        spi2bittimer <= spi2prescaler_var;
                        -- If start is active (data written)
                        if spi2start = '1' then
                            spi2state <= first;
                            spi2sck <= spi2ctrl_int(2);
                            if spi2ctrl_int(1) = '0' then
                                spi2mosi <= spi2txbuffer(31);
                            else
                                -- CPHA = 1, write out data
                                spi2txbuffer <= spi2txbuffer(30 downto 0) & '0';
                                spi2mosi <= spi2txbuffer(31);
                                spi2sck <= not spi2ctrl_int(2);
                                spi2state <= second;
                            end if;
                        else
                            spi2mosi <= spi2mosidefault;
                        end if;
                    when first =>
                        if spi2bittimer > 0 then
                            spi2bittimer <= spi2bittimer - 1;
                        else
                            spi2bittimer <= spi2prescaler_var;
                            spi2state <= second;
                            spi2sck <= not spi2ctrl_int(2);
                            if spi2ctrl_int(1) = '0' then
                                -- CPHA = 0, clock in data from slave
                                spi2rxbuffer <= spi2rxbuffer(30 downto 0) & I_spi2miso;
                            else
                                -- CPHA = 1, write out data
                                spi2txbuffer <= spi2txbuffer(30 downto 0) & '0';
                                spi2mosi <= spi2txbuffer(31);
                            end if;
                        end if;
                    when second =>
                        if spi2bittimer > 0 then
                            spi2bittimer <= spi2bittimer - 1;
                        else
                            spi2bittimer <= spi2prescaler_var;
                            spi2sck <= spi2ctrl_int(2);
                            if spi2ctrl_int(1) = '0' then
                                -- If CPHA is 0, clock out data
                                spi2txbuffer <= spi2txbuffer(30 downto 0) & '0';
                                -- Must be spi2buffer(30) because data is not yet shifted
                                spi2mosi <= spi2txbuffer(30);
                            else
                                -- If CPHA = 1, read in data from slave
                                spi2rxbuffer <= spi2rxbuffer(30 downto 0) & I_spi2miso;
                            end if;
                            -- Are still bits left to transmit?
                            if spi2shiftcounter > 0 then
                                spi2shiftcounter <= spi2shiftcounter - 1;
                                spi2state <= first;
                            else
                                -- All bits transferred
                                if spi2ctrl_int(1) = '1' then
                                    -- CPHA = 1, half SPI clock leadout
                                    spi2state <= leadout;
                                else
                                    -- CPHA = 0, no leadout, goto idle
                                    spi2sck <= spi2ctrl_int(2);
                                    spi2stat_int(3) <= '1';
                                    spi2mosi <= spi2mosidefault;
                                    spi2state <= idle;
                                    -- Copy to data register
                                    spi2data_int <= spi2rxbuffer;
                                end if;
                            end if;
                        end if;
                    when leadout =>
                        -- Hold the data half SPI clock cycle
                        if spi2bittimer > 0 then
                            spi2bittimer <= spi2bittimer - 1;
                        else
                            spi2sck <= spi2ctrl_int(2);
                            spi2stat_int(3) <= '1';
                            spi2mosi <= spi2mosidefault;
                            spi2state <= idle;
                        end if;
                        -- Copy to data register
                        spi2data_int <= spi2rxbuffer;
                when others => null;
                end case;
            end if; -- rising_edge
        end process;
        O_spi2sck <= spi2sck;
        O_spi2mosi <= spi2mosi;
    end generate;
    spi2gen_not : if not HAVE_SPI2 generate
        spi2ctrl_int <= (others => '0');
        spi2stat_int <= (others => '0');
        spi2data_int <= (others =>'0');
        O_spi2sck <= 'Z';
        O_spi2mosi <= 'Z';
    end generate;

    
    --
    -- TIMER1 - a very simple timer
    --
    timer1gen : if HAVE_TIMER1 generate
        process (I_clk, I_areset) is
        begin
            if I_areset = '1' then
                timer1ctrl_int <= (others => '0');
                timer1stat_int <= (others => '0');
                timer1cntr_int <= (others => '0');
                timer1cmpt_int <= (others => '0');
            elsif rising_edge(I_clk) then
                if write_access_granted = '1' then
                    -- Write Timer Control Register
                    if reg_int = timer1ctrl_addr then
                        timer1ctrl_int <= I_datain;
                    end if;
                    -- Write Timer Status Register
                    if reg_int = timer1stat_addr then
                        timer1stat_int <= I_datain;
                    end if;
                    -- Write Timer Counter Register
                    if reg_int = timer1cntr_addr then
                        timer1cntr_int <= I_datain;
                    end if;
                    -- Write Timer Compare Register
                    if reg_int = timer1cmpt_addr then
                        timer1cmpt_int <= I_datain;
                    end if;
                end if;
                -- Set unused bits to 0
                timer1ctrl_int(31 downto 12) <= (others => '0');
                timer1stat_int(31 downto 12) <= (others => '0');
                
                -- If timer is enabled....
                if timer1ctrl_int(0) = '1' then
                    -- If we hit the Compare Register T...
                    if timer1cntr_int >= timer1cmpt_int then
                        -- Reload Counter Register
                        timer1cntr_int <= (others => '0');
                        -- Signal hit
                        timer1stat_int(4) <= '1';
                    else
                        -- else, increment the Counter Register
                        timer1cntr_int <= std_logic_vector(unsigned(timer1cntr_int) + 1);
                    end if;
                end if;
            end if;
        end process;
    end generate;
    timer1gen_not : if not HAVE_TIMER1 generate
        timer1ctrl_int <= (others => '0');
        timer1stat_int <= (others => '0');
        timer1cntr_int <= (others => '0');
        timer1cmpt_int <= (others => '0');
    end generate;

    --
    -- TIMER2 - a more elaborate timer
    --
    timer2gen : if HAVE_TIMER2 generate
        process (I_clk, I_areset) is
        begin
            if I_areset = '1' then
                -- The I/O registers
                timer2ctrl_int <= (others => '0');
                timer2stat_int <= (others => '0');
                timer2cntr_int <= (others => '0');
                timer2cmpt_int <= (others => '0');
                timer2prsc_int <= (others => '0');
                timer2cmpa_int <= (others => '0');
                timer2cmpb_int <= (others => '0');
                timer2cmpc_int <= (others => '0');
                -- The internal prescaler
                timer2prescaler_int <= (others => '0');
                -- The shadow registers
                timer2prscshadow_int <= (others => '0');
                timer2cmptshadow_int <= (others => '0');
                timer2cmpashadow_int <= (others => '0');
                timer2cmpbshadow_int <= (others => '0');
                timer2cmpcshadow_int <= (others => '0');
                -- The OC outputs
                timer2oct_int <= '0';
                timer2oca_int <= '0';
                timer2ocb_int <= '0';
                timer2occ_int <= '0';
            elsif rising_edge(I_clk) then
                if write_access_granted = '1' then
                    -- Write Timer Control Register
                    if reg_int = timer2ctrl_addr then
                        -- Check if one or more FOC bits are set
                        -- If so, the data is NOT copied to the CTRL register
                        -- and the MODE bits indicate the FOC action
                        if I_datain(31 downto 28) /= "0000" then
                            -- FOCT
                            if I_datain(28) = '1' then
                                case I_datain(14 downto 12) is
                                    when "001" => timer2oct_int <= not timer2oct_int;
                                    when "010" => timer2oct_int <= '1';
                                    when "011" => timer2oct_int <= '0';
                                    when others => null;
                                end case;
                            end if;
                            -- FOCA
                            if I_datain(29) = '1' then
                                case I_datain(18 downto 16) is
                                    when "001" => timer2oca_int <= not timer2oca_int;
                                    when "010" => timer2oca_int <= '1';
                                    when "011" => timer2oca_int <= '0';
                                    when others => null;
                                end case;
                            end if;
                            -- FOCB
                            if I_datain(30) = '1' then
                                case I_datain(22 downto 20) is
                                    when "001" => timer2ocb_int <= not timer2ocb_int;
                                    when "010" => timer2ocb_int <= '1';
                                    when "011" => timer2ocb_int <= '0';
                                    when others => null;
                                end case;
                            end if;
                            -- FOCC
                            if I_datain(31) = '1' then
                                case I_datain(26 downto 24) is
                                    when "001" => timer2occ_int <= not timer2occ_int;
                                    when "010" => timer2occ_int <= '1';
                                    when "011" => timer2occ_int <= '0';
                                    when others => null;
                                end case;
                            end if;
                        else
                            -- No FOC bits set, so ...
                            -- Copy to CTRL register
                            timer2ctrl_int <= I_datain;
                            -- Set the signal phase
                            timer2oct_int <= I_datain(15);
                            timer2oca_int <= I_datain(19);
                            timer2ocb_int <= I_datain(23);
                            timer2occ_int <= I_datain(27);
                        end if;
                    end if;
                    -- Write Timer Status Register
                    if reg_int = timer2stat_addr then
                        timer2stat_int <= I_datain;
                    end if;
                    -- Write Timer Counter Register
                    if reg_int = timer2cntr_addr then
                        timer2cntr_int <= I_datain;
                    end if;
                    -- Write Timer Compare T Register
                    if reg_int = timer2cmpt_addr then
                        timer2cmpt_int <= I_datain;
                        -- If the timer is stopped or preload is off, directly write the shadow register
                        if timer2ctrl_int(0) = '0' or timer2ctrl_int(8) = '0' then
                            timer2cmptshadow_int <= I_datain;
                        end if;
                    end if;
                    -- Write Prescaler Register
                    if reg_int = timer2prsc_addr then
                        timer2prsc_int <= I_datain;
                        -- If the timer is stopped, directly write the shadow register
                        if timer2ctrl_int(0) = '0' then
                            timer2prscshadow_int <= I_datain;
                        end if;
                        -- Reset internal prescaler
                        timer2prescaler_int <= (others => '0');
                    end if;
                    -- Write Timer Compare A Register
                    if reg_int = timer2cmpa_addr then
                        timer2cmpa_int <= I_datain;
                        -- If the timer is stopped or preload is off, directly write the shadow register
                        if timer2ctrl_int(0) = '0' or timer2ctrl_int(9) = '0' then
                            timer2cmpashadow_int <= I_datain;
                        end if;
                    end if;
                    -- Write Timer Compare B Register
                    if reg_int = timer2cmpb_addr then
                        timer2cmpb_int <= I_datain;
                        -- If the timer is stopped or preload is off, directly write the shadow register
                        if timer2ctrl_int(0) = '0' or timer2ctrl_int(10) = '0' then
                            timer2cmpbshadow_int <= I_datain;
                        end if;
                    end if;
                    -- Write Timer Compare C Register
                    if reg_int = timer2cmpc_addr then
                        timer2cmpc_int <= I_datain;
                        -- If the timer is stopped or preload is off, directly write the shadow register
                        if timer2ctrl_int(0) = '0' or timer2ctrl_int(11) = '0' then
                            timer2cmpcshadow_int <= I_datain;
                        end if;
                    end if;
                end if;
                -- Set unused bits to 0 for CTRL and STAT
                timer2ctrl_int(31 downto 28) <= (others => '0');
                timer2stat_int(31 downto 12) <= (others => '0');
                
                -- If timer is enabled....
                if timer2ctrl_int(0) = '1' then
                    -- If internal prescaler at end...
                    if timer2prescaler_int >= timer2prscshadow_int then
                        -- Wrap internal prescaler
                        timer2prescaler_int <= (others => '0');
                        -- If we hit the Compare Register T...
                        if timer2cntr_int >= timer2cmptshadow_int then
                            -- Clear Counter Register
                            timer2cntr_int <= (others => '0');
                            -- Signal hit
                            timer2stat_int(4) <= '1';
                            -- Toggle OCT, or not
                            case timer2ctrl_int(14 downto 12) is
                                when "000" => timer2oct_int <= '0'; -- off
                                when "001" => timer2oct_int <= not timer2oct_int; -- toggle
                                when "010" => timer2oct_int <= not timer2ctrl_int(15); -- invert PHAT
                                when "011" => timer2oct_int <= timer2ctrl_int(15); -- write PHAT
                                -- Others not allowed, as T does not have PWM mode
                                when others => timer2oct_int <= '-';
                            end case;
                            -- If we have a one-shot, disable timer
                            if timer2ctrl_int(3) = '1' then
                                timer2ctrl_int(0) <= '0';
                                timer2prescaler_int <= (others => '0');
                                --timer2cntr_int <= (others => '0');
                            end if;
                        else
                            -- If we are at the last step - 1 ...
                            if timer2cntr_int = std_logic_vector(unsigned(timer2cmptshadow_int)-1) then
                                -- Load PRSC shadow register
                                timer2prscshadow_int <= timer2prsc_int;
                                -- Load CMPT shadow register
                                timer2cmptshadow_int <= timer2cmpt_int;
                                -- Load CMPA shadow register
                                timer2cmpashadow_int <= timer2cmpa_int;
                                -- Load CMPB shadow register
                                timer2cmpbshadow_int <= timer2cmpb_int;
                                -- Load CMPC shadow register
                                timer2cmpcshadow_int <= timer2cmpc_int;
                            end if;
                            -- else, increment the Counter Register
                            timer2cntr_int <= std_logic_vector(unsigned(timer2cntr_int) + 1);
                        end if;
                        -- If we hit the Compare Register A
                        if timer2cntr_int = timer2cmpa_int then
                            -- Signal hit
                            timer2stat_int(5) <= '1';
                        end if;
                        -- If we hit the Compare Register B...
                        if timer2cntr_int = timer2cmpb_int then
                            -- Signal hit
                            timer2stat_int(6) <= '1';
                        end if;
                        -- If we hit the Compare Register C...
                        if timer2cntr_int = timer2cmpc_int then
                            -- Signal hit
                            timer2stat_int(7) <= '1';
                        end if;
                    else
                        timer2prescaler_int <= std_logic_vector(unsigned(timer2prescaler_int) + 1);
                    end if;
                    -- If we are at the end of prescale counting
                    if timer2prescaler_int >= timer2prscshadow_int then
                        -- Check CMPA for mode
                        case timer2ctrl_int(18 downto 16) is
                            -- 000 = do nothing
                            when "000" => timer2oca_int <= '0';
                            -- 001 = toggle on compare match
                            when "001" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpashadow_int)-1) then
                                    timer2oca_int <= not timer2oca_int;
                                end if;
                            -- 010 = activate on compare match, invert PHAA
                            when "010" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpashadow_int)-1) then
                                    timer2oca_int <= not timer2ctrl_int(19);
                                end if;
                            -- 011 = deactivate on compare match, write PHAA
                            when "011" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpashadow_int)-1) then
                                    timer2oca_int <= timer2ctrl_int(19);
                                end if;
                            -- 100 = edge aligned PWM
                            when "100" =>
                                if timer2cmpashadow_int = x"00000000" then
                                    timer2oca_int <= timer2ctrl_int(19);
                                elsif timer2cntr_int < std_logic_vector(unsigned(timer2cmpashadow_int)-1) or (timer2cntr_int = timer2cmptshadow_int and timer2ctrl_int(3) = '0') then
                                    timer2oca_int <= not timer2ctrl_int(19);
                                else
                                    timer2oca_int <= timer2ctrl_int(19);
                                end if;
                            -- Others not allowed
                            when others => timer2oca_int <= '-';
                        end case;
                        -- Check CMPB for mode
                        case timer2ctrl_int(22 downto 20) is
                            -- 000 = do nothing
                            when "000" => timer2ocb_int <= '0';
                            -- 001 = toggle on compare match
                            when "001" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpbshadow_int)-1) then
                                    timer2ocb_int <= not timer2ocb_int;
                                end if;
                            -- 010 = activate on compare match, invert PHAB
                            when "010" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpbshadow_int)-1) then
                                    timer2ocb_int <= not timer2ctrl_int(23);
                                end if;
                            -- 011 = deactivate on compare match, write PHAB
                            when "011" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpbshadow_int)-1) then
                                    timer2ocb_int <= timer2ctrl_int(23);
                                end if;
                            -- 100 = edge aligned PWM
                            when "100" =>
                                if timer2cmpbshadow_int = x"00000000" then
                                    timer2ocb_int <= timer2ctrl_int(23);
                                elsif timer2cntr_int < std_logic_vector(unsigned(timer2cmpbshadow_int)-1) or (timer2cntr_int = timer2cmptshadow_int and timer2ctrl_int(3) = '0') then
                                    timer2ocb_int <= not timer2ctrl_int(23);
                                else
                                    timer2ocb_int <= timer2ctrl_int(23);
                                end if;
                            when others => timer2ocb_int <= '-';
                        end case;
                        -- Check CMPC for mode
                        case timer2ctrl_int(26 downto 24) is
                            -- 000 = do nothing
                            when "000" => timer2occ_int <= '0';
                            -- 001 = toggle on compare match
                            when "001" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpcshadow_int)-1) then
                                    timer2occ_int <= not timer2occ_int;
                                end if;
                            -- 010 = activate on compare match, invert PHAC
                            when "010" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpcshadow_int)-1) then
                                    timer2occ_int <= not timer2ctrl_int(27);
                                end if;
                            -- 011 = deactivate on compare match, write PHAC
                            when "011" =>
                                if timer2cntr_int = std_logic_vector(unsigned(timer2cmpcshadow_int)-1) then
                                    timer2occ_int <= timer2ctrl_int(27);
                                end if;
                            -- 100 = edge aligned PWM
                            when "100" =>
                                if timer2cmpcshadow_int = x"00000000" then
                                    timer2occ_int <= timer2ctrl_int(27);
                                elsif timer2cntr_int < std_logic_vector(unsigned(timer2cmpcshadow_int)-1) or (timer2cntr_int = timer2cmptshadow_int and timer2ctrl_int(3) = '0') then
                                    timer2occ_int <= not timer2ctrl_int(27);
                                else
                                    timer2occ_int <= timer2ctrl_int(27);
                                end if;
                            when others => timer2occ_int <= '-';
                        end case;
                    end if;
                else
                end if;
                -- Set unused bits, all counter registers are 16 bits.
                timer2cntr_int(31 downto 16) <= (others => '0');
                timer2cmpt_int(31 downto 16) <= (others => '0');
                timer2prsc_int(31 downto 16) <= (others => '0');
                timer2cmpa_int(31 downto 16) <= (others => '0');
                timer2cmpb_int(31 downto 16) <= (others => '0');
                timer2cmpc_int(31 downto 16) <= (others => '0');
                timer2prescaler_int(31 downto 16) <= (others => '0');
                timer2prscshadow_int(31 downto 16) <= (others => '0');
                timer2cmptshadow_int(31 downto 16) <= (others => '0');
                timer2cmpashadow_int(31 downto 16) <= (others => '0');
                timer2cmpbshadow_int(31 downto 16) <= (others => '0');
                timer2cmpcshadow_int(31 downto 16) <= (others => '0');
            end if;
        end process;
        -- Output the Output Compare match
        O_timer2oct <= timer2oct_int;
        O_timer2oca <= timer2oca_int;
        O_timer2ocb <= timer2ocb_int;
        O_timer2occ <= timer2occ_int;
    end generate;
    timer2gen_not : if not HAVE_TIMER2 generate
        timer2ctrl_int <= (others => '0');
        timer2stat_int <= (others => '0');
        timer2cntr_int <= (others => '0');
        timer2cmpt_int <= (others => '0');
        timer2prsc_int <= (others => '0');
        timer2cmpa_int <= (others => '0');
        timer2cmpb_int <= (others => '0');
        timer2cmpc_int <= (others => '0');
        O_timer2oct <= 'Z';
        O_timer2oca <= 'Z';
        O_timer2ocb <= 'Z';
        O_timer2occ <= 'Z';
   end generate;

    --
    -- RISC-V system timer TIME and TIMECMP
    -- These registers are memory mapped
    --
    process (I_clk, I_areset, io) is
    variable mtime_reg : unsigned(63 downto 0);
    variable mtimecmp_reg : unsigned(63 downto 0);
    variable prescaler : integer range 0 to freq_sys/freq_count-1;
    begin
        if I_areset = '1' then
            mtime_reg := (others => '0');
            mtimecmp_reg := (others => '0');
            prescaler := 0;
        elsif rising_edge(I_clk) then
            if write_access_granted = '1' then
--                -- Load time (low 32 bits)
--                if reg_int = mtime_addr then
--                    mtime_reg(31 downto 0) := unsigned(I_datain);
--                end if;
--                -- Load timeh (high 32 bits)
--                if reg_int = mtimeh_addr then
--                    mtime_reg(63 downto 32) := unsigned(I_datain);
--                end if;
                -- Load compare register (low 32 bits)
                if reg_int = mtimecmp_addr then
                    mtimecmp_reg(31 downto 0) := unsigned(I_datain);
                end if;
                -- Load compare register (high 32 bits)
                if reg_int = mtimecmph_addr then
                    mtimecmp_reg(63 downto 32) := unsigned(I_datain);
                end if;
            end if;
            -- Update system timer
            if prescaler = freq_sys/freq_count-1 then
                prescaler := 0;
                mtime_reg := mtime_reg + 1;
            else
                prescaler := prescaler + 1;
            end if;
        end if;
        mtime_int <= std_logic_vector(mtime_reg(31 downto 0));
        mtimeh_int <= std_logic_vector(mtime_reg(63 downto 32));
        mtimecmp_int <= std_logic_vector(mtimecmp_reg(31 downto 0));
        mtimecmph_int <= std_logic_vector(mtimecmp_reg(63 downto 32));
        -- If compare register >= time register, assert interrupt
        if mtime_reg >= mtimecmp_reg then
            O_intrio(7) <= '1';
        else
            O_intrio(7) <= '0';
        end if;
        O_mtime <= mtime_int;
        O_mtimeh <= mtimeh_int;
    end process;
    
    --
    -- Interrupt generation
    --
    
    -- Unused local interrupts set to 0
    O_intrio(31 downto 22) <= (others => '0');
    -- RISC-V interrupts not used, except System Timer
    O_intrio(15 downto 8) <= (others => '0');
    O_intrio(6 downto 0) <= (others => '0');

    -- O_intrio(7) is set by the System Timer

    -- SPI1 transmit complete interrupt
    O_intrio(21) <= '1' when spi1ctrl_int(3) = '1' and spi1stat_int(3) = '1' else '0';
    -- I2C1 transmit interrupt.
    O_intrio(20) <= '1' when (i2c1ctrl_int(3) = '1' and i2c1stat_int(3) = '1') else '0';
    -- TIMER2 compare match T/A/B/C interrupt
    O_intrio(19) <= '1' when (timer2ctrl_int(4) = '1' and timer2stat_int(4) = '1') or
                             (timer2ctrl_int(5) = '1' and timer2stat_int(5) = '1') or
                             (timer2ctrl_int(6) = '1' and timer2stat_int(6) = '1') or
                             (timer2ctrl_int(7) = '1' and timer2stat_int(7) = '1') else '0';
    -- UART1 receive or transmit interrupt. Software must determine if it was
    -- receive or transmit or both
    O_intrio(18) <= '1' when (uart1stat_int(4) = '1' and uart1ctrl_int(7) = '1') or
                             (uart1stat_int(2) = '1' and uart1ctrl_int(6) = '1') else '0';
    -- TIMER1 compare match interrupt
    O_intrio(17) <= '1' when timer1ctrl_int(4) = '1' and timer1stat_int(4) = '1' else '0';
    -- This next interrupt is for testing only, will be removed
    O_intrio(16) <= '1' when gpioapin_int(0) = '1' else '0';    
    
    
end architecture rtl;