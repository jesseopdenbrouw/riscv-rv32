-- #################################################################################################
-- # core.vhd - The processor core                                                                 #
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

-- This file contains the description of a RISC-V RV32IM core,
-- using a three-stage pipeline. It contains the PC, the
-- instruction decoder and the ALU, the MD unit and the
-- memory interface unit.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.processor_common.all;
use work.processor_common_rom.all;

entity core is
    port (I_clk : in std_logic;
          I_areset : in std_logic;
          -- Instructions from ROM
          O_pc : out data_type;
          I_instr : in data_type;
          O_stall : out std_logic;
          -- To memory
          O_memaccess : out memaccess_type;
          O_memsize : out memsize_type;
          O_memaddress : out data_type;
          O_memdataout : out data_type; 
          I_memdatain : in data_type;
          I_waitfordata : in std_logic;
          -- To CSR
          O_instret : out std_logic;
          O_csr_op : out csr_op_type;
          O_csr_addr: out csraddr_type;
          O_csr_immrs1 : out reg_type;
          O_csr_dataout : out data_type;
          I_csr_datain : in data_type;
          -- CSR - Trap handling
          O_ecall_request : out std_logic;
          O_ebreak_request : out std_logic;
          O_mret_request : out std_logic;
          I_interrupt_request : in interrupt_request_type;
          I_mtvec : in data_type;
          O_pc_to_mepc : out data_type;
          I_mepc : in data_type;
          --Instruction error
          O_illegal_instruction_error : out std_logic
         );
end entity core;

architecture rtl of core is

-- The Program Counter et al.
-- Not part of any record.
signal pc : data_type;

-- IF/ID signals for Instruction Decode stage
type if_id_type is record
    pc : data_type;
    -- synthesis translate_off
    instr_decode : data_type;
    -- synthesis translate_on
end record if_id_type;
signal if_id : if_id_type;

-- ID/EX signals for Execute stage
    -- Behavior of the Program Counter
type pc_op_type is (pc_hold, pc_incr, pc_loadoffset, pc_loadoffsetregister,
                    pc_branch, pc_load_mepc, pc_load_mtvec);
type id_ex_type is record
    alu_op : alu_op_type;
    rd : reg_type;
    rd_en : std_logic;
    rs1 : reg_type;
    rs2 : reg_type;
    rs1data : data_type;
    rs2data : data_type;
    imm : data_type;
    isimm : std_logic;
    isunsigned : std_logic;
    md_op : func3_type;
    md_start : std_logic;
    memaccess : memaccess_type;
    memsize : memsize_type;
    pc_op : pc_op_type;
    pc : data_type;
    csr_op : csr_op_type;
    -- The result is not clocked and really should be in ex_wb...
    result : data_type;
end record id_ex_type;
signal id_ex : id_ex_type;

-- EX/WB for Write Back stage
type ex_wb_type is record
    rd : reg_type;
    rd_en : std_logic;
    rddata : data_type;
end record ex_wb_type;
signal ex_wb : ex_wb_type;

-- The registers
type regs_array_type is array (0 to NUMBER_OF_REGISTERS-1) of data_type;
signal regs : regs_array_type;
-- Do not check for read during write. For some reason, Quartus
-- thinks that there are asynchronous read and write clocks.
attribute ramstyle : string;
attribute ramstyle of regs : signal is "no_rw_check";

-- Control signals
type state_type is (state_boot0, state_boot1, state_exec, state_wait,
                    state_flush, state_md, state_intr, state_intr2,
                    state_mret, state_mret2);
type control_type is record
    stall : std_logic;
    flush : std_logic;
    penalty : std_logic;
    state : state_type;
    ebreak_request : std_logic;
    ecall_request : std_logic;
    mret_request : std_logic;
    select_pc : std_logic;
    forwarda : std_logic;
    forwardb : std_logic;
    -- Write x0 with 0 at startup if regs are in RAM
    reg0_write_once : std_logic;
    -- Determine the correct PC to be loaded into mepc on trap
    pc_to_mepc : data_type;
end record control_type;
signal control : control_type;

type md_type is record
    -- Operation ready
    ready : std_logic;
    -- Multiplier
    rdata_a, rdata_b : unsigned(32 downto 0);
    mul_rd_int : signed(65 downto 0);
    mul_running : std_logic;
    mul_ready : std_logic;
    mul : data_type;
    -- Divider
    buf : unsigned(63 downto 0);
    divisor : unsigned(31 downto 0);
    divisor1: unsigned(33 downto 0);
    divisor2: unsigned(33 downto 0);
    divisor3: unsigned(33 downto 0);
    quotient : unsigned(31 downto 0);
    remainder : unsigned(31 downto 0);
    outsign : std_logic;
    div_ready : std_logic;
    div : data_type;
    count: integer range 0 to 32;
end record md_type;
signal md : md_type;
alias md_buf1 is md.buf(63 downto 32);
alias md_buf2 is md.buf(31 downto 0);

constant all_zeros : std_logic_vector(31 downto 0) := (others => '0');

begin

    --
    -- Control block:
    -- This block holds the current processing state of the
    -- processor and supplies the control signals to the
    -- other block.
    --
   
   -- Determine which PC value must be loaded into mepc on trap
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            control.pc_to_mepc <= (others => '0');
        elsif rising_edge(I_clk) then
            if control.state /= state_flush then
                control.pc_to_mepc <= id_ex.pc;
            end if;
        end if;
    end process;
    O_pc_to_mepc <= id_ex.pc when control.select_pc = '1' else control.pc_to_mepc;
    
    
    -- Processor state control
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            control.state <= state_boot0;
        elsif rising_edge(I_clk) then
            case control.state is
                -- Booting first cycle
                when state_boot0 =>
                    control.state <= state_boot1;
                -- Booting second cycle
                when state_boot1 =>
                    control.state <= state_exec;
                -- The executing state
                when state_exec =>
                    -- Trap request
                    if I_interrupt_request = irq_hard then
                        control.state <= state_intr;
                    -- If we have an mret request (MRET)
                    elsif control.mret_request = '1' then
                        control.state <= state_mret;
                    -- If we have a penalty, we have to flush the pipeline for two cycles
                    elsif control.penalty = '1' then
                        control.state <= state_flush;
                    -- If we have to wait for data, we need to wait one extra cycle
                    elsif I_waitfordata = '1' then
                        control.state <= state_wait;
                    -- If the MD unit is started....
                    elsif id_ex.md_start = '1' then
                        control.state <= state_md;
                    end if;
                -- Wait for data (read from ROM, boot ROM, RAM or I/O)
                when state_wait =>
                    -- Trap request
                    if I_interrupt_request = irq_hard then
                        control.state <= state_intr;
                    else
                        control.state <= state_exec;
                    end if;
                -- Flush
                when state_flush =>
                    -- Trap request
                    if I_interrupt_request = irq_hard then
                        control.state <= state_intr;
                    else
                        control.state <= state_exec;
                    end if;
                -- MD operation in progress
                when state_md =>
                    -- Trap request
                    if I_interrupt_request = irq_hard then
                        control.state <= state_intr;
                    elsif md.ready = '1' then
                        control.state <= state_exec;
                    end if;
                -- First state of trap handling, flushes pipeline
                when state_intr =>
                    control.state <= state_intr2;
                -- Second state of trap handling, flushes pipeline
                when state_intr2 =>
                    control.state <= state_exec;
                -- First state of MRET, flushes the pipeline
                when state_mret =>
                    control.state <= state_mret2;
                -- Second state of MRET, flushes the pipeline
                when state_mret2 =>
                    control.state <= state_exec;
                when others =>
                    control.state <= state_exec;
            end case;
        end if;
    end process;
    
    -- Determine stall
    -- We need to stall if we are waiting for data from memory OR we stall the PC and md unit is not ready
    control.stall <= '1' when (control.state = state_exec and I_waitfordata = '1') or
                              (control.state = state_md) or
                              (control.state = state_exec and id_ex.md_start = '1')
                         else '0';
    -- Needed for the instruction fetch for the ROM or boot ROM
    O_stall <= control.stall;

    -- We need to flush if we are jumping/branching or servicing interrupts
    control.flush <= '1' when control.penalty = '1' or control.state = state_flush or
                              control.state = state_intr or 
                              control.state = state_intr2 or
                              control.state = state_mret or
                              control.state = state_boot0
                         else '0'; -- for now

    -- Instructions retired -- not exact, needs more detail
    O_instret <= '1' when (control.state = state_exec and I_interrupt_request = irq_none and I_waitfordata = '0' and id_ex.md_start = '0' and control.penalty = '0') or
                          (control.state = state_wait and I_interrupt_request = irq_none) else '0'; 
    
    -- Signal trap related
    O_ecall_request <= control.ecall_request;
    O_ebreak_request <= control.ebreak_request;
    O_mret_request <= '1' when control.state = state_mret2 else '0';

    -- Data forwarder. Forward RS1/RS2 if they are used in current instruction,
    -- and were written in the previous instruction.
    process (id_ex, ex_wb) is
    begin
        if ex_wb.rd_en = '1' and ex_wb.rd = id_ex.rs1 then
            control.forwarda <= '1';
        else
            control.forwarda <= '0';
        end if;
        if ex_wb.rd_en = '1' and ex_wb.rd = id_ex.rs2 then
            control.forwardb <= '1';
        else
            control.forwardb <= '0';
        end if;
    end process;


    --
    -- Instruction fetch block
    -- This block controls the instruction fetch from the ROM.
    -- It also instructs the PC to load a new address, either
    -- the next sequencial address or a jump target address.
    --
    
    -- The PC
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            pc <= (others => '0');
            if HAVE_BOOTLOADER_ROM then
                pc(31 downto 28) <= bootloader_high_nibble;
            else
                pc(31 downto 28) <= rom_high_nibble;
            end if;
        elsif rising_edge(I_clk) then
            -- Should we stall the pipeline
            if control.stall = '1' then
                -- PC holds value
                null;
            else
                case id_ex.pc_op is
                    -- Hold the PC
                    when pc_hold =>
                        null;
                    -- Increment the PC
                    when pc_incr =>
                        pc <= std_logic_vector(unsigned(pc) + 4);
                    -- JAL
                    when pc_loadoffset =>
                        pc <= std_logic_vector(unsigned(id_ex.pc) + unsigned(id_ex.imm));
                    -- JALR
                    when pc_loadoffsetregister =>
                        -- Check forwarding
                        if control.forwarda = '1' then
                            pc <= std_logic_vector(unsigned(id_ex.imm) + unsigned(ex_wb.rddata));
                        else
                            pc <= std_logic_vector(unsigned(id_ex.imm) + unsigned(id_ex.rs1data));
                        end if;
                    -- Branch
                    when pc_branch =>
                        -- Must we branch?
                        if control.penalty = '1' then
                            pc <= std_logic_vector(unsigned(id_ex.pc) + unsigned(id_ex.imm));
                        else
                            pc <= std_logic_vector(unsigned(pc) + 4);
                        end if;
                    -- Load mtvec but only if we must
                    when pc_load_mtvec =>
                        pc <= I_mtvec;
                    -- Load mepc
                    when pc_load_mepc =>
                        pc <= I_mepc;
                    when others =>
                        pc <= std_logic_vector(unsigned(pc) + 4);
                end case;
            end if;
            -- Lower two bits always 0
            pc(1 downto 0) <= "00";
        end if;
    end process;
    -- For fetching instructions
    O_pc <= pc;
    
    -- The PC at the fetched instruction
    process (I_clk, I_areset) is
    variable instr_var : data_type;
    begin
        if I_areset = '1' then
            -- Set at 0x00000000 because after reset
            -- the processor will run for two booting
            -- states. After that, this PC will follow
            -- the PC.
            if_id.pc <= (others => '0');
        elsif rising_edge(I_clk) then
            -- Must we stall?
            if control.stall = '1' or id_ex.pc_op = pc_hold then
                null;
            else
                if_id.pc <= pc;
            end if;
        end if;
    end process;
    
    --
    -- Instruction decode block
    -- This block decodes the instruction
    --
    
    -- Decode the instruction
    process (I_clk, I_areset, I_instr, control) is
    variable opcode_v : std_logic_vector(6 downto 0);
    variable func3_v : std_logic_vector(2 downto 0);
    variable func7_v : std_logic_vector(6 downto 0);
    variable imm_u_v : data_type;
    variable imm_j_v : data_type;
    variable imm_i_v : data_type;
    variable imm_b_v : data_type;
    variable imm_s_v : data_type;
    variable imm_shamt_v : data_type;
    variable rs1_v, rs2_v, rd_v : reg_type;
    variable selrs1_v : integer range 0 to NUMBER_OF_REGISTERS-1;
    variable selrs2_v : integer range 0 to NUMBER_OF_REGISTERS-1;
    begin

        -- Replace opcode with a nop if we flush
        if control.flush = '1' then
            opcode_v := "0010011"; --nop
            rd_v := (others => '0');
        else
            -- Get the opcode
            opcode_v := I_instr(6 downto 0);
            rd_v := I_instr(11 downto 7);
        end if;

        -- Registers to select
        rs1_v := I_instr(19 downto 15);
        rs2_v := I_instr(24 downto 20);

        -- Get function (extends the opcode)
        func3_v := I_instr(14 downto 12);
        func7_v := I_instr(31 downto 25);

        -- Create all immediate formats
        imm_u_v(31 downto 12) := I_instr(31 downto 12);
        imm_u_v(11 downto 0) := (others => '0');
        
        imm_j_v(31 downto 21) := (others => I_instr(31));
        imm_j_v(20 downto 1) := I_instr(31) & I_instr(19 downto 12) & I_instr(20) & I_instr(30 downto 21);
        imm_j_v(0) := '0';

        imm_i_v(31 downto 12) := (others => I_instr(31));
        imm_i_v(11 downto 0) := I_instr(31 downto 20);
        
        imm_b_v(31 downto 13) := (others => I_instr(31));
        imm_b_v(12 downto 1) := I_instr(31) & I_instr(7) & I_instr(30 downto 25) & I_instr(11 downto 8);
        imm_b_v(0) := '0';

        imm_s_v(31 downto 12) := (others => I_instr(31));
        imm_s_v(11 downto 0) := I_instr(31 downto 25) & I_instr(11 downto 7);
        
        imm_shamt_v(31 downto 5) := (others => '0');
        imm_shamt_v(4 downto 0) := rs2_v;

        selrs1_v := to_integer(unsigned(rs1_v));
        selrs2_v := to_integer(unsigned(rs2_v));
        
        if I_areset = '1' then
            id_ex.pc <= (others => '0');
            -- synthesis translate_off
            if_id.instr_decode <= x"00000013"; -- 0x00000013 == NOP
            -- synthesis translate_on
            id_ex.rd <= (others => '0');
            id_ex.rs1 <= (others => '0');
            id_ex.rs2 <= (others => '0');
            id_ex.rd_en <= '1';
            id_ex.imm <= (others => '0');
            id_ex.isimm <= '0';
            id_ex.isunsigned <= '0';
            id_ex.alu_op <= alu_unknown;
            id_ex.pc_op <= pc_incr;
            id_ex.rs1data <= (others => '0');
            id_ex.rs2data <= (others => '0');
            id_ex.md_start <= '0';
            id_ex.md_op <= (others => '0');
            id_ex.memaccess <= memaccess_nop;
            id_ex.memsize <= memsize_unknown;
            id_ex.csr_op <= csr_nop;
            O_csr_addr <= (others => '0');
            O_csr_immrs1 <= (others => '0');
            control.ecall_request <= '0';
            control.ebreak_request <= '0';
            control.mret_request <= '0';
            O_illegal_instruction_error <= '0';
            control.reg0_write_once <= '0';
        elsif rising_edge(I_clk) then
            -- If there is a trap request ...
            if I_interrupt_request /= irq_none then
                id_ex.alu_op <= alu_nop;
                id_ex.rd <= (others => '0');
                id_ex.rd_en <= '0';
                id_ex.pc_op <= pc_load_mtvec;
                control.ecall_request <= '0';
                control.ebreak_request <= '0';
            -- We need to stall the operation
            elsif control.stall = '1' then
                -- Set id_ex.md_start to 0. It is already registered.
                id_ex.md_start <= '0';
                -- If the MD unit is ready and we are still doing MD operation,
                -- load the data in the selected register. MD operation can be
                -- interrupted by an interrupt.
                if md.ready = '1' then
                    id_ex.pc_op <= pc_incr;
                    id_ex.rd_en <= '1';
                end if;
            else
                id_ex.pc <= if_id.pc;
                -- synthesis translate_off
                if control.flush = '1' or control.state = state_boot0 or control.state = state_intr or control.state = state_intr2 then
                    if_id.instr_decode <= x"00000013";
                else
                    if_id.instr_decode <= I_instr;
                end if;
                -- synthesis translate_on
                id_ex.rd <= rd_v;
                id_ex.rs1 <= rs1_v;
                id_ex.rs2 <= rs2_v;
                id_ex.rd_en <= '0';
                id_ex.imm <= (others => '0');
                id_ex.isimm <= '0';
                id_ex.isunsigned <= '0';
                id_ex.alu_op <= alu_nop;
                id_ex.pc_op <= pc_incr;
                id_ex.rs1data <= regs(selrs1_v);
                id_ex.rs2data <= regs(selrs2_v);
                id_ex.md_start <= '0';
                id_ex.md_op <= (others => '0');
                id_ex.memaccess <= memaccess_nop;
                id_ex.memsize <= memsize_unknown;
                id_ex.csr_op <= csr_nop;
                O_csr_addr <= (others => '0');
                O_csr_immrs1 <= (others => '0');
                control.ecall_request <= '0';
                control.ebreak_request <= '0';
                control.mret_request <= '0';
                O_illegal_instruction_error <= '0';
                control.reg0_write_once <= '1';
                
                if control.flush = '1' then
                    --alu_op <= alu_flush;
                    id_ex.alu_op <= alu_nop;
                else
                    case opcode_v is
                        -- LUI
                        when "0110111" =>
                            id_ex.alu_op <= alu_lui;
                            id_ex.rd_en <= '1';
                            id_ex.imm <= imm_u_v;
                            id_ex.isimm <= '1';
                        -- AUIPC
                        when "0010111" =>
                            id_ex.alu_op <= alu_auipc;
                            id_ex.rd_en <= '1';
                            id_ex.imm <= imm_u_v;
                            id_ex.isimm <= '1';
                        -- JAL
                        when "1101111" =>
                            id_ex.alu_op <= alu_jal_jalr;
                            id_ex.pc_op <= pc_loadoffset;
                            id_ex.rd_en <= '1';
                            id_ex.imm <= imm_j_v;
                        -- JALR
                        when "1100111" =>
                            if func3_v = "000" then
                                id_ex.alu_op <= alu_jal_jalr;
                                id_ex.pc_op <= pc_loadoffsetregister;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_i_v;
                            else
                                O_illegal_instruction_error <= '1';
                            end if;
                        -- Branches
                        when "1100011" =>
                            -- Set the registers to compare. Comparison is handled by the ALU.
                            id_ex.imm <= imm_b_v;
                            id_ex.pc_op <= pc_branch;
                            case func3_v is
                                when "000" => id_ex.alu_op <= alu_beq;
                                when "001" => id_ex.alu_op <= alu_bne;
                                when "100" => id_ex.alu_op <= alu_blt;
                                when "101" => id_ex.alu_op <= alu_bge;
                                when "110" => id_ex.alu_op <= alu_bltu; id_ex.isunsigned <= '1';
                                when "111" => id_ex.alu_op <= alu_bgeu; id_ex.isunsigned <= '1';
                                when others =>
                                    -- Reset defaults
                                    id_ex.pc_op <= pc_incr;
                                    O_illegal_instruction_error <= '1';
                            end case;

                        -- Arithmetic/logic register/immediate
                        when "0010011" =>
                            -- ADDI
                            if func3_v = "000" then
                                id_ex.alu_op <= alu_addi;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_i_v;
                                id_ex.isimm <= '1';
                            -- SLTI
                            elsif func3_v = "010" then
                                id_ex.alu_op <= alu_slti;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_i_v;
                                id_ex.isimm <= '1';
                            -- SLTIU
                            elsif func3_v = "011" then
                                id_ex.alu_op <= alu_sltiu;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_i_v;
                                id_ex.isimm <= '1';
                                id_ex.isunsigned <= '1';
                            -- XORI
                            elsif func3_v = "100" then
                                id_ex.alu_op <= alu_xori;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_i_v;
                                id_ex.isimm <= '1';
                            -- ORI
                            elsif func3_v = "110" then
                                id_ex.alu_op <= alu_ori;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_i_v;
                                id_ex.isimm <= '1';
                            -- ANDI
                            elsif func3_v = "111" then
                                id_ex.alu_op <= alu_andi;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_i_v;
                                id_ex.isimm <= '1';
                            -- SLLI
                            elsif func3_v = "001" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_slli;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_shamt_v;
                                id_ex.isimm <= '1';
                            -- SRLI
                            elsif func3_v = "101" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_srli;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_shamt_v;
                                id_ex.isimm <= '1';
                            -- SRAI
                            elsif func3_v = "101" and func7_v = "0100000" then
                                id_ex.alu_op <= alu_srai;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_shamt_v;
                                id_ex.isimm <= '1';
                            else
                                O_illegal_instruction_error <= '1';
                            end if;

                        -- Arithmetic/logic register/register
                        when "0110011" =>
                            -- ADD
                            if func3_v = "000" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_add;
                                id_ex.rd_en <= '1';
                            -- SUB
                            elsif func3_v = "000" and func7_v = "0100000" then
                                id_ex.alu_op <= alu_sub;
                                id_ex.rd_en <= '1';
                            -- SLL
                            elsif func3_v = "001" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_sll; 
                                id_ex.rd_en <= '1';
                            -- SLT
                            elsif func3_v = "010" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_slt; 
                                id_ex.rd_en <= '1';
                            -- SLTU
                            elsif func3_v = "011" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_sltu; 
                                id_ex.rd_en <= '1';
                                id_ex.isunsigned <= '1';
                            -- XOR
                            elsif func3_v = "100" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_xor; 
                                id_ex.rd_en <= '1';
                            -- SRL
                            elsif func3_v = "101" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_srl; 
                                id_ex.rd_en <= '1';
                            -- SRA
                            elsif func3_v = "101" and func7_v = "0100000" then
                                id_ex.alu_op <= alu_sra; 
                                id_ex.rd_en <= '1';
                            -- OR
                            elsif func3_v = "110" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_or;
                                id_ex.rd_en <= '1';
                            -- AND
                            elsif func3_v = "111" and func7_v = "0000000" then
                                id_ex.alu_op <= alu_and;
                                id_ex.rd_en <= '1';
                            -- Multiply, divide, remainder
                            elsif func7_v = "0000001" then
                                -- Set operation to multiply or divide/remainder
                                -- func3 contains the real operation
                                case func3_v(2) is
                                    when '0' => id_ex.alu_op <= alu_multiply;
                                    when '1' => id_ex.alu_op <= alu_divrem;
                                    when others => null;
                                end case;
                                -- Hold the PC
                                id_ex.pc_op <= pc_hold;
                                -- func3 contains the function
                                id_ex.md_op <= func3_v;
                                -- Start multiply/divide/remainder
                                id_ex.md_start <= '1';
                            else
                                O_illegal_instruction_error <= '1';
                            end if;

                        -- S(W|H|B)
                        when "0100011" =>
                            case func3_v is
                                -- Store byte (no sign extension or zero extension)
                                when "000" =>
                                    id_ex.alu_op <= alu_sb;
                                    id_ex.memaccess <= memaccess_write;
                                    id_ex.memsize <= memsize_byte;
                                    id_ex.imm <= imm_s_v;
                                -- Store halfword (no sign extension or zero extension)
                                when "001" =>
                                    id_ex.alu_op <= alu_sh;
                                    id_ex.memaccess <= memaccess_write;
                                    id_ex.memsize <= memsize_halfword;
                                    id_ex.imm <= imm_s_v;
                                -- Store word (no sign extension or zero extension)
                                when "010" =>
                                    id_ex.alu_op <= alu_sw;
                                    id_ex.memaccess <= memaccess_write;
                                    id_ex.memsize <= memsize_word;
                                    id_ex.imm <= imm_s_v;
                                when others =>
                                    O_illegal_instruction_error <= '1';
                            end case;
                        -- L{W|H|B|HU|BU}
                        -- Data from memory is routed through the ALU
                        when "0000011" =>
                            case func3_v is
                                -- LB
                                when "000" =>
                                    id_ex.alu_op <= alu_lb;
                                    id_ex.rd_en <= '1';
                                    id_ex.memaccess <= memaccess_read;
                                    id_ex.memsize <= memsize_byte;
                                    id_ex.imm <= imm_i_v;
                                -- LH
                                when "001" =>
                                    id_ex.alu_op <= alu_lh;
                                    id_ex.rd_en <= '1';
                                    id_ex.memaccess <= memaccess_read;
                                    id_ex.memsize <= memsize_halfword;
                                    id_ex.imm <= imm_i_v;
                                -- LW
                                when "010" =>
                                    id_ex.alu_op <= alu_lw;
                                    id_ex.rd_en <= '1';
                                    id_ex.memaccess <= memaccess_read;
                                    id_ex.memsize <= memsize_word;
                                    id_ex.imm <= imm_i_v;
                                -- LBU
                                when "100" =>
                                    id_ex.alu_op <= alu_lbu;
                                    id_ex.rd_en <= '1';
                                    id_ex.memaccess <= memaccess_read;
                                    id_ex.memsize <= memsize_byte;
                                    id_ex.imm <= imm_i_v;
                                -- LHU
                                when "101" =>
                                    id_ex.alu_op <= alu_lhu;
                                    id_ex.rd_en <= '1';
                                    id_ex.memaccess <= memaccess_read;
                                    id_ex.memsize <= memsize_halfword;
                                    id_ex.imm <= imm_i_v;
                                when others =>
                                     O_illegal_instruction_error <= '1';
                            end case;

                        -- CSR{}, {ECALL, EBREAK, MRET, WFI}
                        when "1110011" =>
                            case func3_v is
                                when "000" =>
                                    -- ECALL/EBREAK/MRET/WFI
                                    if I_instr(31 downto 20) = "000000000000" then
                                        -- ECALL
                                        control.ecall_request <= '1';
                                        id_ex.alu_op <= alu_trap;
                                        id_ex.pc_op <= pc_hold;
                                    elsif I_instr(31 downto 20) = "000000000001" then
                                        -- EBREAK
                                        control.ebreak_request <= '1';
                                        id_ex.alu_op <= alu_trap;
                                        id_ex.pc_op <= pc_hold;
                                    elsif I_instr(31 downto 20) = "001100000010" then
                                        -- MRET
                                        id_ex.alu_op <= alu_mret;
                                        control.mret_request <= '1';
                                        id_ex.pc_op <= pc_load_mepc;
                                    elsif I_instr(31 downto 20) = "000100000101" then
                                        -- WFI, skip for now
                                        null;
                                    else
                                        O_illegal_instruction_error <= '1';
                                    end if;
                                when "001" =>
                                    id_ex.alu_op <= alu_csr;
                                    id_ex.csr_op <= csr_rw;
                                    --id_ex.rd <= rd_v;
                                    id_ex.rd_en <= '1';
                                    O_csr_addr <= imm_i_v(11 downto 0);
                                    O_csr_immrs1 <= rs1_v; -- RS1
                                when "010" =>
                                    id_ex.alu_op <= alu_csr;
                                    id_ex.csr_op <= csr_rs;
                                    --id_ex.rd <= rd_v;
                                    id_ex.rd_en <= '1';
                                    O_csr_addr <= imm_i_v(11 downto 0);
                                    O_csr_immrs1 <= rs1_v; -- RS1
                                when "011" =>
                                    id_ex.alu_op <= alu_csr;
                                    id_ex.csr_op <= csr_rc;
                                    --id_ex.rd <= rd_v;
                                    id_ex.rd_en <= '1';
                                    O_csr_addr <= imm_i_v(11 downto 0);
                                    O_csr_immrs1 <= rs1_v; -- RS1
                                when "101" =>
                                    id_ex.alu_op <= alu_csr;
                                    id_ex.csr_op <= csr_rwi;
                                    --id_ex.rd <= rd_v;
                                    id_ex.rd_en <= '1';
                                    O_csr_addr <= imm_i_v(11 downto 0);
                                    O_csr_immrs1 <= rs1_v; -- imm
                                when "110" =>
                                    id_ex.alu_op <= alu_csr;
                                    id_ex.csr_op <= csr_rsi;
                                    --id_ex.rd <= rd_v;
                                    id_ex.rd_en <= '1';
                                    --id_ex.rs1 <= rs1_v;
                                    O_csr_addr <= imm_i_v(11 downto 0);
                                    O_csr_immrs1 <= rs1_v; -- imm
                                when "111" =>
                                    id_ex.alu_op <= alu_csr;
                                    id_ex.csr_op <= csr_rci;
                                    --id_ex.rd <= rd_v;
                                    id_ex.rd_en <= '1';
                                    --id_ex.rs1 <= rs1_v;
                                    O_csr_addr <= imm_i_v(11 downto 0);
                                    O_csr_immrs1 <= rs1_v; -- imm
                                when others =>
                                    O_illegal_instruction_error <= '1';
                            end case;
                            
                        -- Illegal instruction or not implemented
                        when others =>
                            O_illegal_instruction_error <= '1';
                    end case;
                    
                    -- When the registers use onboard RAM, the registers
                    -- cannot be reset. In that case, we must write register
                    -- x0 (zero) with 0x00000000. This needs to be done only
                    -- once when the processor starts up. After that, register
                    -- x0 is not written anymore. When the processor starts,
                    -- it executes an `alu_nop`, which sets the result to 0 so
                    -- register x0 is written with 0x00000000.
                    if control.reg0_write_once = '0' and HAVE_REGISTERS_IN_RAM then
                        id_ex.rd_en <= '1';
                    elsif rd_v = "00000" then
                        id_ex.rd_en <= '0';
                    end if;
               end if; -- flush
            end if; -- stall
        end if; -- rising_edge
            
    end process;

    -- Generate register in RAM
    gen_regs_ram: if HAVE_REGISTERS_IN_RAM generate
        -- Register: exec & retire
        -- Do NOT include a reset, otherwise registers will be in ALM flip-flops
        -- Do NOT set x0 to all zero bits
        process (I_clk, I_areset, id_ex.rd, I_instr) is
        variable selrd_v : integer range 0 to NUMBER_OF_REGISTERS-1;
        begin
            selrd_v := to_integer(unsigned(id_ex.rd));
            
            if rising_edge(I_clk) then
                if control.stall = '0' and id_ex.rd_en = '1' and I_interrupt_request = irq_none then
                    regs(selrd_v) <= id_ex.result;
                end if;
            end if;
        end process;
    end generate;

    -- Generate registers in ALM flip-flops
    gen_regs_ram_not: if not HAVE_REGISTERS_IN_RAM generate
        -- Register: exec & retire
        process (I_clk, I_areset, id_ex.rd, I_instr) is
        variable selrd_v : integer range 0 to NUMBER_OF_REGISTERS-1;
        begin
            selrd_v := to_integer(unsigned(id_ex.rd));

            if I_areset = '1' then
                regs <= (others => (others => '0'));
            elsif rising_edge(I_clk) then
                if control.stall = '0' and id_ex.rd_en = '1' and I_interrupt_request = irq_none then
                    regs(selrd_v) <= id_ex.result;
                end if;
                regs(0) <= (others => '0');
            end if;
        end process;
    end generate;

    --
    -- The execute block
    -- Contains the ALU, the MD unit and result retire unit
    --
    
    -- ALU
    process (id_ex, control, ex_wb, md, I_memdatain, I_csr_datain) is
    variable a_v, b_v, r_v, imm_v : data_type;
    variable al_v, bl_v : std_logic_vector(32 downto 0);
    variable signs_v : data_type;
    constant zeros_v : data_type := (others => '0');
    variable cmpeq_v, cmplt_v : std_logic;
    begin
    
        -- Check if forwarding result is needed
        if control.forwarda = '1' then
            a_v := ex_wb.rddata;
        else
            a_v := id_ex.rs1data;
        end if;
            
        -- Check if forwarding result is needed
        if id_ex.isimm = '1' then
            b_v := id_ex.imm;
        elsif control.forwardb = '1' then
            b_v := ex_wb.rddata;
        else
            b_v := id_ex.rs2data;
        end if;
        
        -- Create a zero or signed extended version of the operands.
        -- For signed operations, the operands are sign extended and
        -- will compare as normal signed operands. For unsigned
        -- operation (SLTU, SLTIU, BLTU and BGEU), the operands are
        -- zero extended and will compare as unsigned operands even
        -- though the compare is signed.
        al_v := (a_v(31) and (not id_ex.isunsigned)) & a_v;
        bl_v := (b_v(31) and (not id_ex.isunsigned)) & b_v;
        
        -- Compare equal
        if a_v = b_v then
            cmpeq_v := '1';
        else
            cmpeq_v := '0';
        end if;
        -- Compare less-than
        if signed(al_v) < signed(bl_v) then
            cmplt_v := '1';
        else
            cmplt_v := '0';
        end if;
        
        r_v := (others => '0');
        
        control.penalty <= '0';
        control.select_pc <= '0';
        
        case id_ex.alu_op is
            -- No operation
            when alu_nop | alu_unknown =>
                null;
            when alu_sw | alu_sh | alu_sb | alu_trap =>
                control.select_pc <= '1';
            when alu_mret =>
                control.penalty <= '1';
                
            when alu_add | alu_addi =>
                r_v := std_logic_vector(unsigned(a_v) + unsigned(b_v));
                control.select_pc <= '1';
            when alu_sub =>
                r_v := std_logic_vector(unsigned(a_v) - unsigned(b_v));
                control.select_pc <= '1';
            when alu_and | alu_andi =>
                r_v := a_v and b_v;
                control.select_pc <= '1';
            when alu_or | alu_ori =>
                r_v := a_v or b_v;
                control.select_pc <= '1';
            when alu_xor | alu_xori =>
                r_v := a_v xor b_v;
                control.select_pc <= '1';
                
            -- Test register & immediate signed/unsigned
            when alu_slt | alu_slti =>
                r_v := (others => '0');
                r_v(0) := cmplt_v;
                control.select_pc <= '1';
            when alu_sltu | alu_sltiu =>
                r_v := (others => '0');
                r_v(0) := cmplt_v;
                control.select_pc <= '1';
                
            -- Shifts et al
            when alu_sll | alu_slli =>
                if b_v(4) = '1' then
                    a_v := a_v(15 downto 0) & zeros_v(15 downto 0);
                end if;
                if b_v(3) = '1' then
                    a_v := a_v(23 downto 0) & zeros_v(7 downto 0);
                end if;
                if b_v(2) = '1' then
                    a_v := a_v(27 downto 0) & zeros_v(3 downto 0);
                end if;
                if b_v(1) = '1' then
                    a_v := a_v(29 downto 0) & zeros_v(1 downto 0);
                end if;
                if b_v(0) = '1' then
                    a_v := a_v(30 downto 0) & zeros_v(0 downto 0);
                end if;
                r_v := a_v;
                control.select_pc <= '1';
            when alu_srl | alu_srli =>
                if b_v(4) = '1' then
                    a_v := zeros_v(15 downto 0) & a_v(31 downto 16);
                end if;
                if b_v(3) = '1' then
                    a_v := zeros_v(7 downto 0) & a_v(31 downto 8);
                end if;
                if b_v(2) = '1' then
                    a_v := zeros_v(3 downto 0) & a_v(31 downto 4);
                end if;
                if b_v(1) = '1' then
                    a_v := zeros_v(1 downto 0) & a_v(31 downto 2);
                end if;
                if b_v(0) = '1' then
                    a_v := zeros_v(0 downto 0) & a_v(31 downto 1);
                end if;
                r_v := a_v;
                control.select_pc <= '1';
            when alu_sra | alu_srai =>
                signs_v := (others => a_v(31));
                if b_v(4) = '1' then
                    a_v := signs_v(15 downto 0) & a_v(31 downto 16);
                end if;
                if b_v(3) = '1' then
                    a_v := signs_v(7 downto 0) & a_v(31 downto 8);
                end if;
                if b_v(2) = '1' then
                    a_v := signs_v(3 downto 0) & a_v(31 downto 4);
                end if;
                if b_v(1) = '1' then
                    a_v := signs_v(1 downto 0) & a_v(31 downto 2);
                end if;
                if b_v(0) = '1' then
                    a_v := signs_v(0 downto 0) & a_v(31 downto 1);
                end if;
                r_v := a_v;
                control.select_pc <= '1';
                
            -- Loads etc
            when alu_lui =>
                r_v := b_v;
                control.select_pc <= '1';
            when alu_auipc =>
                r_v := std_logic_vector(unsigned(id_ex.pc) + unsigned(b_v)) ;
                control.select_pc <= '1';
            when alu_lw =>
                r_v := I_memdatain;
                control.select_pc <= '1';
            when alu_lh =>
                r_v := (others => I_memdatain(15));
                r_v(15 downto 0) := I_memdatain(15 downto 0);
                control.select_pc <= '1';
            when alu_lhu =>
                r_v := (others => '0');
                r_v(15 downto 0) := I_memdatain(15 downto 0);
                control.select_pc <= '1';
            when alu_lb =>
                r_v := (others => I_memdatain(7));
                r_v(7 downto 0) := I_memdatain(7 downto 0);
                control.select_pc <= '1';
            when alu_lbu =>
                r_v := (others => '0');
                r_v(7 downto 0) := I_memdatain(7 downto 0);
                control.select_pc <= '1';
                
            -- Jumps and calls
            when alu_jal_jalr =>
                r_v := std_logic_vector(unsigned(id_ex.pc) + 4);
                control.penalty <= '1';
                control.select_pc <= '1';
                
            -- Branches
            when alu_beq =>
                r_v := (others => '0');
                r_v(0) := cmpeq_v;
                control.penalty <= cmpeq_v;
                control.select_pc <= '1';
            when alu_bne =>
                r_v := (others => '0');
                r_v(0) := not cmpeq_v;
                control.penalty <= not cmpeq_v;
                control.select_pc <= '1';
            when alu_blt | alu_bltu =>
                r_v := (others => '0');
                r_v(0) := cmplt_v;
                control.penalty <= cmplt_v;
                control.select_pc <= '1';
            when alu_bge | alu_bgeu =>
                r_v := (others => '0');
                r_v(0) := not cmplt_v;
                control.penalty <= not cmplt_v;
                control.select_pc <= '1';
                
            -- Pass data from CSR
            when alu_csr =>
                r_v := I_csr_datain;
                control.select_pc <= '1';
                
            -- Pass data from multiplier
            when alu_multiply =>
                r_v := md.mul;
                control.select_pc <= '1';
                
            -- Pass data from divider
            when alu_divrem =>
                r_v := md.div;
                control.select_pc <= '1';
                
            --when others =>
            --    r := (others => '0');
        end case;
        
        -- The result is not clocked.
        id_ex.result <= r_v;
    end process;

    -- The MD unit, can be omitted by setting HAVE_MULDIV to false
    muldivgen: if HAVE_MULDIV generate
        -- Multiplication Unit
        -- Check start of multiplication and load registers
        process (I_clk, I_areset, control, ex_wb, id_ex) is
        variable a_v, b_v : data_type;
        begin
            -- Check if forwarding result is needed
            if control.forwarda = '1' then
                a_v := (ex_wb.rddata);
            else
                a_v := (id_ex.rs1data);
            end if;
                
            if control.forwardb = '1' then
                b_v := (ex_wb.rddata);
            else
                b_v := (id_ex.rs2data);
            end if;
        
            if I_areset = '1' then
                md.rdata_a <= (others => '0');
                md.rdata_b <= (others => '0');
                md.mul_running <= '0';
            elsif rising_edge(I_clk) then
                -- Clock in the multiplicand and multiplier
                -- In the Cyclone V, these are embedded registers
                -- in the DSP units.
                if id_ex.md_start = '1' then
                    if id_ex.md_op(1) = '1' then
                        if id_ex.md_op(0) = '1' then
                            md.rdata_a <= '0' & unsigned(a_v);
                        else
                            md.rdata_a <= a_v(31) & unsigned(a_v);
                        end if;
                        md.rdata_b <= '0' & unsigned(b_v);
                    else
                        md.rdata_a <= a_v(31) & unsigned(a_v);
                        md.rdata_b <= b_v(31) & unsigned(b_v);
                    end if;
                end if;
                -- Only start when start seen and multiply
                md.mul_running <= id_ex.md_start and not id_ex.md_op(2);
            end if;
        end process;

        -- Do the multiplication
        process(I_clk, I_areset) is
        begin
            if I_areset = '1' then
                md.mul_rd_int <= (others => '0');
                md.mul_ready <= '0';
            elsif rising_edge (I_clk) then
                -- Do the multiplication and store in embedded registers
                md.mul_rd_int <= signed(md.rdata_a) * signed(md.rdata_b);
                md.mul_ready <= md.mul_running;
            end if;
        end process;
        
        -- Output multiplier result
        process (md, id_ex) is
        begin
            if id_ex.md_op(1) = '1' or id_ex.md_op(0) = '1' then
                md.mul <= std_logic_vector(md.mul_rd_int(63 downto 32));
            else
                md.mul <= std_logic_vector(md.mul_rd_int(31 downto 0));
            end if;
        end process;

        fast_div: if FAST_DIVIDE generate
        -- The main divider process. The divider retires 2 bits
        -- at a time, hence 16 cycles are needed. We use a
        -- poor man's radix-4 subtraction unit. It is not the
        -- fastest hardware but the easiest to follow. Consider
        -- a SRT radix-4 divider.
        process (I_clk, I_areset, control, ex_wb, id_ex) is
        variable a_v, b_v : data_type;
        variable div_running_v : std_logic;  
        variable count_v : integer range 0 to 16;
        begin 
            -- Check if forwarding result is needed
            if control.forwarda = '1' then
                a_v := (ex_wb.rddata);
            else
                a_v := (id_ex.rs1data);
            end if;

            if control.forwardb = '1' then
                b_v := (ex_wb.rddata);
            else
                b_v := (id_ex.rs2data);
            end if;

            if I_areset = '1' then
                -- Reset everything
                count_v := 0;
                md_buf1 <= (others => '0');
                md_buf2 <= (others => '0');
                md.divisor1 <= (others => '0');
                md.divisor2 <= (others => '0');
                md.divisor3 <= (others => '0');
                div_running_v := '0';
                md.div_ready <= '0';
                md.outsign <= '0';
            elsif rising_edge(I_clk) then 
                -- If start and dividing...
                md.div_ready <= '0';
                if id_ex.md_start = '1' and id_ex.md_op(2) = '1' then
                    -- Signal that we are running
                    div_running_v := '1';
                    -- For restarting the division
                    count_v := 0;
                end if;
                if div_running_v = '1' then
                    case count_v is 
                        when 0 =>
                            md_buf1 <= (others => '0');
                            -- If signed divide, check for negative
                            -- value and make it positive
                            if id_ex.md_op(0) = '0' and a_v(31) = '1' then
                                md_buf2 <= unsigned(not a_v) + 1;
                            else
                                md_buf2 <= unsigned(a_v);
                            end if;
                            -- Load the divisor x1, divisor x2 and divisor x3
                            if id_ex.md_op(0) = '0' and b_v(31) = '1' then
                                md.divisor1 <= "00" & (unsigned(not b_v) + 1);
                                md.divisor2 <= ("0" & (unsigned(not b_v) + 1) & "0");
                                md.divisor3 <= ("0" & (unsigned(not b_v) + 1) & "0") + ("00" & (unsigned(not b_v) + 1));
                            else
                                md.divisor1 <= ("00" & unsigned(b_v));
                                md.divisor2 <= ("0" & unsigned(b_v) & "0");
                                md.divisor3 <= ("0" & unsigned(b_v) & "0") + ("00" & unsigned(b_v));
                            end if;
                            count_v := count_v + 1;
                            md.div_ready <= '0';
                            -- Determine the sign of the quotient and remainder
                            if (id_ex.md_op(0) = '0' and id_ex.md_op(1) = '0' and (a_v(31) /= b_v(31)) and b_v /= all_zeros) or (id_ex.md_op(0) = '0' and id_ex.md_op(1) = '1' and a_v(31) = '1') then
                                md.outsign <= '1';
                            else
                                md.outsign <= '0';
                            end if;
                        when others =>
                            -- Do the divide
                            -- First check is divisor x3 can be subtracted...
                            if md.buf(63 downto 30) >= md.divisor3 then
                                md_buf1(63 downto 32) <= md.buf(61 downto 30) - md.divisor3(31 downto 0);
                                md_buf2 <= md_buf2(29 downto 0) & "11";
                            -- Then check is divisor x2 can be subtracted...
                            elsif md.buf(63 downto 30) >= md.divisor2 then
                                md_buf1(63 downto 32) <= md.buf(61 downto 30) - md.divisor2(31 downto 0);
                                md_buf2 <= md_buf2(29 downto 0) & "10";
                            -- Then check is divisor x1 can be subtracted...
                            elsif md.buf(63 downto 30) >= md.divisor1 then
                                md_buf1(63 downto 32) <= md.buf(61 downto 30) - md.divisor1(31 downto 0);
                                md_buf2 <= md_buf2(29 downto 0) & "01";
                            -- Else no subtraction can be performed.
                            else
                                -- Shift in 0 (00)
                                md.buf <= md.buf(61 downto 0) & "00";
                            end if;
                            -- Do this 16 times (32 bit/2 bits at a time, output in last cycle)
                            if count_v /= 16 then
                                -- Signal ready one clock before
                                if count_v = 15 then
                                    md.div_ready <= '1';
                                end if;
                                count_v := count_v + 1;
                            else
                                -- Ready, show the result
                                count_v := 0;
                                div_running_v := '0';
                            end if;
                    end case;
                end if;
            end if;
-- synthesis translate_off
            md.count <= count_v;
-- synthesis translate_on
        end process;
        end generate;
        
        fast_div_not: if not FAST_DIVIDE generate
        -- Division unit, retires one bit at a time
        process (I_clk, I_areset, control, ex_wb, id_ex) is
        variable a_v, b_v : data_type;
        variable div_running_v : std_logic;  
        variable count_v : integer range 0 to 32;
        begin
            -- Check if forwarding result is needed
            if control.forwarda = '1' then
                a_v := (ex_wb.rddata);
            else
                a_v := (id_ex.rs1data);
            end if;
                
            if control.forwardb = '1' then
                b_v := (ex_wb.rddata);
            else
                b_v := (id_ex.rs2data);
            end if;
            
            if I_areset = '1' then
                -- Reset everything
                count_v := 0;
                md_buf1 <= (others => '0');
                md_buf2 <= (others => '0');
                md.divisor <= (others => '0');
                div_running_v := '0';
                md.div_ready <= '0';
                md.outsign <= '0';
            elsif rising_edge(I_clk) then 
                -- If start and dividing...
                md.div_ready <= '0';
                if id_ex.md_start = '1' and id_ex.md_op(2) = '1' then
                    div_running_v := '1';
                    count_v := 0;
                end if;
                if div_running_v = '1' then
                    case count_v is 
                        when 0 => 
                            md_buf1 <= (others => '0');
                            -- If signed divide, check for negative
                            -- value and make it positive
                            if id_ex.md_op(0) = '0' and a_v(31) = '1' then
                                md_buf2 <= unsigned(not a_v) + 1;
                            else
                                md_buf2 <= unsigned(a_v);
                            end if;
                            if id_ex.md_op(0) = '0' and b_v(31) = '1' then
                                md.divisor <= unsigned(not b_v) + 1;
                            else
                                md.divisor <= unsigned(b_v); 
                            end if;
                            count_v := count_v + 1; 
                            md.div_ready <= '0';
                            -- Determine the result sign
                            if (id_ex.md_op(0) = '0' and id_ex.md_op(1) = '0' and (a_v(31) /= b_v(31)) and b_v /= all_zeros) or (id_ex.md_op(0) = '0' and id_ex.md_op(1) = '1' and a_v(31) = '1') then
                                md.outsign <= '1';
                            else
                                md.outsign <= '0';
                            end if;

                        when others =>
                            -- Do the division
                            if md.buf(62 downto 31) >= md.divisor then 
                                md_buf1 <= '0' & (md.buf(61 downto 31) - md.divisor(30 downto 0)); 
                                md_buf2 <= md_buf2(30 downto 0) & '1'; 
                            else 
                                md.buf <= md.buf(62 downto 0) & '0'; 
                            end if;
                            -- Do this 32 times, last one outputs the result
                            if count_v /= 32 then 
                                -- Signal ready one clock before
                                if count_v = 31 then
                                    md.div_ready <= '1';
                                end if;
                                count_v := count_v + 1;
                            else
                                -- Signal ready
                                count_v := 0;
                                div_running_v := '0';
                            end if; 
                    end case; 
                end if; 
            end if;
-- synthesis translate_off
            -- Only to view in simulator
            md.count <= count_v;
-- synthesis translate_on
        end process;
        end generate;
        
        -- Select the correct signedness of the results
        process (md.outsign, md_buf2, md_buf1) is
        begin
            if md.outsign = '1' then
                md.quotient <= not md_buf2 + 1;
                md.remainder <= not md_buf1 + 1;
            else
                md.quotient <= md_buf2;
                md.remainder <= md_buf1; 
            end if;
        end process;

        -- Select the divider output
        md.div <= std_logic_vector(md.remainder) when id_ex.md_op(1) = '1' else std_logic_vector(md.quotient);
        
        -- Signal that we are ready
        md.ready <= md.div_ready or md.mul_ready;
        
    end generate; -- generate MD unit
    
    -- If we don't have an MD unit, set some signals
    -- to default values. The synthesizer will remove the hardware.
    muldivgennot: if not HAVE_MULDIV generate
        md.ready <= '0';
        md.mul <= (others => '0');
        md.div <= (others => '0');
    end generate;

    -- Save a copy of the result for data forwarding
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            ex_wb.rd <= (others => '0');
            ex_wb.rd_en <= '0';
            ex_wb.rddata <= (others => '0');
        elsif rising_edge(I_clk) then
            if control.stall = '1' then
                null;
            else
                ex_wb.rd_en <= id_ex.rd_en;
                if id_ex.rd_en = '1' and I_interrupt_request = irq_none then
                    ex_wb.rddata <= id_ex.result;
                    ex_wb.rd <= id_ex.rd;
                end if;
            end if;
        end if;       
    end process;


    --
    -- Memory interface block
    -- Interface to the memory and the CSR
    --

    -- Disable the bus when flushing
    O_memaccess <= memaccess_nop when control.flush = '1' else id_ex.memaccess;
    O_memsize <= id_ex.memsize;
    O_csr_op <= csr_nop when control.flush = '1' else id_ex.csr_op;
    
    -- This is the interface between the core and the memory (ROM, RAM, I/O)
    -- Memory access type and size are computed in the instruction decoding unit
    process (control, id_ex, ex_wb) is
    variable address_v : unsigned(31 downto 0);
    begin
        -- Check if we need forward or not
        if control.forwarda = '1' then
            address_v := unsigned(ex_wb.rddata);
        else
            address_v := unsigned(id_ex.rs1data);
        end if;
        address_v := address_v + unsigned(id_ex.imm);

        -- Data out to memory
        O_memaddress <= std_logic_vector(address_v);
        
        if control.forwardb = '1' then
            O_memdataout <= ex_wb.rddata;
        else
            O_memdataout <= id_ex.rs2data;
        end if;
        
    end process;
    
    -- Set the address of the CSR register
    process (control.forwarda, id_ex.rs1data, ex_wb.rddata) is
    begin
        -- Check if we need forward or not
        if control.forwarda = '1' then
            O_csr_dataout <= ex_wb.rddata;
        else
            O_csr_dataout <= id_ex.rs1data;
        end if;
       
    end process;
    
end architecture rtl;
