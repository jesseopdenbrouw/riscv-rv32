#################################################################################################
# tb_riscv.do - QuestaSim/ModelSim command file for simulation                                  #
# ********************************************************************************************* #
# This file is part of the THUAS RISCV RV32 Project                                             #
# ********************************************************************************************* #
# BSD 3-Clause License                                                                          #
#                                                                                               #
# Copyright (c) 2023, Jesse op den Brouw. All rights reserved.                                  #
#                                                                                               #
# Redistribution and use in source and binary forms, with or without modification, are          #
# permitted provided that the following conditions are met:                                     #
#                                                                                               #
# 1. Redistributions of source code must retain the above copyright notice, this list of        #
#    conditions and the following disclaimer.                                                   #
#                                                                                               #
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
#    conditions and the following disclaimer in the documentation and/or other materials        #
#    provided with the distribution.                                                            #
#                                                                                               #
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
#    endorse or promote products derived from this software without specific prior written      #
#    permission.                                                                                #
#                                                                                               #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
# OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
# ********************************************************************************************* #
# https:/github.com/jesseopdenbrouw/riscv-mrv32                                                 #
#################################################################################################

# Transcript on
transcript on

# Recreate work library
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

# Find out if we're started through Quartus or by hand
# (or by using an exec in the Tcl window in Quartus).
# Quartus has the annoying property that it will start
# Modelsim from a directory called "simulation/modelsim".
# The design and the testbench are located in the project
# root, so we've to compensate for that.
if [ string match "*simulation/modelsim" [pwd] ] { 
	set prefix "../../"
	puts "Running Modelsim from Quartus..."
} else {
	set prefix ""
	puts "Running Modelsim..."
}

# Compile the VHDL description and testbench,
# please note that the design and its testbench are located
# in the project root, but the simulator start in directory
# <project_root>/simulation/modelsim, so we have to compensate
# for that.
vcom -93 -work work ${prefix}processor_common.vhd
vcom -93 -work work ${prefix}processor_common_rom.vhd
vcom -93 -work work ${prefix}rom.vhd
vcom -93 -work work ${prefix}bootloader.vhd
vcom -93 -work work ${prefix}ram.vhd
vcom -93 -work work ${prefix}io.vhd
vcom -93 -work work ${prefix}csr.vhd
vcom -93 -work work ${prefix}lic.vhd
vcom -93 -work work ${prefix}core.vhd
vcom -93 -work work ${prefix}address_decode.vhd
vcom -93 -work work ${prefix}instr_router.vhd
vcom -93 -work work ${prefix}riscv.vhd
vcom -93 -work work ${prefix}tb_riscv.vhd

# Start the simulator
vsim -t 1ns -L rtl_work -L work -voptargs="+acc" tb_riscv

# Log all signals in the design, good if the number
# of signals is small.
add log -r *

# Add all toplevel signals
# Add a number of signals of the simulated design
add wave -divider "Inputs"
add wave            -label clk clk
add wave            -label areset areset
add wave -radix hex -label gpioapin gpioapin
add wave -radix hex -label uart1rxd uart1rxd
add wave -divider "Outputs"
add wave -radix hex -label gpioapout gpioapout
add wave -radix hex -label uart1txd uart1txd
add wave            -label timer2oct timer2oct
add wave            -label timer2icoca timer2icoca
add wave            -label timer2icocb timer2icocb
add wave            -label timer2icocc timer2icocc
--add wave -radix hex -label O_pc_to_mepc dut/pc_to_mepc_int
add wave -divider "Core - Inputs & Outputs"
add wave            -label I_interrupts_request dut/core0/I_interrupt_request
add wave            -label O_csr_addr dut/core0/O_csr_addr
add wave            -label O_pc_to_mepc dut/core0/O_pc_to_mepc
add wave -divider "Core Internals - Control"
add wave            -label control dut/core0/control
add wave -divider "Core Internals - Instruction Fetch"
add wave            -label pc dut/core0/pc
add wave            -label if_id dut/core0/if_id
add wave -divider "Core Internals - Instruction Decode"
add wave            -label id_ex dut/core0/id_ex
add wave -divider "Core Internals - Execute & Write back"
add wave            -label ex_wb dut/core0/ex_wb
add wave -divider "Core Internals - Registers"
add wave            -label regs dut/core0/regs
add wave -divider "Core Internals - Execute MD"
add wave            -label md dut/core0/md
add wave -divider "Internals - Memory access"
add wave            -label memaccess dut/memaccess_int
add wave            -label memsize dut/memsize_int
add wave            -label memaddress dut/memaddress_int
add wave            -label memvma dut/memvma_int
add wave            -label waitfordata dut/waitfordata_int
add wave            -label read_access_granted dut/io0/read_access_granted
add wave            -label read_access_granted_ff dut/io0/read_access_granted_ff
add wave            -label read_access_granted_second_cycle dut/io0/read_access_granted_second_cycle
add wave            -label write_access_granted dut/io0/write_access_granted
add wave            -label O_dataout dut/io0/O_dataout
add wave -divider "Internals - CSR"
add wave -radix hex -label CSR dut/csr0/csr
add wave -divider "Internals - RAM"
add wave -radix hex -label ram dut/ram0/ram_alt
#add wave -divider "Internals - ROM"
#add wave -radix hex -label rom dut/rom
add wave -divider "Internals - IO"
add wave -radix hex -label io dut/io0/io

# Open Structure, Signals (waveform) and List window
view structure
#view list
view signals

# Disable NUMERIC STD Warnings
# This will speed up simulation considerably
# and prevents writing to the transcript file
set NumericStdNoWarnings 1

# Run simulation for xx us
run 4 us

# Fill up the waveform in the window
wave zoom full
