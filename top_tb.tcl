# Vivado simulation TCL script for top_tb
# This script is automatically sourced when simulation starts

# Add all top-level signals
add_wave /top_tb/*

# Add memory module signals
add_wave /top_tb/dut/mem_u/*
add_wave /top_tb/dut/mem_u/mem_array

# Add physical register file
add_wave /top_tb/dut/phys_reg_file

# Add rename table
add_wave /top_tb/dut/rename_table

# Add ROB entries
add_wave /top_tb/dut/rob

# Add issue queues
add_wave /top_tb/dut/int_iq
add_wave /top_tb/dut/mem_iq

# Configure wave window
set curr_wave [current_wave_config]
if { [string length $curr_wave] > 0 } {
    set_property needs_save false [current_wave_config]
}

# Run for 1000ns
run 1000ns
