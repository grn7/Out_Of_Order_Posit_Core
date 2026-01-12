# Vivado xsim compilation script for Out of Order RISC-V Processor

# Create work library
set_property target_language Verilog [current_project]

# Compile files in correct order
# First compile the package definitions
xvlog -sv includes/defines.sv

# Then compile the source files
xvlog -sv src/bp.sv
xvlog -sv src/btb.sv
xvlog -sv src/commit.sv
xvlog -sv src/ID.sv
xvlog -sv src/IF.sv
xvlog -sv src/int_alu_test.sv
xvlog -sv src/lq.sv
xvlog -sv src/lsu.sv
xvlog -sv src/mem.sv
xvlog -sv src/rob.sv
xvlog -sv src/sq.sv
xvlog -sv src/writeback.sv
xvlog -sv src/top.sv

# Finally compile the testbench
xvlog -sv tb/top_tb.sv

# Elaborate the design
xelab -debug typical top_tb -s top_tb_sim

# Run simulation (optional - comment out if you want to run manually)
# xsim top_tb_sim -runall
