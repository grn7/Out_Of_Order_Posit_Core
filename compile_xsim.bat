@echo off
REM Vivado xsim compilation script for Out of Order RISC-V Processor

echo Compiling package definitions...
xvlog -sv includes/defines.sv
if %errorlevel% neq 0 exit /b %errorlevel%

echo Compiling source files...
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

echo Compiling testbench...
xvlog -sv tb/top_tb.sv
if %errorlevel% neq 0 exit /b %errorlevel%

echo Elaborating design...
xelab -debug typical top_tb -s top_tb_sim
if %errorlevel% neq 0 exit /b %errorlevel%

echo Compilation successful! Run simulation with: xsim top_tb_sim -runall
