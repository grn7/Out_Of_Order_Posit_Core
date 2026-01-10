all: lint

SV_FILES = \
	includes/defines.sv \
	src/bp.sv \
	src/btb.sv \
	src/commit.sv \
	src/ID.sv \
	src/IF.sv \
	src/int_alu_test.sv \
	src/lq.sv \
	src/lsu.sv \
	src/mem.sv \
	src/sq.sv \
	src/writeback.sv \
	src/top.sv \
	tb/top_tb.sv

lint:
	verilator --sv --lint-only --timing $(SV_FILES)