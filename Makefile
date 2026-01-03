all: lint

SV_FILES = \
	includes/defines.sv \
	src/bp.sv \
	src/btb.sv \
	src/ID.sv \
	src/IF.sv \
	src/int_alu_test.sv \
	src/lq.sv \
	src/lsu.sv \
	src/mem.sv \
	src/sq.sv \
	src/top.sv \

lint:
	verilator --sv --lint-only $(SV_FILES)