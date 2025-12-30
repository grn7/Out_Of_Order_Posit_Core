package general_defines;
    // for parameters common to the whole core
    parameter INT_DATA_W = 32;
    parameter POSIT_DATA_W = 8;
    parameter ROB_LENGTH = 16;
    parameter IQ_LENGTH = 8;
    parameter BYPASS_LENGTH = 3;
    parameter PHYS_REG_LENGTH = 64;
    parameter ARCH_REG_LENGTH = 32;
    parameter INSTR_MEM_LENGTH = 256;
    parameter DATA_MEM_LENGTH = 256;
    parameter FETCH_BUFFER_LENGTH = 8;
    parameter PHT_LENGTH = 1024; 
    parameter BTB_LENGTH = 64;

    //derived parameters 
    localparam ROB_IDX_W = (ROB_LENGTH > 1) ? $clog2(ROB_LENGTH) : 1;
    localparam IQ_IDX_W = (IQ_LENGTH > 1) ? $clog2(IQ_LENGTH) : 1;
    localparam BYPASS_IDX_W = (BYPASS_LENGTH > 1) ? $clog2(BYPASS_LENGTH) : 1;
    localparam PHYS_REG_IDX_W = (PHYS_REG_LENGTH > 1) ? $clog2(PHYS_REG_LENGTH) : 1;
    localparam ARCH_REG_IDX_W = (ARCH_REG_LENGTH > 1) ? $clog2(ARCH_REG_LENGTH) : 1;
    localparam INSTR_MEM_IDX_W = (INSTR_MEM_LENGTH > 1) ? $clog2(INSTR_MEM_LENGTH) : 1;
    localparam DATA_MEM_IDX_W = (DATA_MEM_LENGTH > 1) ? $clog2(DATA_MEM_LENGTH) : 1;
    localparam FETCH_BUFFER_IDX_W = (FETCH_BUFFER_LENGTH > 1) ? $clog2(FETCH_BUFFER_LENGTH) : 1;
    localparam PHT_IDX_W = (PHT_LENGTH> 1) ? $clog2(PHT_LENGTH) : 1;
    localparam BTB_IDX_W = (BTB_LENGTH > 1) ? $clog2(BTB_LENGTH) : 1;

    //defining opcode and other similar things 
    `define OPCODE_LOAD 7'b0000011 //for LB,LH,LW,LBU,LHU
    `define OPCODE_STORE 7'b0100011 //for SB,SH,SW
    `define OPCODE_ARITH_I 7'b0010011 //for ADDI,XORI,ORI,ANDI, ...
    `define OPCODE_ARITH_R 7'b0110011 //for ADD,SUB,SLL,XOR,OR,AND
    `define OPCODE_LUI 7'b0110111 //for LUI
    `define OPCODE_AUIPC 7'b0010111 //for AUIPC
    `define OPCODE_BRANCH 7'b1100011 //for BEQ,BNE,BLT ...
    `define OPCODE_JAL 7'b1101111 //for JAL
    `define OPCODE_JALR 7'b1100111 //for JALR
    //M extension - same opcode as ARITH_R , uses funct 7 to differentiate
    `define OPCODE_M_EXT 7'b0110011 //for MUL,DIV,MULH .. 
    // FP 
    `define OPCODE_FP 7'b1010011 //for FADD,FSUB,FMUL,FDIV ..
    

    //typedefs 
    typedef struct packed{
        logic valid;
        logic [INSTR_MEM_IDX_W-1:0] pc;
        logic [ARCH_REG_IDX_W-1:0] logical_rd;
        logic [PHYS_REG_IDX_W-1:0] phys_rd;
        logic [INT_DATA_W-1:0] result;
        logic done;
        logic is_store;
        logic is_load;
        logic [6:0] opcode;
        logic [2:0] funct3;
        logic [6:0] funct7;
        logic is_branch;
        logic pred_taken;
        logic [INSTR_MEM_IDX_W-1:0] pred_target;
    } rob_entry_t;

endpackage
