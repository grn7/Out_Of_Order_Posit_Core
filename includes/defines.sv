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

    //derived parameters 
    localparam ROB_IDX_W = (ROB_LENGTH > 1) ? $clog2(ROB_LENGTH) : 1;
    localparam IQ_IDX_W = (IQ_LENGTH > 1) ? $clog2(IQ_LENGTH) : 1;
    localparam BYPASS_IDX_W = (BYPASS_LENGTH > 1) ? $clog2(BYPASS_LENGTH) : 1;
    localparam PHYS_REG_IDX_W = (PHYS_REG_LENGTH > 1) ? $clog2(PHYS_REG_LENGTH) : 1;
    localparam ARCH_REG_IDX_W = (ARCH_REG_LENGTH > 1) ? $clog2(ARCH_REG_LENGTH) : 1;
    localparam INSTR_MEM_IDX_W = (INSTR_MEM_LENGTH > 1) ? $clog2(INSTR_MEM_LENGTH) : 1;
    localparam DATA_MEM_IDX_W = (DATA_MEM_LENGTH > 1) ? $clog2(DATA_MEM_LENGTH) : 1;
    localparam FETCH_BUFFER_IDX_W = (FETCH_BUFFER_LENGTH > 1) ? $clog2(FETCH_BUFFER_LENGTH) : 1;

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
    } rob_entry_t;

endpackage
