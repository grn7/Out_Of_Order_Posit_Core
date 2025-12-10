// BOOM Out-of-Order RISC-V Core (RV32IMF)

/*
1. Add floating point reg, look into how to do floats

*/




`timescale 1ns / 1ps

// Instruction opcodes
`define OPCODE_LOAD     7'b0000011
`define OPCODE_STORE    7'b0100011
`define OPCODE_ARITH_I  7'b0010011
`define OPCODE_ARITH_R  7'b0110011
`define OPCODE_FP       7'b1010011

// ALU operations
`define ALU_ADD   3'b000
`define ALU_SUB   3'b001
`define ALU_MUL   3'b010
`define ALU_DIV   3'b011

/* imem_ready will be asserted HIGH when :-
1. The frontend has a valid PC &&
2. There’s no stall or hazard preventing instruction fetch &&
3. The instruction queue or decode stage has room to accept new instructions.
*/

module boom_core #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter ROB_ENTRIES = 16,
    parameter IQ_ENTRIES = 8,
    parameter PHYS_REG_COUNT = 64,
    parameter ARCH_REG_COUNT = 32
)(
    input logic clk,
    input logic reset,

    // Instruction memory interface
    output logic [ADDR_WIDTH-1:0] imem_addr,
    input logic [DATA_WIDTH-1:0] imem_data,
    output logic imem_req, // Core is requesting an instruction at the addr imem_addr 
    input logic imem_ready, // L1 cache is ready with the data

    // Data memory interface
    output logic [ADDR_WIDTH-1:0] dmem_addr,
    output logic [DATA_WIDTH-1:0] dmem_wdata,
    input logic [DATA_WIDTH-1:0] dmem_rdata,
    output logic dmem_we,
    output logic dmem_req,
    input logic dmem_ready
);

    // ROB entry structure
    typedef struct packed {
        logic        valid;
        logic [31:0] pc;
        logic [4:0]  logical_rd;
        logic [5:0]  phys_rd;
        logic [31:0] result;
        logic        done;
        logic        is_store;
        logic        is_load;
        logic [6:0]  opcode;
        logic [2:0]  funct3;
        logic [6:0]  funct7;
    } rob_entry_t;

    // Issue queue entry structure
    typedef struct packed {
        logic [6:0]  opcode;
        logic [31:0] pc;
        logic [5:0]  phys_rd;
        logic [5:0]  phys_rs1;
        logic [5:0]  phys_rs2;
        logic [31:0] imm;
        logic [2:0]  fu_type; // 0=ALU, 1=MEM, 2=MUL, 3=DIV, 4=FPU
        logic        rs1_ready;
        logic        rs2_ready;
        logic [31:0] rs1_value; // are these tags ??
        logic [31:0] rs2_value;
        logic        valid;
        logic [2:0]  funct3;
        logic [6:0]  funct7;
        logic [3:0]  rob_idx;
    } iq_entry_t;

    // LSU entry structure; Load-Store unit, that will handle ONLY loads and stores
    typedef struct packed {
        logic        valid;
        logic [31:0] addr;
        logic [31:0] data;
        logic [3:0]  rob_idx;
    } lsu_entry_t;


    // Bypass network entry, a.k.a DATA FORWARDING NETWORK
    typedef struct packed {
        logic        valid;
        logic [5:0]  phys_rd;
        logic [31:0] result;
    } bypass_entry_t;


    // Pipeline stage registers
    logic [ADDR_WIDTH-1:0] pc;
    logic [DATA_WIDTH-1:0] fetch_buffer [0:7];
    logic [2:0] fetch_head, fetch_tail;
    logic       fetch_buffer_full, fetch_stall;

    // Reorder Buffer
    rob_entry_t rob [0:ROB_ENTRIES-1];
    logic [3:0] rob_head, rob_tail;
    logic       rob_full, rob_empty;

    // Register Renaming
    logic [5:0]  rename_table [0:ARCH_REG_COUNT-1]; // Maps logical to physical regs
    logic [5:0]  old_rename_table [0:ARCH_REG_COUNT-1]; // For recovery
    logic [5:0]  free_list [0:PHYS_REG_COUNT-1];
    logic [5:0]  free_list_head, free_list_tail;
    logic        free_list_empty;

    // Issue Queues
    iq_entry_t int_iq [0:IQ_ENTRIES-1];
    iq_entry_t mem_iq [0:IQ_ENTRIES-1];
    iq_entry_t fp_iq  [0:IQ_ENTRIES-1];
    logic [2:0] int_iq_head, int_iq_tail;
    logic [2:0] mem_iq_head, mem_iq_tail;
    logic [2:0] fp_iq_head, fp_iq_tail;
    logic       int_iq_full, mem_iq_full, fp_iq_full;

    // Physical Register File
    logic [31:0] phys_reg_file [0:PHYS_REG_COUNT-1];
    logic        phys_reg_valid [0:PHYS_REG_COUNT-1];

    // Load/Store Unit
    lsu_entry_t laq [0:7]; // Load Address Queue
    lsu_entry_t saq [0:7]; // Store Address Queue
    lsu_entry_t sdq [0:7]; // Store Data Queue
    logic [2:0] laq_head, laq_tail;
    logic [2:0] saq_head, saq_tail;
    logic [2:0] sdq_head, sdq_tail;
    logic       laq_full, saq_full, sdq_full;

    // Functional Units
    logic [31:0] alu_result;
    logic        alu_valid;
    logic [5:0]  alu_dest;

    logic [31:0] mul_result;
    logic        mul_valid;
    logic [5:0]  mul_dest;
    logic        mul_busy;
    logic [4:0]  mul_counter;
    logic [31:0] mul_op1, mul_op2;

    logic [31:0] div_result;
    logic        div_valid;
    logic [5:0]  div_dest;
    logic        div_busy;
    logic [4:0]  div_counter;
    logic [31:0] div_op1, div_op2;

    logic [31:0] fdiv_result;
    logic        fdiv_valid;
    logic [5:0]  fdiv_dest;
    logic        fdiv_busy;
    logic [4:0]  fdiv_counter;
    logic [31:0] fdiv_op1, fdiv_op2;

    // Load execution
    logic [31:0] load_result;
    logic        load_valid;
    logic [5:0]  load_dest;
    logic [3:0]  load_rob_idx;
    logic [4:0]  load_counter;

    // Bypass network
    bypass_entry_t bypass [0:2]; // 3-stage bypass

    // Control signals
    logic decode_stall, rename_stall, dispatch_stall;
    logic commit_store;

    // Helper functions
    function logic [31:0] sign_extend_12(input logic [11:0] imm);
        return {{20{imm[11]}}, imm};
    endfunction

    function logic [31:0] sign_extend_20(input logic [19:0] imm);
        return {{12{imm[19]}}, imm};
    endfunction

    function bypass_entry_t get_default_bypass();
        get_default_bypass.phys_rd = '0;
        get_default_bypass.result = '0;
        get_default_bypass.valid = 1'b0;
    endfunction

    // Status flags
    assign fetch_buffer_full = (fetch_tail + 1) % 8 == fetch_head ;
    assign rob_full = (rob_tail + 1) % ROB_ENTRIES == rob_head && rob[rob_tail].valid;
    assign rob_empty = rob_head == rob_tail && !rob[rob_head].valid;
    assign free_list_empty = free_list_head == free_list_tail;
    assign int_iq_full = (int_iq_tail + 1) % IQ_ENTRIES == int_iq_head;
    assign mem_iq_full = (mem_iq_tail + 1) % IQ_ENTRIES == mem_iq_head;
    assign fp_iq_full  = (fp_iq_tail + 1) % IQ_ENTRIES == fp_iq_head;
    assign laq_full = (laq_tail + 1) % 8 == laq_head;
    assign saq_full = (saq_tail + 1) % 8 == saq_head;
    assign sdq_full = (sdq_tail + 1) % 8 == sdq_head;

    // Instruction fetch
    assign imem_addr = pc;  // this signal looks pointless,but will be used for impl branch instr
    // (imem_req is an output)only ask for new instruction 
    // if you can put it in buffer / there is no stalls
    assign imem_req = !reset && !fetch_stall && !fetch_buffer_full; 
    
    int idx ;
    logic starting;

    always_ff @(posedge clk) begin
        if (reset) begin
            pc <= 32'h0;
            fetch_head <= 0;
            fetch_tail <= 0;
            fetch_stall <= 0;
            starting <= 1;
        end else begin
            // Fetch logic
            // if (imem_ready && imem_req) begin
            if (imem_ready && imem_req) begin
                fetch_buffer[fetch_tail] <= imem_data;
                fetch_tail <= starting ? 0 : (fetch_tail + 1) % 8;
                pc <= starting ? 0 : pc + 4;
                starting <= 0;
            end
            
            // Stall conditions
            fetch_stall <= rob_full || free_list_empty;
        end
    end

    // Decode/Rename/Dispatch stage
    logic [31:0] current_inst;
    logic [4:0]  rs1, rs2, rd;
    logic [31:0] immediate;
    logic [6:0]  opcode;
    logic [2:0]  funct3;
    logic [6:0]  funct7;
    logic        inst_valid;


    always_ff @(posedge clk) begin
        if (reset) begin
            // Initialize rename table to identity mapping
            for (int i = 0; i < ARCH_REG_COUNT; i++) begin
                rename_table[i] <= i;
            end
            
            // Initialize free list
            for (int i = ARCH_REG_COUNT; i < PHYS_REG_COUNT; i++) begin
                free_list[i - ARCH_REG_COUNT] <= i;
            end
            free_list_head <= 0;
            free_list_tail <= PHYS_REG_COUNT - ARCH_REG_COUNT;
            
            // Initialize issue queues
            for (int i = 0; i < IQ_ENTRIES; i++) begin
                int_iq[i] <= '0;
                mem_iq[i] <= '0;
                fp_iq[i] <= '0;
            end
            int_iq_head <= 0; int_iq_tail <= 0;
            mem_iq_head <= 0; mem_iq_tail <= 0;
            fp_iq_head <= 0; fp_iq_tail <= 0;
            
            // Initialize ROB
            for (int i = 0; i < ROB_ENTRIES; i++) begin
                rob[i] <= '0;
            end
            rob_head <= 0;
            rob_tail <= 0;
            
        end else if (fetch_head != fetch_tail && !rob_full && !free_list_empty) begin

            // Decode
            current_inst = fetch_buffer[fetch_head];
            opcode = current_inst[6:0];
            rd = current_inst[11:7];
            funct3 = current_inst[14:12];
            rs1 = current_inst[19:15];
            rs2 = current_inst[24:20];
            funct7 = current_inst[31:25];
            
            // Immediate generation
            case (opcode)
                `OPCODE_LOAD, `OPCODE_ARITH_I: 
                    immediate = sign_extend_12(current_inst[31:20]);
                7'b0100011: // S-type (STORE)
                    immediate = sign_extend_12({current_inst[31:25], current_inst[11:7]});
                default: immediate = 0;
            endcase
            
            inst_valid = 1;
            
            // Allocate ROB entry
            if (inst_valid && !rob_full) begin
                // what the fuck is this soccery ??? whenever is tail kept incr and is ABOVE head, it's a bit different
                rob[rob_tail].pc <= pc - ((fetch_tail >= fetch_head) ? 
                                        (fetch_tail - fetch_head) * 4 : 
                                        (8 + fetch_tail - fetch_head) * 4);
                rob[rob_tail].logical_rd <= rd; // what's the use of logical_rd ???
                rob[rob_tail].done <= 0;
                rob[rob_tail].valid <= 1;
                rob[rob_tail].opcode <= opcode;
                rob[rob_tail].funct3 <= funct3;
                rob[rob_tail].funct7 <= funct7;
                rob[rob_tail].is_load <= (opcode == `OPCODE_LOAD);
                rob[rob_tail].is_store <= (opcode == `OPCODE_STORE);
                
                // Register renaming for destination
                if (rd != 0 && !free_list_empty) begin // why can't rd be zero ???
                    rob[rob_tail].phys_rd <= free_list[free_list_head];
                    rename_table[rd] <= free_list[free_list_head];
                    free_list_head <= (free_list_head + 1) % PHYS_REG_COUNT;
                end else begin
                    rob[rob_tail].phys_rd <= 0; // if rd is zero, we make phys_rd of that ROB entry as 0
                end
                
                // Dispatch to issue queue
                case (opcode)
                    `OPCODE_ARITH_I, `OPCODE_ARITH_R: begin
                        if (!int_iq_full) begin

                            int_iq[int_iq_tail].pc <= pc - ((fetch_tail >= fetch_head) ? 
                                        (fetch_tail - fetch_head) * 4 : 
                                        (8 + fetch_tail - fetch_head) * 4);
                            int_iq[int_iq_tail].phys_rd <= free_list[free_list_head];
                            int_iq[int_iq_tail].phys_rs1 <= rename_table[rs1];
                            int_iq[int_iq_tail].phys_rs2 <= (opcode == `OPCODE_ARITH_R) ? rename_table[rs2] : 0;
                            int_iq[int_iq_tail].imm <= immediate; 

                            int_iq[int_iq_tail].rs1_ready <= phys_reg_valid[rename_table[rs1]];
                            int_iq[int_iq_tail].rs2_ready <= (opcode == `OPCODE_ARITH_I) ? 1 : phys_reg_valid[rename_table[rs2]];
                            int_iq[int_iq_tail].rs1_value <= phys_reg_file[rename_table[rs1]];
                            int_iq[int_iq_tail].rs2_value <= phys_reg_file[rename_table[rs2]];

                            int_iq[int_iq_tail].valid <= 1;
                            int_iq[int_iq_tail].opcode <= opcode;
                            int_iq[int_iq_tail].funct3 <= funct3;
                            int_iq[int_iq_tail].funct7 <= funct7;
                            int_iq[int_iq_tail].rob_idx <= rob_tail;
                            
                            // Determine FU type
                            if (opcode == `OPCODE_ARITH_R && funct7 == 7'b0000001) begin
                                case (funct3)
                                    3'b000: int_iq[int_iq_tail].fu_type <= 2; // MUL
                                    3'b100: int_iq[int_iq_tail].fu_type <= 3; // DIV
                                    default: int_iq[int_iq_tail].fu_type <= 0; // ALU
                                endcase
                            end else begin
                                int_iq[int_iq_tail].fu_type <= 0; // ALU
                            end
                            
                            int_iq_tail <= (int_iq_tail + 1) % IQ_ENTRIES;
                        end
                    end
                    
                    `OPCODE_LOAD, `OPCODE_STORE: begin
                        if (!mem_iq_full) begin
                            mem_iq[mem_iq_tail].pc <= rob[rob_tail].pc;
                            mem_iq[mem_iq_tail].phys_rd <= rob[rob_tail].phys_rd;
                            mem_iq[mem_iq_tail].phys_rs1 <= rename_table[rs1];
                            mem_iq[mem_iq_tail].phys_rs2 <= (opcode == `OPCODE_STORE) ? rename_table[rs2] : 0;
                            mem_iq[mem_iq_tail].imm <= immediate;
                            mem_iq[mem_iq_tail].rs1_ready <= phys_reg_valid[rename_table[rs1]];
                            mem_iq[mem_iq_tail].rs2_ready <= (opcode == `OPCODE_LOAD) ? 1 : phys_reg_valid[rename_table[rs2]];
                            mem_iq[mem_iq_tail].rs1_value <= phys_reg_file[rename_table[rs1]];
                            mem_iq[mem_iq_tail].rs2_value <= phys_reg_file[rename_table[rs2]];
                            mem_iq[mem_iq_tail].valid <= 1;
                            mem_iq[mem_iq_tail].opcode <= opcode;
                            mem_iq[mem_iq_tail].funct3 <= funct3;
                            mem_iq[mem_iq_tail].fu_type <= 1; // MEM
                            mem_iq[mem_iq_tail].rob_idx <= rob_tail;
                            
                            mem_iq_tail <= (mem_iq_tail + 1) % IQ_ENTRIES;
                        end
                    end
                    
                    `OPCODE_FP: begin
                        if (!fp_iq_full) begin
                            fp_iq[fp_iq_tail].pc <= rob[rob_tail].pc;
                            fp_iq[fp_iq_tail].phys_rd <= rob[rob_tail].phys_rd;
                            fp_iq[fp_iq_tail].phys_rs1 <= rename_table[rs1];
                            fp_iq[fp_iq_tail].phys_rs2 <= rename_table[rs2];
                            fp_iq[fp_iq_tail].rs1_ready <= phys_reg_valid[rename_table[rs1]];
                            fp_iq[fp_iq_tail].rs2_ready <= phys_reg_valid[rename_table[rs2]];
                            fp_iq[fp_iq_tail].rs1_value <= phys_reg_file[rename_table[rs1]];
                            fp_iq[fp_iq_tail].rs2_value <= phys_reg_file[rename_table[rs2]];
                            fp_iq[fp_iq_tail].valid <= 1;
                            fp_iq[fp_iq_tail].opcode <= opcode;
                            fp_iq[fp_iq_tail].funct3 <= funct3;
                            fp_iq[fp_iq_tail].funct7 <= funct7;
                            fp_iq[fp_iq_tail].fu_type <= 4; // FPU
                            fp_iq[fp_iq_tail].rob_idx <= rob_tail;
                            
                            fp_iq_tail <= (fp_iq_tail + 1) % IQ_ENTRIES;
                        end
                    end
                endcase
                
                rob_tail <= (rob_tail + 1) % ROB_ENTRIES;
                fetch_head <= (fetch_head + 1) % 8;
            end
        end
        
        // Wakeup logic - check bypass network for ready operands
        for (int i = 0; i < IQ_ENTRIES; i++) begin

            // Check bypass for rs1
            if (int_iq[i].valid) begin
                if (!int_iq[i].rs1_ready) begin
                    for (int j = 0; j < 3; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == int_iq[i].phys_rs1) begin
                            int_iq[i].rs1_value <= bypass[j].result;
                            int_iq[i].rs1_ready <= 1;
                        end
                    end
                end
                
                // Check bypass for rs2
                if (!int_iq[i].rs2_ready && int_iq[i].opcode == `OPCODE_ARITH_R) begin
                    for (int j = 0; j < 3; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == int_iq[i].phys_rs2) begin
                            int_iq[i].rs2_value <= bypass[j].result;
                            int_iq[i].rs2_ready <= 1;
                        end
                    end
                end
            end
            
            // Similar wakeup logic for mem_iq
            if (mem_iq[i].valid) begin
                if (!mem_iq[i].rs1_ready) begin
                    for (int j = 0; j < 3; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == mem_iq[i].phys_rs1) begin
                            mem_iq[i].rs1_value <= bypass[j].result;
                            mem_iq[i].rs1_ready <= 1;
                        end
                    end
                end
                
                if (!mem_iq[i].rs2_ready && mem_iq[i].opcode == `OPCODE_STORE) begin
                    for (int j = 0; j < 3; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == mem_iq[i].phys_rs2) begin
                            mem_iq[i].rs2_value <= bypass[j].result;
                            mem_iq[i].rs2_ready <= 1;
                        end
                    end
                end
            end
            
            // Similar wakeup logic for fp_iq
            if (fp_iq[i].valid) begin
                if (!fp_iq[i].rs1_ready) begin
                    for (int j = 0; j < 3; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == fp_iq[i].phys_rs1) begin
                            fp_iq[i].rs1_value <= bypass[j].result;
                            fp_iq[i].rs1_ready <= 1;
                        end
                    end
                end
                
                if (!fp_iq[i].rs2_ready) begin
                    for (int j = 0; j < 3; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == fp_iq[i].phys_rs2) begin
                            fp_iq[i].rs2_value <= bypass[j].result;
                            fp_iq[i].rs2_ready <= 1;
                        end
                    end
                end
            end
        end
    end



    // Issue/Execute stage
    always_ff @(posedge clk) begin
        if (reset) begin
            alu_valid <= 0;
            mul_valid  <= 0;  mul_busy  <= 0;  mul_counter <= 0;
            div_valid  <= 0;  div_busy  <= 0;  div_counter <= 0;
            fdiv_valid <= 0;  fdiv_busy <= 0; fdiv_counter <= 0;
            load_valid <= 0;  load_counter <= 0;
            
            laq_head <= 0; laq_tail <= 0;
            saq_head <= 0; saq_tail <= 0;
            sdq_head <= 0; sdq_tail <= 0;
            
            for (int i = 0; i < 8; i++) begin
                laq[i] <= '0;
                saq[i] <= '0;
                sdq[i] <= '0;
            end
            
        end else begin
            // Clear valid signals
            alu_valid <= 0;
            mul_valid <= 0;
            div_valid <= 0;
            fdiv_valid <= 0;
            load_valid <= 0;
            
            // Issue from integer queue
            for (int i = 0; i < IQ_ENTRIES; i++) begin
                idx = (int_iq_head + i) % IQ_ENTRIES;
                if (int_iq[idx].valid && int_iq[idx].rs1_ready && 
                    (int_iq[idx].opcode == `OPCODE_ARITH_I || int_iq[idx].rs2_ready)) begin
                    
                    case (int_iq[idx].fu_type)
                        0: begin // ALU
                            case (int_iq[idx].opcode)
                                `OPCODE_ARITH_I: begin
                                    case (int_iq[idx].funct3)
                                        3'b000: alu_result <= int_iq[idx].rs1_value + int_iq[idx].imm; // ADDI
                                        default: alu_result <= int_iq[idx].rs1_value + int_iq[idx].imm;
                                    endcase
                                end
                                `OPCODE_ARITH_R: begin
                                    case (int_iq[idx].funct3)
                                        3'b000: begin
                                            if (int_iq[idx].funct7 == 7'b0000000)
                                                alu_result <= int_iq[idx].rs1_value + int_iq[idx].rs2_value; // ADD
                                            else
                                                alu_result <= int_iq[idx].rs1_value - int_iq[idx].rs2_value; // SUB
                                        end
                                        default: alu_result <= int_iq[idx].rs1_value + int_iq[idx].rs2_value;
                                    endcase
                                end
                                default: alu_result <= 0;
                            endcase
                            alu_valid <= 1;
                            alu_dest <= int_iq[idx].phys_rd;
                            int_iq[idx].valid <= 0;
                            break;
                        end
                        
                        2: begin // MUL
                            if (!mul_busy) begin
                                mul_op1 <= int_iq[idx].rs1_value;
                                mul_op2 <= int_iq[idx].rs2_value;
                                mul_dest <= int_iq[idx].phys_rd;
                                mul_busy <= 1;
                                mul_counter <= 4; // 5-cycle multiply
                                int_iq[idx].valid <= 0;
                                break;
                            end
                        end
                        
                        3: begin // DIV
                            if (!div_busy) begin
                                div_op1 <= int_iq[idx].rs1_value;
                                div_op2 <= int_iq[idx].rs2_value;
                                div_dest <= int_iq[idx].phys_rd;
                                div_busy <= 1;
                                div_counter <= 9; // 10-cycle divide
                                int_iq[idx].valid <= 0;
                                break;
                            end
                        end
                    endcase
                end
            end
            
            // Issue from memory queue
            for (int i = 0; i < IQ_ENTRIES; i++) begin
                idx = (mem_iq_head + i) % IQ_ENTRIES;
                if (mem_iq[idx].valid && mem_iq[idx].rs1_ready && 
                    (mem_iq[idx].opcode == `OPCODE_LOAD || mem_iq[idx].rs2_ready)) begin
                    
                    case (mem_iq[idx].opcode)
                        `OPCODE_LOAD: begin
                            if (!laq_full) begin
                                laq[laq_tail].addr <= mem_iq[idx].rs1_value + mem_iq[idx].imm;
                                laq[laq_tail].valid <= 1;
                                laq[laq_tail].rob_idx <= mem_iq[idx].rob_idx;
                                laq_tail <= (laq_tail + 1) % 8;
                                mem_iq[idx].valid <= 0;
                                break;
                            end
                        end
                        
                        `OPCODE_STORE: begin
                            if (!saq_full && !sdq_full) begin
                                saq[saq_tail].addr <= mem_iq[idx].rs1_value + mem_iq[idx].imm;
                                saq[saq_tail].valid <= 1;
                                saq[saq_tail].rob_idx <= mem_iq[idx].rob_idx;
                                saq_tail <= (saq_tail + 1) % 8;
                                
                                sdq[sdq_tail].data <= mem_iq[idx].rs2_value;
                                sdq[sdq_tail].valid <= 1;
                                sdq[sdq_tail].rob_idx <= mem_iq[idx].rob_idx;
                                sdq_tail <= (sdq_tail + 1) % 8;
                                
                                mem_iq[idx].valid <= 0;
                                break;
                            end
                        end
                    endcase
                end
            end
            
            // Issue from FP queue
            for (int i = 0; i < IQ_ENTRIES; i++) begin
                idx = (fp_iq_head + i) % IQ_ENTRIES;
                if (fp_iq[idx].valid && fp_iq[idx].rs1_ready && fp_iq[idx].rs2_ready) begin
                    case (fp_iq[idx].funct7)
                        7'b0001100: begin // FDIV
                            if (!fdiv_busy) begin
                                fdiv_op1 <= fp_iq[idx].rs1_value;
                                fdiv_op2 <= fp_iq[idx].rs2_value;
                                fdiv_dest <= fp_iq[idx].phys_rd;
                                fdiv_busy <= 1;
                                fdiv_counter <= 15; // 16-cycle FP divide
                                fp_iq[idx].valid <= 0;
                                break;
                            end
                        end
                        
                        7'b0000000: begin // FADD
                            // Simplified FP add - should be proper IEEE754 implementation
                            alu_result <= fp_iq[idx].rs1_value + fp_iq[idx].rs2_value;
                            alu_valid <= 1;
                            alu_dest <= fp_iq[idx].phys_rd;
                            fp_iq[idx].valid <= 0;
                            break;
                        end
                        
                        default: begin
                            alu_result <= fp_iq[idx].rs1_value + fp_iq[idx].rs2_value;
                            alu_valid <= 1;
                            alu_dest <= fp_iq[idx].phys_rd;
                            fp_iq[idx].valid <= 0;
                            break;
                        end
                    endcase
                end
            end
            
            // Multiplier pipeline
            if (mul_busy) begin
                if (mul_counter > 0) begin
                    mul_counter <= mul_counter - 1;
                    if (mul_counter == 1) begin
                        mul_result <= mul_op1 * mul_op2;
                        mul_valid <= 1;
                        mul_busy <= 0;
                    end
                end
            end
            
            // Divider pipeline
            if (div_busy) begin
                if (div_counter > 0) begin
                    div_counter <= div_counter - 1;
                    if (div_counter == 1) begin
                        div_result <= (div_op2 != 0) ? div_op1 / div_op2 : 32'hFFFFFFFF;
                        div_valid <= 1;
                        div_busy <= 0;
                    end
                end
            end
            
            // FP Divider pipeline
            if (fdiv_busy) begin
                if (fdiv_counter > 0) begin
                    fdiv_counter <= fdiv_counter - 1;
                    if (fdiv_counter == 1) begin
                        // Simplified FP division - should be proper IEEE754
                        fdiv_result <= (fdiv_op2 != 0) ? fdiv_op1 / fdiv_op2 : 32'h7FC00000;
                        fdiv_valid <= 1;
                        fdiv_busy <= 0;
                    end
                end
            end
            
            // Load execution
            if (laq[laq_head].valid && !load_valid && load_counter == 0) begin
                load_counter <= 4; // 5-cycle load latency
            end
            
            if (load_counter > 0) begin
                load_counter <= load_counter - 1;
                if (load_counter == 1) begin
                    load_result <= dmem_rdata; // Assume data is ready
                    load_valid <= 1;
                    load_rob_idx <= laq[laq_head].rob_idx;
                    
                    // Find destination register from ROB
                    load_dest <= rob[laq[laq_head].rob_idx].phys_rd;
                    
                    laq[laq_head].valid <= 0;
                    laq_head <= (laq_head + 1) % 8;
                end
            end
        end
    end

    // Memory interface for loads
    assign dmem_addr = (laq[laq_head].valid && load_counter == 4) ? laq[laq_head].addr : 
                            commit_store ? saq[saq_head].addr : 32'h0;
    assign dmem_wdata = commit_store ? sdq[sdq_head].data : 32'h0;
    assign dmem_we = commit_store;
    assign dmem_req = (laq[laq_head].valid && load_counter == 4) || commit_store;

    // Writeback stage
    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < PHYS_REG_COUNT; i++) begin
                phys_reg_file[i] <= 0;
                phys_reg_valid[i] <= (i < ARCH_REG_COUNT); // Arch regs start valid
            end
            bypass <= '{default:'0}; 
        end else begin
            // Shift bypass network
            bypass[2] <= bypass[1];
            bypass[1] <= bypass[0];
            bypass[0] <= '0;
            
            // Writeback from ALU
            if (alu_valid && alu_dest != 0) begin
                phys_reg_file[alu_dest] <= alu_result;
                phys_reg_valid[alu_dest] <= 1;
                bypass[0].phys_rd <= alu_dest;
                bypass[0].result <= alu_result;
                bypass[0].valid <= 1;
                
                // Mark ROB entry as done
                for (int i = 0; i < ROB_ENTRIES; i++) begin
                    if (rob[i].valid && rob[i].phys_rd == alu_dest && !rob[i].done) begin
                        rob[i].result <= alu_result;
                        rob[i].done <= 1;
                        break;
                    end
                end
            end
            
            // Writeback from multiplier
            if (mul_valid && mul_dest != 0) begin
                phys_reg_file[mul_dest] <= mul_result;
                phys_reg_valid[mul_dest] <= 1;
                bypass[0].phys_rd <= mul_dest;
                bypass[0].result <= mul_result;
                bypass[0].valid <= 1;
                
                // Mark ROB entry as done
                for (int i = 0; i < ROB_ENTRIES; i++) begin
                    if (rob[i].valid && rob[i].phys_rd == mul_dest && !rob[i].done) begin
                        rob[i].result <= mul_result;
                        rob[i].done <= 1;
                        break;
                    end
                end
            end
            
            // Writeback from divider
            if (div_valid && div_dest != 0) begin
                phys_reg_file[div_dest] <= div_result;
                phys_reg_valid[div_dest] <= 1;
                bypass[0].phys_rd <= div_dest;
                bypass[0].result <= div_result;
                bypass[0].valid <= 1;
                
                // Mark ROB entry as done
                for (int i = 0; i < ROB_ENTRIES; i++) begin
                    if (rob[i].valid && rob[i].phys_rd == div_dest && !rob[i].done) begin
                        rob[i].result <= div_result;
                        rob[i].done <= 1;
                        break;
                    end
                end
            end
            
            // Writeback from FP divider
            if (fdiv_valid && fdiv_dest != 0) begin
                phys_reg_file[fdiv_dest] <= fdiv_result;
                phys_reg_valid[fdiv_dest] <= 1;
                bypass[0].phys_rd <= fdiv_dest;
                bypass[0].result <= fdiv_result;
                bypass[0].valid <= 1;
                
                // Mark ROB entry as done
                for (int i = 0; i < ROB_ENTRIES; i++) begin
                    if (rob[i].valid && rob[i].phys_rd == fdiv_dest && !rob[i].done) begin
                        rob[i].result <= fdiv_result;
                        rob[i].done <= 1;
                        break;
                    end
                end
            end
            
            // Writeback from load
            if (load_valid && load_dest != 0) begin
                phys_reg_file[load_dest] <= load_result;
                phys_reg_valid[load_dest] <= 1;
                bypass[0].phys_rd <= load_dest;
                bypass[0].result <= load_result;
                bypass[0].valid <= 1;
                
                // Mark ROB entry as done
                rob[load_rob_idx].result <= load_result;
                rob[load_rob_idx].done <= 1;
            end
        end
    end


    // Commit stage
    always_ff @(posedge clk) begin
        if (reset) begin
            commit_store <= 0;
        end else begin
            commit_store <= 0;
            
            // Commit instructions in program order
            if (rob[rob_head].valid && rob[rob_head].done) begin
                if (rob[rob_head].is_store) begin
                    // Commit store - find matching entries in SAQ/SDQ
                    for (int i = 0; i < 8; i++) begin
                        if (saq[i].valid && saq[i].rob_idx == rob_head &&
                            sdq[i].valid && sdq[i].rob_idx == rob_head) begin
                            commit_store <= 1;
                            if (dmem_ready) begin
                                saq[i].valid <= 0;
                                sdq[i].valid <= 0;
                                saq_head <= (saq_head + 1) % 8;
                                sdq_head <= (sdq_head + 1) % 8;
                                
                                // Free old physical register if needed
                                if (rob[rob_head].logical_rd != 0) begin
                                    free_list[free_list_tail] <= rename_table[rob[rob_head].logical_rd];
                                    free_list_tail <= (free_list_tail + 1) % PHYS_REG_COUNT;
                                end
                                
                                rob[rob_head].valid <= 0;
                                rob_head <= (rob_head + 1) % ROB_ENTRIES;
                            end
                            break;
                        end
                    end
                end else begin
                    // Commit non-store instruction
                    // Free old physical register if needed
                    if (rob[rob_head].logical_rd != 0) begin
                        free_list[free_list_tail] <= rename_table[rob[rob_head].logical_rd];
                        free_list_tail <= (free_list_tail + 1) % PHYS_REG_COUNT;
                    end
                    
                    rob[rob_head].valid <= 0;
                    rob_head <= (rob_head + 1) % ROB_ENTRIES;
                end
            end
        end
    end

endmodule