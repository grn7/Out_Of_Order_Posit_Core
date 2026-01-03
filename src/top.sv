import general_defines::*;

module top (
    input logic clk,
    input logic rst
);

    logic [INSTR_MEM_IDX_W-1:0] if_pc;           // IF stage PC
    logic [INT_DATA_W-1:0]      if_instr;        // fetched instruction
    logic                       if_valid;        // IF stage output valid

    logic                       if_stall;        // stall IF stage
    logic                       if_flush;        // flush IF stage
    logic [INSTR_MEM_IDX_W-1:0] if_flush_pc;     // flush target PC

    logic                       imem_req_valid;
    logic [INSTR_MEM_IDX_W-1:0] imem_req_addr;
    logic                       imem_resp_valid;
    logic [INT_DATA_W-1:0]      imem_resp_data;

    logic                       bp_valid;        // branch prediction valid
    logic [INSTR_MEM_IDX_W-1:0] bp_target;       // predicted target
    logic                       btb_hit;         // BTB hit signal
    logic [INSTR_MEM_IDX_W-1:0] btb_target;      // BTB target


    logic            issue_valid;             // issue enable (stub)
    logic            issue_is_load;           // load op
    logic            issue_is_store;          // store op
    logic [ROB_IDX_W-1:0] issue_rob;          // rob index
    logic [INT_DATA_W-1:0] issue_addr;        // memory address
    logic [INT_DATA_W-1:0] issue_store_data;  // store data

    logic            wb_valid;                // writeback valid
    logic [ROB_IDX_W-1:0] wb_rob;             // writeback rob index
    logic [INT_DATA_W-1:0] wb_data;           // writeback data

    logic            commit_store;             // store commit from rob
    logic [ROB_IDX_W-1:0] commit_rob;          // committing rob entry

    logic            mem_rd_valid;             // memory read request
    logic [INT_DATA_W-1:0] mem_rd_addr;        // memory read address
    logic            mem_rd_resp;              // memory read response
    logic [INT_DATA_W-1:0] mem_rd_data;        // memory read data

    logic            mem_wr_valid;             // memory write enable
    logic [INT_DATA_W-1:0] mem_wr_addr;        // memory write address
    logic [INT_DATA_W-1:0] mem_wr_data;        // memory write data

    assign if_stall = 1'b0;                   // no stall for now
    assign if_flush = 1'b0;                   // no flush for now
    assign if_flush_pc = '0;                  // flush PC stub
    assign imem_resp_valid = 1'b1;            // always ready for now
    assign imem_resp_data = '0;               // instruction memory stub

    assign bp_valid = btb_hit;                // use BTB hit as prediction valid
    assign bp_target = btb_target;            // use BTB target


    // Instruction Fetch Stage
    IF if_stage (
        .clk              (clk),
        .rst              (rst),
        .pred_valid       (bp_valid),
        .pred_target      (bp_target),
        .stall            (if_stall),
        .flush            (if_flush),
        .flush_pc         (if_flush_pc),
        .imem_req_valid   (imem_req_valid),
        .imem_req_addr    (imem_req_addr),
        .imem_resp_valid  (imem_resp_valid),
        .imem_resp_data   (imem_resp_data),
        .if_valid         (if_valid),
        .if_pc            (if_pc),
        .if_instr         (if_instr)
    );

    // Branch Predictor
    bp bp_u (
        .clk          (clk),
        .rst          (rst),
        .fetch_pc     (if_pc),
        .actual_taken (1'b0),                 // EX feedback stub
        .update_valid (1'b0),                 // EX feedback stub
        .pred_taken   ()                      // connect when needed
    );

    // Branch Target Buffer
    btb btb_u (
        .clk    (clk),
        .rst    (rst),
        .pc     (if_pc),
        .hit    (btb_hit),
        .target (btb_target)
    );

    // Reorder Buffer
    rob rob_u (
        .clk          (clk),
        .rst          (rst),
        .wb_valid     (wb_valid),
        .wb_rob       (wb_rob),
        .wb_data      (wb_data),
        .commit_store (commit_store),
        .commit_rob   (commit_rob)
    );

    lsu lsu_u (
        .clk              (clk),
        .rst              (rst),
        .issue_valid      (issue_valid),
        .issue_is_load    (issue_is_load),
        .issue_is_store   (issue_is_store),
        .issue_rob        (issue_rob),
        .issue_addr       (issue_addr),
        .issue_store_data (issue_store_data),
        .commit_store     (commit_store),
        .commit_rob       (commit_rob),
        .wb_valid         (wb_valid),
        .wb_rob           (wb_rob),
        .wb_data          (wb_data),
        .mem_rd_valid     (mem_rd_valid),
        .mem_rd_addr      (mem_rd_addr),
        .mem_rd_resp      (mem_rd_resp),
        .mem_rd_data      (mem_rd_data),
        .mem_wr_valid     (mem_wr_valid),
        .mem_wr_addr      (mem_wr_addr),
        .mem_wr_data      (mem_wr_data)
    );

    mem mem_u (
        .clk      (clk),
        .rst      (rst),
        .rd_valid (mem_rd_valid),
        .rd_addr  (mem_rd_addr),
        .rd_resp  (mem_rd_resp),
        .rd_data  (mem_rd_data),
        .wr_valid (mem_wr_valid),
        .wr_addr  (mem_wr_addr),
        .wr_data  (mem_wr_data)
    );

    assign issue_valid      = 1'b0;            // decode/issue stub
    assign issue_is_load    = 1'b0;            // decode stub
    assign issue_is_store   = 1'b0;            // decode stub
    assign issue_rob        = '0;              // rename stub
    assign issue_addr       = '0;              // execute stub
    assign issue_store_data = '0;              // execute stub

    //ROB
    rob_entry_t rob [0:ROB_LENGTH-1];
    logic [ROB_IDX_W-1:0] rob_head,rob_tail;
    logic rob_full, rob_empty;

    //Register renaming 
    logic [ARCH_REG_IDX_W-1:0] rename_table [0:ARCH_REG_LENGTH-1]; //maps logical to physical register
    logic [ARCH_REG_IDX_W-1:0] old_rename_table [0:ARCH_REG_LENGTH-1]; //for recovery
    logic [PHYS_REG_IDX_W-1:0] free_list [0:PHYS_REG_LENGTH-1];
    logic [PHYS_REG_IDX_W-1:0] free_list_head, free_list_tail;
    logic free_list_empty;

    //IQs
    iq_entry_t int_iq [0:IQ_LENGTH-1];
    iq_entry_t mem_iq [0:IQ_LENGTH-1];
    iq_entry_t fp_iq [0:IQ_LENGTH-1];
    logic [IQ_LENGTH-1:0] int_iq_head, int_iq_tail;
    logic [IQ_LENGTH-1:0] mem_iq_head, mem_iq_tail;
    logic [IQ_LENGTH-1:0] fp_iq_head, fp_iq_tail;
    logic int_iq_full, mem_iq_full, fp_iq_full;

    //Physical reg file
    logic [INT_DATA_W-1:0] phys_reg_file [0:PHYS_REG_LENGTH-1];
    logic phys_reg_valid [0:PHYS_REG_LENGTH-1];

    //Functional units 
    logic [INT_DATA_W-1:0] alu_result;
    logic alu_valid;
    logic [PHYS_REG_IDX_W-1:0] alu_dest;

    //Bypass unit
    bypass_entry_t bypass [0:BYPASS_LENGTH-1]; 

    //Control signals
    logic decode_stall, rename_stall, dispatch_stall;

    function bypass_entry_t get_default_bypass();
        get_default_bypass.phys_rd = '0;
        get_default_bypass.result = '0;
        get_default_bypass.valid = 1'b0;
    endfunction

    //Status flags
    assign rob_full = ((rob_tail+1)%ROB_LENGTH == rob_head) && rob[rob_tail].valid;
    assign rob_empty = (rob_head == rob_tail) && !rob[rob_head].valid;
    assign free_list_empty = (free_list_head == free_list_tail);
    assign int_iq_full = ((int_iq_tail+1)%IQ_LENGTH == int_iq_head) && int_iq[int_iq_tail].valid;
    assign mem_iq_full = ((mem_iq_tail+1)%IQ_LENGTH == mem_iq_head) && mem_iq[mem_iq_tail].valid;
    assign fp_iq_full = ((fp_iq_tail+1)%IQ_LENGTH == fp_iq_head) && fp_iq[fp_iq_tail].valid;

    int idx;

    //Decode / Rename / Dispatch stage
    logic [31:0] current_inst;
    logic [PHYS_REG_IDX_W-1:0] rs1, rs2, rd;
    logic [INT_DATA_W-1:0] immediate;
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic inst_valid;
    // if_valid instantiated in the if module tells whether the instruction is valid or not

    // instantiate the decoder 
    ID decoder (
        .instr(current_inst),
        .opcode(opcode),
        .rd(rd),
        .funct3(funct3),
        .rs1(rs1),
        .rs2(rs2),
        .funct7(funct7),
        .immediate(immediate)
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            // Initialize rename table to identity mapping
            for (int i = 0; i < ARCH_REG_LENGTH; i++) begin
                rename_table[i] <= i;
            end
    
            // Initialize free list
            for (int i = ARCH_REG_LENGTH; i < PHYS_REG_LENGTH; i++) begin
                free_list[i - ARCH_REG_LENGTH] <= i;
            end
            free_list_head <= 0;
            free_list_tail <= PHYS_REG_LENGTH - ARCH_REG_LENGTH;
            
            // Initialize issue queues
            for (int i = 0; i < IQ_LENGTH; i++) begin
                int_iq[i] <= '0;
                mem_iq[i] <= '0;
                fp_iq[i] <= '0;
            end
            int_iq_head <= 0; int_iq_tail <= 0;
            mem_iq_head <= 0; mem_iq_tail <= 0;
            fp_iq_head <= 0; fp_iq_tail <= 0;
            
            // Initialize ROB
            for (int i = 0; i < ROB_LENGTH; i++) begin
                rob[i] <= '0;
            end
            rob_head <= 0;
            rob_tail <= 0;
        end

        else if (!rob_full && !free_list_empty) begin

            // decode 
            current_inst = if_instr;
            inst_valid = if_valid;

            // Allocate ROB entry
            if (inst_valid && !rob_full) begin
                rob[rob_tail].pc <= if_pc; // pc given from IF stage to be sent to next stage
                rob[rob_tail].logical_rd <= rd; // what's the use of logical_rd ???
                rob[rob_tail].done <= 0;
                rob[rob_tail].valid <= 1;
                rob[rob_tail].opcode <= opcode;
                rob[rob_tail].funct3 <= funct3;
                rob[rob_tail].funct7 <= funct7;
                rob[rob_tail].is_load <= (opcode == `OPCODE_LOAD);
                rob[rob_tail].is_store <= (opcode == `OPCODE_STORE);
                rob[rob_tail].is_branch <= (opcode == `OPCODE_BRANCH);
                // what should i allocate to pred_taken and pred_target??
                
                // Register renaming for destination
                if (rd != 0 && !free_list_empty) begin 
                    rob[rob_tail].phys_rd <= free_list[free_list_head];
                    rename_table[rd] <= free_list[free_list_head];
                    free_list_head <= (free_list_head + 1) % PHYS_REG_LENGTH;
                end 
                else begin
                    rob[rob_tail].phys_rd <= 0; // if rd is zero, we make phys_rd of that ROB entry as 0
                end          

                // Dispatch to issue queue
                case (opcode)
                    `OPCODE_ARITH_I, `OPCODE_ARITH_R: begin
                        if (!int_iq_full) begin

                            int_iq[int_iq_tail].pc <= if_pc;
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
                            
                            int_iq_tail <= (int_iq_tail + 1) % IQ_LENGTH;
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
                            
                            mem_iq_tail <= (mem_iq_tail + 1) % IQ_LENGTH;
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
                            
                            fp_iq_tail <= (fp_iq_tail + 1) % IQ_LENGTH;
                        end
                    end
                endcase
                
                rob_tail <= (rob_tail + 1) % ROB_LENGTH;
                // fetch_head <= (fetch_head + 1) % 8;
            end
        end    

        // Wakeup logic - check bypass network for ready operands
        for (int i = 0; i < IQ_LENGTH; i++) begin

            // Check bypass for rs1
            if (int_iq[i].valid) begin
                if (!int_iq[i].rs1_ready) begin
                    for (int j = 0; j < BYPASS_LENGTH; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == int_iq[i].phys_rs1) begin
                            int_iq[i].rs1_value <= bypass[j].result;
                            int_iq[i].rs1_ready <= 1;
                        end
                    end
                end
                
                // Check bypass for rs2
                if (!int_iq[i].rs2_ready && int_iq[i].opcode == `OPCODE_ARITH_R) begin
                    for (int j = 0; j < BYPASS_LENGTH; j++) begin
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
                    for (int j = 0; j < BYPASS_LENGTH; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == mem_iq[i].phys_rs1) begin
                            mem_iq[i].rs1_value <= bypass[j].result;
                            mem_iq[i].rs1_ready <= 1;
                        end
                    end
                end
                
                if (!mem_iq[i].rs2_ready && mem_iq[i].opcode == `OPCODE_STORE) begin
                    for (int j = 0; j < BYPASS_LENGTH; j++) begin
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
                    for (int j = 0; j < BYPASS_LENGTH; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == fp_iq[i].phys_rs1) begin
                            fp_iq[i].rs1_value <= bypass[j].result;
                            fp_iq[i].rs1_ready <= 1;
                        end
                    end
                end
                
                if (!fp_iq[i].rs2_ready) begin
                    for (int j = 0; j < BYPASS_LENGTH; j++) begin
                        if (bypass[j].valid && bypass[j].phys_rd == fp_iq[i].phys_rs2) begin
                            fp_iq[i].rs2_value <= bypass[j].result;
                            fp_iq[i].rs2_ready <= 1;
                        end
                    end
                end
            end
        end
    end

endmodule
