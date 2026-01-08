import general_defines::*;

module top (
    input logic clk,
    input logic rst
);

    logic [INSTR_MEM_IDX_W-1:0] if_pc;           // IF stage PC (fetched)
    logic [INSTR_MEM_IDX_W-1:0] current_pc;      // Current PC being fetched (for BP/BTB)
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
    logic                       bp_pred_taken;   // branch predictor direction prediction
    logic [INSTR_MEM_IDX_W-1:0] bp_target;       // predicted target
    logic                       btb_hit;         // BTB hit signal
    logic [INSTR_MEM_IDX_W-1:0] btb_target;      // BTB target


    logic            issue_valid;             // issue enable from mem_iq
    logic            issue_is_load;           // load op
    logic            issue_is_store;          // store op
    logic [ROB_IDX_W-1:0] issue_rob;          // rob index
    logic [PHYS_REG_IDX_W-1:0] issue_phys_rd; // physical dest register
    logic [INT_DATA_W-1:0] issue_addr;        // memory address
    logic [INT_DATA_W-1:0] issue_store_data;  // store data

    // Execution results muxed from functional units (ALU/MUL/DIV)
    logic                        exec_valid;
    logic [ROB_IDX_W-1:0]        exec_rob_idx;
    logic [PHYS_REG_IDX_W-1:0]   exec_phys_rd;
    logic [INT_DATA_W-1:0]       exec_result;
    logic                        exec_is_branch;
    logic                        exec_branch_taken;
    logic [INSTR_MEM_IDX_W-1:0]  exec_branch_target;

    // Writeback stage outputs
    logic                        rob_wb_valid;
    logic [ROB_IDX_W-1:0]        rob_wb_idx;
    logic [INT_DATA_W-1:0]       rob_wb_result;
    logic                        rob_wb_is_branch;
    logic                        rob_wb_branch_taken;
    logic [INSTR_MEM_IDX_W-1:0]  rob_wb_branch_target;
    logic                        prf_we;
    logic [PHYS_REG_IDX_W-1:0]   prf_waddr;
    logic [INT_DATA_W-1:0]       prf_wdata;
    logic                        bypass_valid;
    logic [PHYS_REG_IDX_W-1:0]   bypass_phys_rd;
    logic [INT_DATA_W-1:0]       bypass_result;

    // LSU writeback (from existing LSU)
    logic            wb_valid;                // writeback valid
    logic [ROB_IDX_W-1:0] wb_rob;             // writeback rob index
    logic [PHYS_REG_IDX_W-1:0] wb_phys_rd;    // writeback phys_rd
    logic [INT_DATA_W-1:0] wb_data;           // writeback data

    // Commit stage outputs
    logic                        rob_commit;
    logic                        rob_advance_head;
    logic                        arf_we;
    logic [ARCH_REG_IDX_W-1:0]   arf_waddr;
    logic [INT_DATA_W-1:0]       arf_wdata;
    logic                        commit_store;
    logic [ROB_IDX_W-1:0]        commit_rob;
    logic                        flush_pipeline;
    logic [INSTR_MEM_IDX_W-1:0]  redirect_pc;
    logic                        update_bp;
    logic [INSTR_MEM_IDX_W-1:0]  update_bp_pc;
    logic                        update_bp_taken;
    logic                        free_phys_reg;
    logic [PHYS_REG_IDX_W-1:0]   freed_phys_reg;
    
    // ROB head entry signals for commit
    logic                        rob_head_valid;
    logic                        rob_head_done;
    logic [INSTR_MEM_IDX_W-1:0]  rob_head_pc;
    logic [ARCH_REG_IDX_W-1:0]   rob_head_logical_rd;
    logic [PHYS_REG_IDX_W-1:0]   rob_head_phys_rd;
    logic [PHYS_REG_IDX_W-1:0]   rob_head_old_phys_rd;
    logic [INT_DATA_W-1:0]       rob_head_result;
    logic [6:0]                  rob_head_opcode;
    logic [2:0]                  rob_head_funct3;
    logic [6:0]                  rob_head_funct7;
    logic                        rob_head_is_store;
    logic                        rob_head_is_load;
    logic                        rob_head_is_branch;
    logic                        rob_head_pred_taken;
    logic [INSTR_MEM_IDX_W-1:0]  rob_head_pred_target;
    logic                        rob_head_branch_taken;
    logic [INSTR_MEM_IDX_W-1:0]  rob_head_branch_target;

    logic            mem_rd_valid;             // memory read request
    logic [INT_DATA_W-1:0] mem_rd_addr;        // memory read address
    logic            mem_rd_resp;              // memory read response
    logic [INT_DATA_W-1:0] mem_rd_data;        // memory read data

    logic            mem_wr_valid;             // memory write enable
    logic [INT_DATA_W-1:0] mem_wr_addr;        // memory write address
    logic [INT_DATA_W-1:0] mem_wr_data;        // memory write data

    assign if_stall = 1'b0;                   // no stall for now
    assign if_flush = flush_pipeline;         // flush from commit stage
    assign if_flush_pc = redirect_pc;         // redirect PC from commit
    assign imem_resp_valid = 1'b1;            // always ready for now
    assign imem_resp_data = '0;               // instruction memory not connected - needs real imem module

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
        .if_instr         (if_instr),
        .current_pc       (current_pc)
    );

    // Branch Predictor
    bp bp_u (
        .clk          (clk),
        .rst          (rst),
        .fetch_pc     (current_pc),           // Use current PC being fetched
        .actual_taken (update_bp_taken),      // From commit stage
        .update_valid (update_bp),            // From commit stage
        .pred_taken   (bp_pred_taken)         // Direction prediction output
    );

    // Branch Target Buffer
    btb btb_u (
        .clk          (clk),
        .rst          (rst),
        .fetch_pc     (current_pc),           // Use current PC being fetched
        .btb_hit      (btb_hit),
        .btb_target   (btb_target),
        .update_valid (update_bp && rob_head_is_branch),  // Update BTB on branch commit
        .update_pc    (update_bp_pc),                     // Branch PC from commit
        .update_target(rob_head_branch_target)            // Actual branch target
    );

    // Writeback Stage
    writeback wb_stage (
        .clk                    (clk),
        .rst                    (rst),
        .exec_valid             (exec_valid),
        .exec_rob_idx           (exec_rob_idx),
        .exec_phys_rd           (exec_phys_rd),
        .exec_result            (exec_result),
        .exec_is_branch         (exec_is_branch),
        .exec_branch_taken      (exec_branch_taken),
        .exec_branch_target     (exec_branch_target),
        .lsu_wb_valid           (wb_valid),
        .lsu_wb_rob             (wb_rob),
        .lsu_wb_phys_rd         (wb_phys_rd),
        .lsu_wb_data            (wb_data),
        .rob_wb_valid           (rob_wb_valid),
        .rob_wb_idx             (rob_wb_idx),
        .rob_wb_result          (rob_wb_result),
        .rob_wb_is_branch       (rob_wb_is_branch),
        .rob_wb_branch_taken    (rob_wb_branch_taken),
        .rob_wb_branch_target   (rob_wb_branch_target),
        .prf_we                 (prf_we),
        .prf_waddr              (prf_waddr),
        .prf_wdata              (prf_wdata),
        .bypass_valid           (bypass_valid),
        .bypass_phys_rd         (bypass_phys_rd),
        .bypass_result          (bypass_result)
    );

    // Commit Stage
    commit commit_stage (
        .clk                     (clk),
        .rst                     (rst),
        .rob_head_idx            (rob_head),
        .rob_head_valid          (rob_head_valid),
        .rob_head_done           (rob_head_done),
        .rob_head_pc             (rob_head_pc),
        .rob_head_logical_rd     (rob_head_logical_rd),
        .rob_head_phys_rd        (rob_head_phys_rd),
        .rob_head_old_phys_rd    (rob_head_old_phys_rd),
        .rob_head_result         (rob_head_result),
        .rob_head_opcode         (rob_head_opcode),
        .rob_head_funct3         (rob_head_funct3),
        .rob_head_funct7         (rob_head_funct7),
        .rob_head_is_store       (rob_head_is_store),
        .rob_head_is_load        (rob_head_is_load),
        .rob_head_is_branch      (rob_head_is_branch),
        .rob_head_pred_taken     (rob_head_pred_taken),
        .rob_head_pred_target    (rob_head_pred_target),
        .rob_head_branch_taken   (rob_head_branch_taken),
        .rob_head_branch_target  (rob_head_branch_target),
        .rob_commit              (rob_commit),
        .rob_advance_head        (rob_advance_head),
        .arf_we                  (arf_we),
        .arf_waddr               (arf_waddr),
        .arf_wdata               (arf_wdata),
        .commit_store            (commit_store),
        .commit_rob_idx          (commit_rob),
        .flush_pipeline          (flush_pipeline),
        .redirect_pc             (redirect_pc),
        .update_bp               (update_bp),
        .update_bp_pc            (update_bp_pc),
        .update_bp_taken         (update_bp_taken),
        .free_phys_reg           (free_phys_reg),
        .freed_phys_reg          (freed_phys_reg)
    );
    
    // Execution results - mux between different FUs
    always_comb begin
        // Priority: ALU (single cycle) > Multiplier > Divider
        if (alu_valid) begin
            exec_valid = 1'b1;
            exec_rob_idx = alu_rob_idx;
            exec_phys_rd = alu_dest;
            exec_result = alu_result;
            exec_is_branch = 1'b0;
            exec_branch_taken = 1'b0;
            exec_branch_target = '0;
        end else begin
            exec_valid = 1'b0;
            exec_rob_idx = '0;
            exec_phys_rd = '0;
            exec_result = '0;
            exec_is_branch = 1'b0;
            exec_branch_taken = 1'b0;
            exec_branch_target = '0;
        end
    end

    lsu lsu_u (
        .clk              (clk),
        .rst              (rst),
        .issue_valid      (issue_valid),
        .issue_is_load    (issue_is_load),
        .issue_is_store   (issue_is_store),
        .issue_rob        (issue_rob),
        .issue_phys_rd    (issue_phys_rd),
        .issue_addr       (issue_addr),
        .issue_store_data (issue_store_data),
        .commit_store     (commit_store),
        .commit_rob       (commit_rob),
        .wb_valid         (wb_valid),
        .wb_rob           (wb_rob),
        .wb_phys_rd       (wb_phys_rd),
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

    // Issue signals driven by mem_iq execution logic 

    //ROB
    rob_entry_t rob [0:ROB_LENGTH-1];
    logic [ROB_IDX_W-1:0] rob_head,rob_tail;
    logic rob_full, rob_empty;
    
    // Extract ROB head entry for commit stage
    assign rob_head_valid = rob[rob_head].valid;
    assign rob_head_done = rob[rob_head].done;
    assign rob_head_pc = rob[rob_head].pc;
    assign rob_head_logical_rd = rob[rob_head].logical_rd;
    assign rob_head_phys_rd = rob[rob_head].phys_rd;
    assign rob_head_old_phys_rd = rob[rob_head].old_phys_rd;
    assign rob_head_result = rob[rob_head].result;
    assign rob_head_opcode = rob[rob_head].opcode;
    assign rob_head_funct3 = rob[rob_head].funct3;
    assign rob_head_funct7 = rob[rob_head].funct7;
    assign rob_head_is_store = rob[rob_head].is_store;
    assign rob_head_is_load = rob[rob_head].is_load;
    assign rob_head_is_branch = rob[rob_head].is_branch;
    assign rob_head_pred_taken = rob[rob_head].pred_taken;
    assign rob_head_pred_target = rob[rob_head].pred_target;
    assign rob_head_branch_taken = rob[rob_head].branch_taken;
    assign rob_head_branch_target = rob[rob_head].branch_target;

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
    logic [ROB_IDX_W-1:0] alu_rob_idx;
    
    // Track ROB indices for multi-cycle operations
    logic [ROB_IDX_W-1:0] mul_rob_idx;
    logic [ROB_IDX_W-1:0] div_rob_idx;
    
    // Control signals for issue
    logic issued;        // For int_iq
    logic mem_issued;    // For mem_iq

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
            
            // Initialize execution tracking
            alu_valid <= 1'b0;
            alu_dest <= '0;
            alu_rob_idx <= '0;
            mul_rob_idx <= '0;
            div_rob_idx <= '0;
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
            
            // Initialize physical register file and valid bits
            for (int i = 0; i < PHYS_REG_LENGTH; i++) begin
                phys_reg_file[i] <= '0;
                phys_reg_valid[i] <= (i < ARCH_REG_LENGTH) ? 1'b1 : 1'b0; // Architectural regs valid initially
            end
            
            // Initialize ROB
            for (int i = 0; i < ROB_LENGTH; i++) begin
                rob[i] <= '0;
                rob[i].branch_taken <= 1'b0;
                rob[i].branch_target <= '0;
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
                rob[rob_tail].logical_rd <= rd; // Architectural register for ARF commit and old phys_rd lookup
                rob[rob_tail].done <= 0;
                rob[rob_tail].valid <= 1;
                rob[rob_tail].opcode <= opcode;
                rob[rob_tail].funct3 <= funct3;
                rob[rob_tail].funct7 <= funct7;
                rob[rob_tail].is_load <= (opcode == `OPCODE_LOAD);
                rob[rob_tail].is_store <= (opcode == `OPCODE_STORE);
                rob[rob_tail].is_branch <= (opcode == `OPCODE_BRANCH);
                
                // Store branch prediction info for misprediction detection
                if (opcode == `OPCODE_BRANCH) begin
                    rob[rob_tail].pred_taken <= bp_valid;  // Store prediction: 1=predicted taken, 0=predicted not taken
                    rob[rob_tail].pred_target <= bp_valid ? bp_target : '0;
                end else begin
                    rob[rob_tail].pred_taken <= 1'b0;
                    rob[rob_tail].pred_target <= '0;
                end
                
                // Register renaming for destination
                if (rd != 0 && !free_list_empty) begin
                    rob[rob_tail].old_phys_rd <= rename_table[rd];  // Save old mapping for free list return
                    rob[rob_tail].phys_rd <= free_list[free_list_head];
                    rename_table[rd] <= free_list[free_list_head];
                    free_list_head <= (free_list_head + 1) % PHYS_REG_LENGTH;
                end 
                else begin
                    rob[rob_tail].old_phys_rd <= 0;  // No old mapping for x0
                    rob[rob_tail].phys_rd <= 0; // if rd is zero, we make phys_rd of that ROB entry as 0
                end          

                // Dispatch to issue queue
                case (opcode)
                    `OPCODE_ARITH_I, `OPCODE_ARITH_R: begin
                        if (!int_iq_full) begin

                            int_iq[int_iq_tail].pc <= if_pc;
                            int_iq[int_iq_tail].phys_rd <= rob[rob_tail].phys_rd;  // Use allocated phys_rd from ROB
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
            end
        end    
        
        // Physical register file write from writeback stage
        if (prf_we) begin
            phys_reg_file[prf_waddr] <= prf_wdata;
            phys_reg_valid[prf_waddr] <= 1'b1;
        end

        // use bypass network from writeback for ready operands
        // This creates combinational forwarding paths
        for (int i = 0; i < IQ_LENGTH; i++) begin
            // Check bypass for integer IQ
            if (int_iq[i].valid) begin
                if (!int_iq[i].rs1_ready && bypass_valid && 
                    (bypass_phys_rd == int_iq[i].phys_rs1)) begin
                    int_iq[i].rs1_value <= bypass_result;
                    int_iq[i].rs1_ready <= 1'b1;
                end
                
                if (!int_iq[i].rs2_ready && bypass_valid && 
                    (bypass_phys_rd == int_iq[i].phys_rs2) && 
                    (int_iq[i].opcode == `OPCODE_ARITH_R)) begin
                    int_iq[i].rs2_value <= bypass_result;
                    int_iq[i].rs2_ready <= 1'b1;
                end
            end
            
            // Check bypass for memory IQ
            if (mem_iq[i].valid) begin
                if (!mem_iq[i].rs1_ready && bypass_valid && 
                    (bypass_phys_rd == mem_iq[i].phys_rs1)) begin
                    mem_iq[i].rs1_value <= bypass_result;
                    mem_iq[i].rs1_ready <= 1'b1;
                end
                
                if (!mem_iq[i].rs2_ready && bypass_valid && 
                    (bypass_phys_rd == mem_iq[i].phys_rs2) && 
                    (mem_iq[i].opcode == `OPCODE_STORE)) begin
                    mem_iq[i].rs2_value <= bypass_result;
                    mem_iq[i].rs2_ready <= 1'b1;
                end
            end
            
            // Check bypass for FP IQ
            if (fp_iq[i].valid) begin
                if (!fp_iq[i].rs1_ready && bypass_valid && 
                    (bypass_phys_rd == fp_iq[i].phys_rs1)) begin
                    fp_iq[i].rs1_value <= bypass_result;
                    fp_iq[i].rs1_ready <= 1'b1;
                end
                
                if (!fp_iq[i].rs2_ready && bypass_valid && 
                    (bypass_phys_rd == fp_iq[i].phys_rs2)) begin
                    fp_iq[i].rs2_value <= bypass_result;
                    fp_iq[i].rs2_ready <= 1'b1;
                end
            end
        end
    end
        // EX stage

        // Instantiate the required modules

        logic [INT_DATA_W-1:0] adder_a;
        logic [INT_DATA_W-1:0] adder_b;
        logic adder_valid_i;
        logic [INT_DATA_W-1:0] adder_result;
        logic adder_valid_o;

        adder int_adder(
            .clk(clk),
            .rst(rst),
            .a(adder_a),
            .b(adder_b),
            .valid_i(adder_valid_i),
            .result(adder_result),
            .valid_o(adder_valid_o)
        );

        logic [INT_DATA_W-1:0] sub_a;
        logic [INT_DATA_W-1:0] sub_b;
        logic sub_valid_i;
        logic [INT_DATA_W-1:0] sub_result;
        logic sub_valid_o;

        subtractor int_subtractor(
            .clk(clk),
            .rst(rst),
            .a(sub_a),
            .b(sub_b),
            .valid_i(sub_valid_i),
            .result(sub_result),
            .valid_o(sub_valid_o)
        );

        logic [INT_DATA_W-1:0] mul_a;
        logic [INT_DATA_W-1:0] mul_b;
        logic mul_valid_i;
        logic [INT_DATA_W-1:0] mul_result;
        logic mul_busy;

        multiplier int_multiplier(
            .clk(clk),
            .rst(rst),
            .a(mul_a),
            .b(mul_b),
            .valid_i(mul_valid_i),
            .result(mul_result),
            .busy(mul_busy)
        );

        logic [INT_DATA_W-1:0] div_a;
        logic [INT_DATA_W-1:0] div_b;
        logic div_valid_i;
        logic [INT_DATA_W-1:0] div_result;
        logic div_busy;

        divider int_divider(
            .clk(clk),
            .rst(rst),
            .a(div_a),
            .b(div_b),
            .valid_i(div_valid_i),
            .result(div_result),
            .busy(div_busy)
        );

        //start actual implentation of EX stage
        logic [PHYS_REG_IDX_W-1:0] mul_dest; 
        logic [PHYS_REG_IDX_W-1:0] div_dest;
        
        // Capture ALU results (single-cycle operations)
        always_ff @(posedge clk) begin
            if (rst) begin
                alu_valid <= 1'b0;
                alu_result <= '0;
            end else begin
                // ALU valid when adder or subtractor completes
                if (adder_valid_o) begin
                    alu_valid <= 1'b1;
                    alu_result <= adder_result;
                end else if (sub_valid_o) begin
                    alu_valid <= 1'b1;
                    alu_result <= sub_result;
                end else begin
                    alu_valid <= 1'b0;
                end
            end
        end
        
        // Issue logic from int_iq         
        always_ff @(posedge clk) begin
            if(rst) begin
                adder_valid_i <= 'b0;
                sub_valid_i <= 'b0;
                mul_valid_i <= 'b0;
                div_valid_i <= 'b0;
                issued <= 1'b0;
            end

            else begin
                issued <= 1'b0; // to replace break as not supported some times
                adder_valid_i <= 'b0;
                sub_valid_i <= 'b0;
                mul_valid_i <= 'b0;
                div_valid_i <= 'b0;                
                for(int i = 0; i < IQ_LENGTH; i++) begin
                    automatic int local_idx = (int_iq_head + i) % IQ_LENGTH;
                    if(!issued && int_iq[local_idx].valid && int_iq[local_idx].rs1_ready &&
                    (int_iq[local_idx].opcode == `OPCODE_ARITH_I || int_iq[local_idx].rs2_ready)) begin

                        case(int_iq[local_idx].fu_type)
                            0: begin // for add, sub and other operations can be added
                                case(int_iq[local_idx].opcode)
                                    `OPCODE_ARITH_I: begin
                                        case(int_iq[local_idx].funct3)
                                            3'b000: begin
                                                    adder_valid_i <= 1'b1;
                                                    adder_a <= int_iq[local_idx].rs1_value;
                                                    adder_b <= int_iq[local_idx].imm;
                                                    // ADDI 
                                            end
                                            default: begin
                                                     adder_valid_i <= 1'b1;
                                                     adder_a <= int_iq[local_idx].rs1_value;
                                                     adder_b <= int_iq[local_idx].imm;
                                            end
                                        endcase
                                    end
                                    `OPCODE_ARITH_R: begin
                                        case(int_iq[local_idx].funct3)
                                            3'b000: begin
                                                if(int_iq[local_idx].funct7 == 7'b0) begin
                                                    adder_valid_i <= 1'b1;
                                                    adder_a <= int_iq[local_idx].rs1_value;
                                                    adder_b <= int_iq[local_idx].rs2_value;
                                                    // ADD
                                                end
                                                else begin
                                                    sub_valid_i <= 1'b1;
                                                    sub_a <= int_iq[local_idx].rs1_value;
                                                    sub_b <= int_iq[local_idx].rs2_value;
                                                    // SUB
                                                end
                                            end
                                            default: begin
                                                     adder_valid_i <= 1'b1;
                                                     adder_a <= int_iq[local_idx].rs1_value;
                                                     adder_b <= int_iq[local_idx].rs2_value;
                                            end
                                        endcase
                                    end
                                    default: begin
                                             adder_valid_i <= 1'b0;
                                             sub_valid_i <= 1'b0;
                                    end
                                endcase
                                alu_dest <= int_iq[local_idx].phys_rd;
                                alu_rob_idx <= int_iq[local_idx].rob_idx;
                                int_iq[local_idx].valid <= 1'b0;
                                issued <= 1'b1;
                            end

                            2: begin //MUL
                                if(!mul_busy) begin
                                    mul_valid_i <= 1'b1;
                                    mul_a <= int_iq[local_idx].rs1_value;
                                    mul_b <= int_iq[local_idx].rs2_value;
                                    mul_dest <= int_iq[local_idx].phys_rd;
                                    mul_rob_idx <= int_iq[local_idx].rob_idx;
                                    int_iq[local_idx].valid <= 1'b0;
                                    issued <= 1'b1;
                                end
                            end

                            3: begin //DIV
                                if(!div_busy) begin
                                    div_valid_i <= 1'b1;
                                    div_a <= int_iq[local_idx].rs1_value;
                                    div_b <= int_iq[local_idx].rs2_value;
                                    div_dest <= int_iq[local_idx].phys_rd;
                                    div_rob_idx <= int_iq[local_idx].rob_idx;
                                    int_iq[local_idx].valid <= 1'b0;
                                    issued <= 1'b1;
                                end
                            end
                        endcase
                    end
                end
            end
        end
        
        // Memory IQ execution - issue to LSU
        // Issue logic from mem_iq
    // end of main always_ff block
    
    // Separate always block for mem_iq execution
    always_ff @(posedge clk) begin
        if (rst) begin
            issue_valid <= 1'b0;
            issue_is_load <= 1'b0;
            issue_is_store <= 1'b0;
            issue_rob <= '0;
            issue_phys_rd <= '0;
            issue_addr <= '0;
            issue_store_data <= '0;
            mem_issued <= 1'b0;
        end else begin
            mem_issued <= 1'b0;
            issue_valid <= 1'b0;
            
            // Select ready entry from mem_iq to issue to LSU
            for (int i = 0; i < IQ_LENGTH; i++) begin
                automatic int local_idx = (mem_iq_head + i) % IQ_LENGTH;
                if (!mem_issued && mem_iq[local_idx].valid && mem_iq[local_idx].rs1_ready &&
                    (mem_iq[local_idx].opcode == `OPCODE_LOAD || mem_iq[local_idx].rs2_ready)) begin
                    
                    issue_valid <= 1'b1;
                    issue_is_load <= (mem_iq[local_idx].opcode == `OPCODE_LOAD);
                    issue_is_store <= (mem_iq[local_idx].opcode == `OPCODE_STORE);
                    issue_rob <= mem_iq[local_idx].rob_idx;
                    issue_phys_rd <= mem_iq[local_idx].phys_rd;
                    
                    // Calculate address: base + offset
                    issue_addr <= mem_iq[local_idx].rs1_value + mem_iq[local_idx].imm;
                    
                    // Store data from rs2
                    if (mem_iq[local_idx].opcode == `OPCODE_STORE) begin
                        issue_store_data <= mem_iq[local_idx].rs2_value;
                    end else begin
                        issue_store_data <= '0;
                    end
                    
                    mem_iq[local_idx].valid <= 1'b0;
                    mem_issued <= 1'b1;
                end
            end
        end
    end
    
    // Main pipeline always block for writeback/commit/flush
    always_ff @(posedge clk) begin
        if (rst) begin
            // Reset handled in first always block
        end else begin
            // Writeback to ROB - mark entries as done and update results
            if (rob_wb_valid) begin
                rob[rob_wb_idx].done <= 1'b1;
                rob[rob_wb_idx].result <= rob_wb_result;
                if (rob_wb_is_branch) begin
                    rob[rob_wb_idx].branch_taken <= rob_wb_branch_taken;
                    rob[rob_wb_idx].branch_target <= rob_wb_branch_target;
                end
            end
            
            // Commit stage - advance ROB head
            if (rob_advance_head && !flush_pipeline) begin
                rob[rob_head].valid <= 1'b0;  // Mark as committed
                rob_head <= (rob_head + 1) % ROB_LENGTH;
                
                // Return freed physical register to free list
                if (free_phys_reg) begin
                    free_list[free_list_tail] <= freed_phys_reg;
                    free_list_tail <= (free_list_tail + 1) % PHYS_REG_LENGTH;
                end
            end
            
            // Flush pipeline on misprediction 
            if (flush_pipeline) begin
                // Clear valid bit for all ROB entries except current head
                for (int i = 0; i < ROB_LENGTH; i++) begin
                    // Only keep the head entry, clear all others
                    rob[i].valid <= (i == rob_head) ? rob[i].valid : 1'b0;
                end
                // Reset tail to head+1 to start fresh allocation
                rob_tail <= (rob_head + 1) % ROB_LENGTH;
                
                // Clear all issue queue entries
                for (int i = 0; i < IQ_LENGTH; i++) begin
                    int_iq[i].valid <= 1'b0;
                    mem_iq[i].valid <= 1'b0;
                    fp_iq[i].valid <= 1'b0;
                end
            end
        end
    end

endmodule
