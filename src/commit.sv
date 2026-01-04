import general_defines::*;

module commit (
    input logic clk,
    input logic rst,
    
    // ROB interface - read head entry
    input logic                        rob_head_valid,
    input logic                        rob_head_done,
    input logic [INSTR_MEM_IDX_W-1:0]  rob_head_pc,
    input logic [ARCH_REG_IDX_W-1:0]   rob_head_logical_rd,
    input logic [PHYS_REG_IDX_W-1:0]   rob_head_phys_rd,
    input logic [INT_DATA_W-1:0]       rob_head_result,
    input logic [6:0]                  rob_head_opcode,
    input logic [2:0]                  rob_head_funct3,
    input logic [6:0]                  rob_head_funct7,
    input logic                        rob_head_is_store,
    input logic                        rob_head_is_load,
    input logic                        rob_head_is_branch,
    input logic                        rob_head_pred_taken,
    input logic [INSTR_MEM_IDX_W-1:0]  rob_head_pred_target,
    input logic                        rob_head_branch_taken,
    input logic [INSTR_MEM_IDX_W-1:0]  rob_head_branch_target,
    
    // ROB control
    output logic                       rob_commit,
    output logic                       rob_advance_head,
    
    // Architectural register file (ARF) update
    output logic                       arf_we,
    output logic [ARCH_REG_IDX_W-1:0]  arf_waddr,
    output logic [INT_DATA_W-1:0]      arf_wdata,
    
    // Store commit to LSU
    output logic                       commit_store,
    output logic [ROB_IDX_W-1:0]       commit_rob_idx,
    
    // Branch misprediction handling
    output logic                       flush_pipeline,
    output logic [INSTR_MEM_IDX_W-1:0] redirect_pc,
    output logic                       update_bp,
    output logic [INSTR_MEM_IDX_W-1:0] update_bp_pc,
    output logic                       update_bp_taken,
    
    // Free list management (for recovery)
    output logic                       free_phys_reg,
    output logic [PHYS_REG_IDX_W-1:0]  freed_phys_reg,
    
    // Statistics/Debug
    output logic [63:0]                commit_count,
    output logic [63:0]                branch_mispredictions
);

    logic [63:0] commit_counter;
    logic [63:0] mispred_counter;
    
    logic misprediction_detected;
    logic can_commit;
    
    // Check if instruction at ROB head can commit
    assign can_commit = rob_head_valid && rob_head_done;
    
    // Detect branch misprediction
    always_comb begin
        misprediction_detected = 1'b0;
        
        if (can_commit && rob_head_is_branch) begin
            // Check if prediction matches actual outcome
            if (rob_head_pred_taken != rob_head_branch_taken) begin
                misprediction_detected = 1'b1;
            end else if (rob_head_branch_taken && 
                        (rob_head_pred_target != rob_head_branch_target)) begin
                misprediction_detected = 1'b1;
            end
        end
    end
    
    // Commit logic
    always_comb begin
        rob_commit = 1'b0;
        rob_advance_head = 1'b0;
        arf_we = 1'b0;
        arf_waddr = '0;
        arf_wdata = '0;
        commit_store = 1'b0;
        commit_rob_idx = '0;
        flush_pipeline = 1'b0;
        redirect_pc = '0;
        update_bp = 1'b0;
        update_bp_pc = '0;
        update_bp_taken = 1'b0;
        free_phys_reg = 1'b0;
        freed_phys_reg = '0;
        
        if (can_commit) begin
            rob_commit = 1'b1;
            rob_advance_head = 1'b1;
            
            // Update architectural register file if not x0
            if (rob_head_logical_rd != 0) begin
                arf_we = 1'b1;
                arf_waddr = rob_head_logical_rd;
                arf_wdata = rob_head_result;
            end
            
            // Commit store to memory
            if (rob_head_is_store) begin
                commit_store = 1'b1;
                // commit_rob_idx would be rob_head index (needs to be passed from top)
            end
            
            // Handle branch misprediction
            if (misprediction_detected) begin
                flush_pipeline = 1'b1;
                
                // Redirect to correct PC
                if (rob_head_branch_taken) begin
                    redirect_pc = rob_head_branch_target;
                end else begin
                    // Branch not taken, go to next sequential instruction
                    redirect_pc = rob_head_pc + INSTR_MEM_IDX_W'(1);
                end
            end
            
            // Update branch predictor
            if (rob_head_is_branch) begin
                update_bp = 1'b1;
                update_bp_pc = rob_head_pc;
                update_bp_taken = rob_head_branch_taken;
            end
            
            // Note: Free list management for old physical register
            // This would require tracking the old physical register mapping
            // which should come from ROB or rename table snapshot
        end
    end
    
    // Statistics counters
    always_ff @(posedge clk) begin
        if (rst) begin
            commit_counter <= 64'b0;
            mispred_counter <= 64'b0;
        end else begin
            if (rob_commit) begin
                commit_counter <= commit_counter + 1;
            end
            
            if (misprediction_detected) begin
                mispred_counter <= mispred_counter + 1;
            end
        end
    end
    
    assign commit_count = commit_counter;
    assign branch_mispredictions = mispred_counter;

endmodule
