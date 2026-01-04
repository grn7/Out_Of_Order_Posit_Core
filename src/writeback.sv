import general_defines::*;

module writeback (
    input logic clk,
    input logic rst,
    
    // Execution unit results (can be extended for multiple FUs)
    input logic                        exec_valid,
    input logic [ROB_IDX_W-1:0]        exec_rob_idx,
    input logic [PHYS_REG_IDX_W-1:0]   exec_phys_rd,
    input logic [INT_DATA_W-1:0]       exec_result,
    input logic                        exec_is_branch,
    input logic                        exec_branch_taken,
    input logic [INSTR_MEM_IDX_W-1:0]  exec_branch_target,
    
    // LSU writeback interface
    input logic                        lsu_wb_valid,
    input logic [ROB_IDX_W-1:0]        lsu_wb_rob,
    input logic [PHYS_REG_IDX_W-1:0]   lsu_wb_phys_rd,
    input logic [INT_DATA_W-1:0]       lsu_wb_data,
    
    // ROB write interface
    output logic                       rob_wb_valid,
    output logic [ROB_IDX_W-1:0]       rob_wb_idx,
    output logic [INT_DATA_W-1:0]      rob_wb_result,
    output logic                       rob_wb_is_branch,
    output logic                       rob_wb_branch_taken,
    output logic [INSTR_MEM_IDX_W-1:0] rob_wb_branch_target,
    
    // Physical register file write
    output logic                       prf_we,
    output logic [PHYS_REG_IDX_W-1:0]  prf_waddr,
    output logic [INT_DATA_W-1:0]      prf_wdata,
    
    // Bypass network (forwarding)
    output logic                       bypass_valid,
    output logic [PHYS_REG_IDX_W-1:0]  bypass_phys_rd,
    output logic [INT_DATA_W-1:0]      bypass_result
);

    // Arbitration: LSU has priority over execution units for writeback
    logic arb_lsu_grant;
    logic arb_exec_grant;
    
    always_comb begin
        arb_lsu_grant = lsu_wb_valid;
        arb_exec_grant = exec_valid && !lsu_wb_valid;
    end
    
    // Writeback to ROB
    always_comb begin
        if (arb_lsu_grant) begin
            rob_wb_valid = 1'b1;
            rob_wb_idx = lsu_wb_rob;
            rob_wb_result = lsu_wb_data;
            rob_wb_is_branch = 1'b0;
            rob_wb_branch_taken = 1'b0;
            rob_wb_branch_target = '0;
        end else if (arb_exec_grant) begin
            rob_wb_valid = 1'b1;
            rob_wb_idx = exec_rob_idx;
            rob_wb_result = exec_result;
            rob_wb_is_branch = exec_is_branch;
            rob_wb_branch_taken = exec_branch_taken;
            rob_wb_branch_target = exec_branch_target;
        end else begin
            rob_wb_valid = 1'b0;
            rob_wb_idx = '0;
            rob_wb_result = '0;
            rob_wb_is_branch = 1'b0;
            rob_wb_branch_taken = 1'b0;
            rob_wb_branch_target = '0;
        end
    end
    
    // Writeback to physical register file
    always_comb begin
        if (arb_lsu_grant) begin
            prf_we = (lsu_wb_phys_rd != 0); // Don't write to x0
            prf_waddr = lsu_wb_phys_rd;
            prf_wdata = lsu_wb_data;
        end else if (arb_exec_grant) begin
            prf_we = (exec_phys_rd != 0); // Don't write to x0
            prf_waddr = exec_phys_rd;
            prf_wdata = exec_result;
        end else begin
            prf_we = 1'b0;
            prf_waddr = '0;
            prf_wdata = '0;
        end
    end
    
    // Bypass network for forwarding
    always_comb begin
        if (arb_lsu_grant) begin
            bypass_valid = 1'b1;
            bypass_phys_rd = lsu_wb_phys_rd;
            bypass_result = lsu_wb_data;
        end else if (arb_exec_grant) begin
            bypass_valid = 1'b1;
            bypass_phys_rd = exec_phys_rd;
            bypass_result = exec_result;
        end else begin
            bypass_valid = 1'b0;
            bypass_phys_rd = '0;
            bypass_result = '0;
        end
    end

endmodule
