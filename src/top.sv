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


\    logic            issue_valid;             // issue enable (stub)
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

endmodule
