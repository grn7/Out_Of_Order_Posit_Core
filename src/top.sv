import general_defines::*;

module top (
    input logic clk,
    input logic rst
);

    logic [XLEN-1:0] if_pc;                 // IF stage PC
    logic [XLEN-1:0] if_next_pc;            // next PC

    logic            bp_valid;               // branch prediction valid
    logic [XLEN-1:0] bp_target;              // predicted target

    logic [XLEN-1:0] if_instr;               // fetched instruction (stub)

    logic            issue_valid;             // issue enable (stub)
    logic            issue_is_load;           // load op
    logic            issue_is_store;          // store op
    logic [ROB_IDX_W-1:0] issue_rob;          // rob index
    logic [XLEN-1:0] issue_addr;              // memory address
    logic [XLEN-1:0] issue_store_data;        // store data

    logic            wb_valid;                // writeback valid
    logic [ROB_IDX_W-1:0] wb_rob;             // writeback rob index
    logic [XLEN-1:0] wb_data;                 // writeback data

    logic            commit_store;             // store commit from rob
    logic [ROB_IDX_W-1:0] commit_rob;          // committing rob entry

    logic            mem_rd_valid;             // memory read request
    logic [XLEN-1:0] mem_rd_addr;              // memory read address
    logic            mem_rd_resp;              // memory read response
    logic [XLEN-1:0] mem_rd_data;              // memory read data

    logic            mem_wr_valid;             // memory write enable
    logic [XLEN-1:0] mem_wr_addr;              // memory write address
    logic [XLEN-1:0] mem_wr_data;              // memory write data

    always_ff @(posedge clk) begin
        if (rst)
            if_pc <= '0;                      // reset IF PC
        else
            if_pc <= if_next_pc;              // update IF PC
    end

    assign if_next_pc = bp_valid ? bp_target : if_pc + XLEN'(4); // IF pc logic

    assign if_instr = '0;                     // instruction memory stub

    bp bp_u (
        .clk          (clk),
        .rst          (rst),
        .fetch_pc     (if_pc),
        .actual_taken (1'b0),                 // EX feedback stub
        .update_valid (1'b0),                 // EX feedback stub
        .pred_taken   (bp_valid)
    );

    btb btb_u (
        .clk    (clk),
        .rst    (rst),
        .pc     (if_pc),
        .hit    (),                           // unused for now
        .target ()
    );

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
