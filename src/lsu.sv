module lsu #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter ROB_W  = 6,
    parameter LQ_SIZE = 8,
    parameter SQ_SIZE = 8
)(
    input  logic clk,
    input  logic rst,

    // Issue interface from instruction queue
    input  logic              issue_valid,
    input  logic              issue_is_load,
    input  logic              issue_is_store,
    input  logic [ROB_W-1:0]  issue_rob,
    input  logic [ADDR_W-1:0] issue_addr,
    input  logic [DATA_W-1:0] issue_store_data,

    // Commit interface from ROB
    input  logic              commit_store,
    input  logic [ROB_W-1:0]  commit_rob,

    // Writeback interface to ROB
    output logic              wb_valid,
    output logic [ROB_W-1:0]  wb_rob,
    output logic [DATA_W-1:0] wb_data,

    // Memory interface
    output logic              mem_rd_valid,
    output logic [ADDR_W-1:0] mem_rd_addr,
    input  logic              mem_rd_resp,
    input  logic [DATA_W-1:0] mem_rd_data,

    output logic              mem_wr_valid,
    output logic [ADDR_W-1:0] mem_wr_addr,
    output logic [DATA_W-1:0] mem_wr_data
);

    // Load Queue instantiation
    lq #(
        .LQ_SIZE(LQ_SIZE),
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W),
        .ROB_W(ROB_W)
    ) lq_u (
        .clk(clk),
        .rst(rst),
        .enq_valid(issue_valid && issue_is_load),
        .enq_rob(issue_rob),
        .enq_addr(issue_addr),
        .mem_resp(mem_rd_resp),
        .mem_data(mem_rd_data),
        .lq_done(wb_valid),
        .lq_rob(wb_rob),
        .lq_data(wb_data)
    );

    // Store Queue instantiation
    sq #(
        .SQ_SIZE(SQ_SIZE),
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W),
        .ROB_W(ROB_W)
    ) sq_u (
        .clk(clk),
        .rst(rst),
        .enq_valid(issue_valid && issue_is_store),
        .enq_rob(issue_rob),
        .enq_addr(issue_addr),
        .enq_data(issue_store_data),
        .commit_valid(commit_store),
        .commit_rob(commit_rob),
        .mem_wr_valid(mem_wr_valid),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data)
    );

    // Memory read request - direct from issue
    assign mem_rd_valid = issue_valid && issue_is_load;
    assign mem_rd_addr  = issue_addr;

    // TODO: Add store-to-load forwarding logic
    // - Check if load address matches any pending store in SQ
    // - Forward data from SQ if address matches
    // - Implement memory disambiguation

endmodule
