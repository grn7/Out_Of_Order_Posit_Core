module sq #(
    parameter SQ_SIZE = 8,
    parameter ADDR_W  = 32,
    parameter DATA_W  = 32,
    parameter ROB_W   = 6
)(
    input  logic clk,
    input  logic rst,

    input  logic              enq_valid,   // store issue
    input  logic [ROB_W-1:0]  enq_rob,
    input  logic [ADDR_W-1:0] enq_addr,
    input  logic [DATA_W-1:0] enq_data,

    input  logic              commit_valid, // store commit
    input  logic [ROB_W-1:0]  commit_rob,

    output logic              mem_wr_valid,
    output logic [ADDR_W-1:0] mem_wr_addr,
    output logic [DATA_W-1:0] mem_wr_data
);

    typedef struct packed {
        logic valid;
        logic [ROB_W-1:0] rob;
        logic [ADDR_W-1:0] addr;
        logic [DATA_W-1:0] data;
    } sq_entry_t;

    sq_entry_t sq [SQ_SIZE];
    
    // Store to commit match index
    logic [SQ_SIZE-1:0] commit_match;
    logic [$clog2(SQ_SIZE)-1:0] commit_idx;
    logic commit_found;

    // Free slot for enqueue
    logic [SQ_SIZE-1:0] free_slot;
    logic [$clog2(SQ_SIZE)-1:0] enq_idx;
    logic enq_found;

    // Combinational priority encoder for free slot
    always_comb begin
        free_slot = '0;
        enq_found = 1'b0;
        enq_idx = '0;
        for (int i = 0; i < SQ_SIZE; i++) begin
            if (!sq[i].valid && !enq_found) begin
                free_slot[i] = 1'b1;
                enq_idx = i[$clog2(SQ_SIZE)-1:0];
                enq_found = 1'b1;
            end
        end
    end

    // Combinational priority encoder for commit match
    always_comb begin
        commit_match = '0;
        commit_found = 1'b0;
        commit_idx = '0;
        for (int i = 0; i < SQ_SIZE; i++) begin
            if (sq[i].valid && sq[i].rob == commit_rob && !commit_found) begin
                commit_match[i] = 1'b1;
                commit_idx = i[$clog2(SQ_SIZE)-1:0];
                commit_found = 1'b1;
            end
        end
    end

    // Sequential logic for SQ management
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < SQ_SIZE; i++) begin
                sq[i].valid <= 1'b0;
                sq[i].rob   <= '0;
                sq[i].addr  <= '0;
                sq[i].data  <= '0;
            end
        end else begin
            // Enqueue new store
            if (enq_valid && enq_found) begin
                sq[enq_idx].valid <= 1'b1;
                sq[enq_idx].rob   <= enq_rob;
                sq[enq_idx].addr  <= enq_addr;
                sq[enq_idx].data  <= enq_data;
            end

            // Commit/retire store - invalidate the entry
            if (commit_valid && commit_found) begin
                sq[commit_idx].valid <= 1'b0;
            end
        end
    end

    // Memory write outputs - drive memory bus when committing
    assign mem_wr_valid = commit_valid && commit_found;
    assign mem_wr_addr  = commit_found ? sq[commit_idx].addr : '0;
    assign mem_wr_data  = commit_found ? sq[commit_idx].data : '0;

endmodule
