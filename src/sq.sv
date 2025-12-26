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

    integer i;

    always_ff @(posedge clk) begin
        if (rst) begin
            foreach (sq[i]) sq[i].valid <= 1'b0;
        end else begin
            if (enq_valid) begin
                for (i = 0; i < SQ_SIZE; i = i + 1) begin
                    if (!sq[i].valid) begin
                        sq[i].valid <= 1'b1;
                        sq[i].rob   <= enq_rob;
                        sq[i].addr  <= enq_addr;
                        sq[i].data  <= enq_data;
                        disable for;
                    end
                end
            end

            if (commit_valid) begin
                for (i = 0; i < SQ_SIZE; i = i + 1) begin
                    if (sq[i].valid && sq[i].rob == commit_rob) begin
                        sq[i].valid <= 1'b0;
                    end
                end
            end
        end
    end

    assign mem_wr_valid = commit_valid;
    assign mem_wr_addr  = sq[0].addr;
    assign mem_wr_data  = sq[0].data;

endmodule
