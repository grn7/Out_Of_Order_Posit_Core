module lq #(
    parameter LQ_SIZE = 8,
    parameter ADDR_W  = 32,
    parameter DATA_W  = 32,
    parameter ROB_W   = 6
)(
    input  logic clk,
    input  logic rst,

    input  logic              enq_valid,   // load issue
    input  logic [ROB_W-1:0]  enq_rob,
    input  logic [ADDR_W-1:0] enq_addr,

    input  logic              mem_resp,    // memory response
    input  logic [DATA_W-1:0] mem_data,

    output logic              lq_done,     // load complete
    output logic [ROB_W-1:0]  lq_rob,
    output logic [DATA_W-1:0] lq_data
);

    typedef struct packed {
        logic valid;
        logic [ROB_W-1:0] rob;
        logic [ADDR_W-1:0] addr;
    } lq_entry_t;

    lq_entry_t lq [LQ_SIZE];

    logic [$clog2(LQ_SIZE)-1:0] head;
    logic [$clog2(LQ_SIZE)-1:0] tail;

    always_ff @(posedge clk) begin
        if (rst) begin
            head <= '0;
            tail <= '0;
            foreach (lq[i]) lq[i].valid <= 1'b0;
        end else begin
            if (enq_valid && !lq[tail].valid) begin
                lq[tail].valid <= 1'b1;
                lq[tail].rob   <= enq_rob;
                lq[tail].addr  <= enq_addr;
                tail <= (tail == LQ_SIZE-1) ? '0 : tail + 1'b1;
            end

            if (mem_resp && lq[head].valid) begin
                lq[head].valid <= 1'b0;
                head <= (head == LQ_SIZE-1) ? '0 : head + 1'b1;
            end
        end
    end

    assign lq_done = mem_resp;
    assign lq_rob  = lq[head].rob;
    assign lq_data = mem_data;

endmodule
