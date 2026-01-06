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
    input  logic [ROB_W-1:0]  enq_phys_rd,
    input  logic [ADDR_W-1:0] enq_addr,

    input  logic              mem_resp,    // memory response
    input  logic [DATA_W-1:0] mem_data,

    output logic              lq_done,     // load complete
    output logic [ROB_W-1:0]  lq_rob,
    output logic [ROB_W-1:0]  lq_phys_rd,
    output logic [DATA_W-1:0] lq_data
);

    typedef struct packed {
        logic valid;
        logic [ROB_W-1:0] rob;
        logic [ROB_W-1:0] phys_rd;
        logic [ADDR_W-1:0] addr;
    } lq_entry_t;

    lq_entry_t lq [LQ_SIZE];

    logic [$clog2(LQ_SIZE)-1:0] head;
    logic [$clog2(LQ_SIZE)-1:0] tail;
    logic [$clog2(LQ_SIZE)-1:0] next_head;
    logic [$clog2(LQ_SIZE)-1:0] next_tail;
    logic lq_full;
    logic lq_empty;

    // FIFO status flags
    assign lq_empty = !lq[head].valid;
    assign lq_full = lq[tail].valid;
    assign next_head = (head == (LQ_SIZE-1)) ? '0 : head + 1'b1;
    assign next_tail = (tail == (LQ_SIZE-1)) ? '0 : tail + 1'b1;

    // Sequential logic for LQ FIFO management
    always_ff @(posedge clk) begin
        if (rst) begin
            head <= '0;
            tail <= '0;
            for (int i = 0; i < LQ_SIZE; i++) begin
                lq[i].valid <= 1'b0;
                lq[i].rob   <= '0;
                lq[i].phys_rd <= '0;
                lq[i].addr  <= '0;
            end
        end else begin
            // Enqueue new load request
            if (enq_valid && !lq_full) begin
                lq[tail].valid <= 1'b1;
                lq[tail].rob   <= enq_rob;
                lq[tail].phys_rd <= enq_phys_rd;
                lq[tail].addr  <= enq_addr;
                tail <= next_tail;
            end

            // Dequeue completed load
            if (mem_resp && !lq_empty) begin
                lq[head].valid <= 1'b0;
                head <= next_head;
            end
        end
    end

    // Output assignments
    assign lq_done = mem_resp && !lq_empty;
    assign lq_rob  = lq[head].rob;
    assign lq_phys_rd = lq[head].phys_rd;
    assign lq_data = mem_data;

endmodule
