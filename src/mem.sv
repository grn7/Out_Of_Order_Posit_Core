module mem #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter DEPTH  = 1024
)(
    input  logic clk,
    input  logic rst,

    input  logic              rd_valid,   // read request valid
    input  logic [ADDR_W-1:0] rd_addr,    // read address
    output logic              rd_resp,    // read response valid (1-cycle later)
    output logic [DATA_W-1:0] rd_data,    // read data

    input  logic              wr_valid,   // write enable
    input  logic [ADDR_W-1:0] wr_addr,    // write address
    input  logic [DATA_W-1:0] wr_data     // write data
);

    logic [DATA_W-1:0] mem_array [0:DEPTH-1];  // memory storage
    logic [ADDR_W-1:0] rd_addr_q;               // registered read address

    // Initialize memory for testing
    initial begin
        mem_array[0] = 32'h12345678; // Value for lw instruction
        for (int i = 1; i < DEPTH; i++) mem_array[i] = 0;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            rd_resp   <= 1'b0;                  // clear read response
            rd_addr_q <= '0;
            rd_data   <= '0;
        end else begin
            rd_resp   <= rd_valid;              // propagate read valid
            rd_addr_q <= rd_addr;               // latch address
            if (rd_valid) begin
                // Convert byte address to word index (divide by 4)
                rd_data <= mem_array[rd_addr[$clog2(DEPTH)-1:2]]; // read data
            end
        end
    end

    always_ff @(posedge clk) begin
        if (wr_valid) begin
            // Convert byte address to word index (divide by 4)
            mem_array[wr_addr[$clog2(DEPTH)-1:2]] <= wr_data; // write memory
        end
    end

endmodule
