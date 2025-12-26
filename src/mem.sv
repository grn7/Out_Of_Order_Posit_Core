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

    always_ff @(posedge clk) begin
        if (rst) begin
            rd_resp <= 1'b0;                    // clear read response
        end else begin
            rd_resp   <= rd_valid;              // propagate read valid
            rd_addr_q <= rd_addr;               // latch address
            rd_data   <= mem_array[rd_addr_q[$clog2(DEPTH)-1:0]]; // read data
        end
    end

    always_ff @(posedge clk) begin
        if (wr_valid) begin
            mem_array[wr_addr[$clog2(DEPTH)-1:0]] <= wr_data; // write memory
        end
    end

endmodule
