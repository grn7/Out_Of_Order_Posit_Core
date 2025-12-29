import general_defines::*;

module IF (
    input  logic clk,
    input  logic rst,

    input  logic            pred_valid,     // prediction valid
    input  logic [XLEN-1:0] pred_pc,        // predicted pc

    input  logic            stall,           // stall fetch (stub for now)
    input  logic            flush,           // flush fetch (stub for now)
    input  logic [XLEN-1:0] flush_pc,        // redirect pc on flush

    output logic [XLEN-1:0] if_pc            // pc sent to next stage
);

    logic [XLEN-1:0] pc;                     // internal pc
    logic [XLEN-1:0] next_pc;                // next pc value

    always_ff @(posedge clk) begin
        if (rst)
            pc <= '0;                        // reset pc
        else if (flush)
            pc <= flush_pc;                  // redirect on flush
        else if (!stall)
            pc <= next_pc;                   // normal pc update
    end

    always_comb begin
        if (pred_valid)
            next_pc = pred_pc;               // predicted target
        else
            next_pc = pc + XLEN'(4);          // sequential fetch
    end

    assign if_pc = pc;                       // output pc

endmodule
