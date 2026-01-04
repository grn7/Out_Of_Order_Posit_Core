import general_defines::*;

module IF (
    input  logic clk,
    input  logic rst,

    // Branch prediction interface
    input  logic                        pred_valid,      // prediction valid from branch predictor
    input  logic [INSTR_MEM_IDX_W-1:0]  pred_target,     // predicted target pc

    // Pipeline control
    input  logic                        stall,           // stall fetch
    input  logic                        flush,           // flush fetch on misprediction
    input  logic [INSTR_MEM_IDX_W-1:0]  flush_pc,        // redirect pc on flush

    // Instruction memory interface
    output logic                        imem_req_valid,  // instruction fetch request
    output logic [INSTR_MEM_IDX_W-1:0]  imem_req_addr,   // instruction memory address
    input  logic                        imem_resp_valid, // instruction memory response valid
    input  logic [INT_DATA_W-1:0]       imem_resp_data,  // instruction from memory

    // Outputs to next stage (ID)
    output logic                        if_valid,        // valid instruction fetched
    output logic [INSTR_MEM_IDX_W-1:0]  if_pc,           // pc sent to next stage
    output logic [INT_DATA_W-1:0]       if_instr,        // instruction sent to next stage
    
    // Current PC for branch predictor (before fetch completes)
    output logic [INSTR_MEM_IDX_W-1:0]  current_pc       // current PC being fetched
);

    // Internal PC register
    logic [INSTR_MEM_IDX_W-1:0] pc;
    logic [INSTR_MEM_IDX_W-1:0] next_pc;
    
    // Fetch buffer to handle stalls
    logic [INT_DATA_W-1:0]       fetched_instr;
    logic [INSTR_MEM_IDX_W-1:0]  fetched_pc;
    logic                        fetched_valid;

    // PC update logic - synthesizable sequential logic
    always_ff @(posedge clk) begin
        if (rst) begin
            pc <= '0;
        end else if (flush) begin
            pc <= flush_pc;
        end else if (!stall) begin
            pc <= next_pc;
        end
        // if stall, pc holds current value
    end

    // Next PC calculation - synthesizable combinational logic
    always_comb begin
        if (pred_valid) begin
            next_pc = pred_target;              // use predicted target
        end else begin
            next_pc = pc + INSTR_MEM_IDX_W'(1);  // sequential fetch (pc+1 for instruction memory addressing)
        end
    end

    // Instruction memory request logic
    always_comb begin
        imem_req_valid = !stall;  // request when not stalled
        imem_req_addr = pc;       // current pc as address
    end

    // Fetch buffer register - holds fetched instruction during stalls
    always_ff @(posedge clk) begin
        if (rst) begin
            fetched_valid <= 1'b0;
            fetched_instr <= '0;
            fetched_pc <= '0;
        end else if (flush) begin
            fetched_valid <= 1'b0;
            fetched_instr <= '0;
            fetched_pc <= '0;
        end else if (imem_resp_valid && !stall) begin
            fetched_valid <= 1'b1;
            fetched_instr <= imem_resp_data;
            fetched_pc <= pc;
        end else if (!stall) begin
            fetched_valid <= 1'b0;
        end
        // if stall, hold current values
    end

    // Outputs to ID stage
    assign if_valid = fetched_valid;
    assign if_pc = fetched_pc;
    assign if_instr = fetched_instr;
    
    // Output current PC for branch predictor
    assign current_pc = pc;

endmodule
