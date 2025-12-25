import general_defines::*;

module btb(
    input  logic clk,
    input  logic rst,
    input  logic [INSTR_MEM_IDX_W-1:0] fetch_pc, // pc from fetch stage

    output logic btb_hit,
    output logic [INSTR_MEM_IDX_W-1:0] btb_target,

    // update from branch resolution
    input  logic update_valid,
    input  logic [INSTR_MEM_IDX_W-1:0] update_pc,
    input  logic [INSTR_MEM_IDX_W-1:0] update_target
);

    reg valid [0:BTB_LENGTH-1]; // valid bit for each btb entry

    // store pc (tag) and target
    reg [INSTR_MEM_IDX_W-1:0] tag    [0:BTB_LENGTH-1];
    reg [INSTR_MEM_IDX_W-1:0] target [0:BTB_LENGTH-1];

    // index into btb using pc
    wire [BTB_IDX_W-1:0] fetch_idx;
    wire [BTB_IDX_W-1:0] update_idx;

    assign fetch_idx  = fetch_pc[BTB_IDX_W-1:0];
    assign update_idx = update_pc[BTB_IDX_W-1:0];

    // lookup logic
    always @(*) begin
        if(valid[fetch_idx] && tag[fetch_idx] == fetch_pc) begin
            btb_hit    = 1'b1;
            btb_target = target[fetch_idx];
        end
        else begin
            btb_hit    = 1'b0;
            btb_target = '0;
        end
    end

    // update btb on branch resolution
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            for(int i = 0; i < BTB_LENGTH; i++) begin
                valid[i]  <= 1'b0;
                tag[i]    <= '0;
                target[i] <= '0;
            end
        end
        else if(update_valid) begin
            valid[update_idx]  <= 1'b1;
            tag[update_idx]    <= update_pc;
            target[update_idx] <= update_target;
        end
    end

endmodule
