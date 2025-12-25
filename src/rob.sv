import general_defines::*;  

module rob_controller(
    input logic clk,
    input logic rst,
    input logic valid,
    input logic [INSTR_MEM_IDX_W-1:0] pc,
    input logic [ARCH_REG_IDX_W-1:0] logical_rd,
    input logic [PHYS_REG_IDX_W-1:0] phys_rd,
    input logic [INT_DATA_W-1:0] result,
    input logic done,
    input logic is_store,
    input logic is_load,
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [ROB_IDX_W-1:0] rob_head,
    input logic [ROB_IDX_W-1:0] rob_tail,
    input logic rob_write,
    
    output logic valid_out,
    output logic [INSTR_MEM_IDX_W-1:0] pc_out,
    output logic [ARCH_REG_IDX_W-1:0] logical_rd_out,
    output logic [PHYS_REG_IDX_W-1:0] phys_rd_out,
    output logic [INT_DATA_W-1:0] result_out,
    output logic done_out,
    output logic is_store_out,
    output logic is_load_out,
    output logic [6:0] opcode_out,
    output logic [2:0] funct3_out,
    output logic [6:0] funct7_out
);

rob_entry_t rob [0:ROB_LENGTH-1];

always_ff @(posedge clk) begin
    if(rst) begin
        for(int i = 0; i < ROB_LENGTH; i++) begin
            rob[i].valid <= 'b0;
            rob[i].pc <= 'b0;
            rob[i].logical_rd <= 'b0;
            rob[i].phys_rd <= 'b0;
            rob[i].result <= 'b0;
            rob[i].done <= 'b0;
            rob[i].is_store <= 'b0;
            rob[i].is_load <= 'b0;
            rob[i].opcode <= 'b0;
            rob[i].funct3 <= 'b0;
            rob[i].funct7 <= 'b0;
        end
    end
    else if (rob_write) begin
        rob[rob_tail].valid <= valid;
        rob[rob_tail].pc <= pc;
        rob[rob_tail].logical_rd <= logical_rd;
        rob[rob_tail].phys_rd <= phys_rd;
        rob[rob_tail].result <= result;
        rob[rob_tail].done <= done;
        rob[rob_tail].is_store <= is_store;
        rob[rob_tail].is_load <= is_load;
        rob[rob_tail].opcode <= opcode;
        rob[rob_tail].funct3 <= funct3;
        rob[rob_tail].funct7 <= funct7;
    end
end

//asynchronous data read 
assign valid_out = rob[rob_head].valid;
assign pc_out = rob[rob_head].pc;
assign logical_rd_out = rob[rob_head].logical_rd;
assign phys_rd_out = rob[rob_head].phys_rd;
assign result_out = rob[rob_head].result;
assign done_out = rob[rob_head].done;
assign is_store_out = rob[rob_head].is_store;
assign is_load_out = rob[rob_head].is_load;
assign opcode_out = rob[rob_head].opcode;
assign funct3_out = rob[rob_head].funct3;
assign funct7_out = rob[rob_head].funct7;

endmodule
