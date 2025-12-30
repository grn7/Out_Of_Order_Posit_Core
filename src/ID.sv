// purely combinational module
// more like a decoder 

import general_defines::*;

module ID(
    input logic [31:0] instr, //to be given as input from fetch stage 
    output logic [6:0] opcode,
    output logic [4:0] rd,
    output logic [2:0] funct3,
    output logic [4:0] rs1,
    output logic [4:0] rs2,
    output logic [6:0] funct7,
    output logic [INT_DATA_W-1:0] immediate
);

assign opcode = instr[6:0];
assign rd = instr[11:7];
assign funct3 = instr[14:12];
assign rs1 = instr[19:15];
assign rs2 = instr[24:20];
assign funct7 = instr[31:25];

always_comb begin 
    immediate = '0; 
    case (opcode) 
        `OPCODE_LOAD, `OPCODE_ARITH_I: begin
            immediate = {{21{instr[31]}},instr[30:20]};
        end
        `OPCODE_STORE: begin
            immediate = {{21{instr[31]}},{instr[30:25],instr[11:7]}};
        end
        default: begin
            immediate = 0;
        end
    endcase
end


endmodule


