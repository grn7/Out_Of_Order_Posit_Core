import general_defines::*;

module bp(
    input  logic rst, 
    input  logic clk,

    input  logic [INSTR_MEM_IDX_W-1:0] fetch_pc,   // pc coming from fetch
    input  logic actual_taken,                     // 1 or 0 for taken/not taken
    input  logic update_valid,                     // update enable

    output logic pred_taken                         // final prediction
);

    reg [1:0] state [0:PHT_LENGTH-1];   // states 00,01,10,11

    wire [PHT_IDX_W-1:0] pht_idx;
    assign pht_idx = fetch_pc[PHT_IDX_W-1:0];

    // main fsm 
    always @(*) begin
        if(state[pht_idx][1] == 1'b0)   // if state is 00 or 01 we predict taken
            pred_taken = 1'b1;
        else                            // if state is 10 or 11 we predict not taken
            pred_taken = 1'b0;
    end

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            // Default WNT 
            for(int i = 0; i < PHT_LENGTH; i++) begin
                state[i] <= 2'b10;
            end
        end
        else if(update_valid) begin
            case(state[pht_idx])
                2'b00: begin  // ST 
                    if(actual_taken == 1'b1)
                        state[pht_idx] <= 2'b00;  
                    else
                        state[pht_idx] <= 2'b01;  
                end
                
                2'b01: begin  // WT 
                    if(actual_taken == 1'b1)
                        state[pht_idx] <= 2'b00;  
                    else
                        state[pht_idx] <= 2'b10;  
                end
                
                2'b10: begin  // WNT 
                    if(actual_taken == 1'b1)
                        state[pht_idx] <= 2'b01;  
                    else
                        state[pht_idx] <= 2'b11;  
                end
                
                2'b11: begin  // SNT 
                    if(actual_taken == 1'b1)
                        state[pht_idx] <= 2'b10;  
                    else
                        state[pht_idx] <= 2'b11;  
                end
            endcase
        end
    end

endmodule
// module bp(
//     input reset, 
//     input clock,
//     input flag,           // 1/0 for taken/not taken
//     output reg [1:0] state,  // states 00,01,10,11
//     output reg flag_out   // predicted taken/not taken same 1/0 as flag
// );

// always@(*) begin
//     if(state[1] == 1'b0)  // if state is 00 or 01 we predict taken
//         flag_out = 1'b1;
//     else                  // if state is 10 or 11 we predict not taken
//         flag_out = 1'b0;
// end

// // main fsm
// always@(posedge clock or posedge reset) 
//     begin
//         if(reset)
//             state <= 2'b10;  // Default WNT 
//         else begin
//             case(state)
//                 2'b00: begin  // ST 
//                     if(flag == 1'b1)
//                         state <= 2'b00;  
//                     else
//                         state <= 2'b01;  
//                 end
                
//                 2'b01: begin  // WT 
//                     if(flag == 1'b1)
//                         state <= 2'b00;  
//                     else
//                         state <= 2'b10;  
//                 end
                
//                 2'b10: begin  // WNT 
//                     if(flag == 1'b1)
//                         state <= 2'b01;  
//                         state <= 2'b11;  
//                 end
                
//                 2'b11: begin  // SNT 
//                     if(flag == 1'b1)
//                         state <= 2'b10;  
//                     else
//                         state <= 2'b11;  
//                 end
//             endcase
//         end
// end

// endmodule
