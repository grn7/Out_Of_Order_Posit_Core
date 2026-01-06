/* verilator lint_off MULTITOP */

import general_defines::*;

//single cycle add 
module adder(
    input clk,
    input rst,
    input logic [INT_DATA_W-1:0] a,
    input logic [INT_DATA_W-1:0] b,
    input logic valid_i,
    output logic [INT_DATA_W-1:0] result,
    output logic valid_o
);

always_ff @(posedge clk) begin
    if(rst) begin
        result <= 'b0;
        valid_o <= 1'b0;
    end
    else begin
        valid_o <= valid_i;
        if(valid_i) begin
            result <= a+b;
        end
    end
end
    
endmodule

//single cycle subtractor 
module subtractor(
    input clk,
    input rst,
    input logic [INT_DATA_W-1:0] a,
    input logic [INT_DATA_W-1:0] b,
    input logic valid_i,
    output logic [INT_DATA_W-1:0] result,
    output logic valid_o
);

always_ff @(posedge clk) begin
    if(rst) begin
        result <= 'b0;
        valid_o <= 1'b0;
    end
    else begin
        valid_o <= valid_i;
        if(valid_i) begin
            result <= a-b;
        end
    end
end
    
endmodule

//4 cycle multiplier
module multiplier(
    input clk,
    input rst,
    input logic [INT_DATA_W-1:0] a,
    input logic [INT_DATA_W-1:0] b,
    input logic valid_i,
    output logic [INT_DATA_W-1:0] result,
    output logic busy
);

logic [1:0] count_mul;
logic [INT_DATA_W-1:0] a_store,b_store ; //so that we can compute result for the values which were given as inputs ,
//instead of current values on those lines 

always_ff @(posedge clk) begin
    if(rst) begin
        result <= 'b0;
        count_mul <= 'b0;
        busy <= 'b0;
    end
    else if (valid_i && !busy ) begin
        a_store <= a; //latch the inputs 
        b_store <= b;
        busy <= 1'b1;
        count_mul <= 'b0;
    end
    else if(busy) begin
        if(count_mul == 2'b11) begin
            result <= a_store*b_store;
            busy <= 'b0;
            count_mul <= 'b0;
        end
        else begin
            count_mul <= count_mul+1;
        end
    end

end
    
endmodule

//8 cycle divider
module divider(
    input clk,
    input rst,
    input logic [INT_DATA_W-1:0] a,
    input logic [INT_DATA_W-1:0] b,
    input logic valid_i,
    output logic [INT_DATA_W-1:0] result,
    output logic busy
);

logic [2:0] count_div;
logic [INT_DATA_W-1:0] a_store,b_store ; //so that we can compute result for the values which were given as inputs ,
//instead of current values on those lines 

always_ff @(posedge clk) begin
    if(rst) begin
        result <= 'b0;
        count_div <= 'b0;
        busy <= 'b0;
    end
    else if (valid_i && !busy ) begin
        a_store <= a; //latch the inputs 
        b_store <= b;
        busy <= 1'b1;
        count_div <= 'b0;
    end
    else if(busy) begin
        if(count_div == 3'b111) begin
            if(b_store == '0) begin
                result <= 'b0; //safety
                busy <= 'b0;
                count_div <= 'b0;
            end
            else begin
                result <= a_store/b_store;
                busy <= 'b0;
                count_div <= 'b0;
            end
        end
        else begin
            count_div <= count_div+1;
        end
    end

end
    
endmodule

/* verilator lint_on MULTITOP */
