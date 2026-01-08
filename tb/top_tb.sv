module top_tb();

    import general_defines::*;

    logic clk;
    logic rst;
    
    // Instantiate the core
    top dut (
        .clk(clk),
        .rst(rst)
    );
    
    // Clock generation
    initial begin
        clk = 1;
        forever #5 clk = ~clk;
    end
    
    // Instruction memory content
    logic [31:0] inst_mem [0:255];
    
    // Initialize instruction memory with our test sequence
    initial begin
        inst_mem[0] = 32'b0000000_00000_00001_000_00101_0000011; // lw x5, 0(x1)
        inst_mem[1] = 32'b0000000_00010_00101_000_00110_0110011; // add x6, x5, x2
        inst_mem[2] = 32'b0000001_00100_00011_000_00111_0110011; // mul x7, x3, x4
        inst_mem[3] = 32'b0000000_01001_00111_000_01000_0110011; // add x8, x7, x9
        inst_mem[4] = 32'b0000000_01100_01011_000_01010_0110011; // add x10, x11, x12
        inst_mem[5] = 32'b0000000_01010_01101_010_00100_0100011; // sw x10, 4(x13)
        inst_mem[6] = 32'b000000001010_01111_000_01110_0010011; // addi x14, x15, 10
        inst_mem[7] = 32'b0000000_10001_01110_000_10000_0110011; // add x16, x14, x17
    end
    
    // Data memory
    logic [31:0] data_mem [0:255];
    
    // Initialize data memory
    initial begin
        data_mem[0] = 32'h12345678; // Value for lw x5, 0(x1)
        for (int i = 1; i < 256; i++) data_mem[i] = 0;
    end
    
    // Simulation control
    initial begin
        rst = 1;
        #20 rst = 0;

        #1000;
        
        // Display final register values
        $display("Final register values:");
        $display("x5: %h", dut.phys_reg_file[dut.rename_table[5]]);
        $display("x6: %h", dut.phys_reg_file[dut.rename_table[6]]);
        $display("x7: %h", dut.phys_reg_file[dut.rename_table[7]]);
        $display("x8: %h", dut.phys_reg_file[dut.rename_table[8]]);
        $display("x10: %h", dut.phys_reg_file[dut.rename_table[10]]);
        $display("x14: %h", dut.phys_reg_file[dut.rename_table[14]]);
        $display("x16: %h", dut.phys_reg_file[dut.rename_table[16]]);
        
        $display("Stored value at x13+4: %h", data_mem[1]);
        
        $finish;
    end
    
    // =====================================================================
    // Instruction memory interface with 1-cycle startup delay
    // =====================================================================
    logic startup_done;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            startup_done <= 0;
        end else begin
            if (!startup_done) begin
                startup_done <= 1;
            end else begin
                if (dut.imem_req_valid) begin
                    force dut.imem_resp_data = inst_mem[dut.imem_req_addr >> 2];
                    force dut.imem_resp_valid = 1;  // Always ready after startup
                end else begin
                    force dut.imem_resp_valid = 0;
                end
            end
        end
    end
    
    // =====================================================================
    // Data memory interface
    // =====================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            force dut.mem_rd_resp = 0;
        end else begin
            if (dut.mem_rd_valid) begin
                force dut.mem_rd_data = data_mem[dut.mem_rd_addr >> 2];
                force dut.mem_rd_resp = 1;
            end else if (dut.mem_wr_valid) begin
                data_mem[dut.mem_wr_addr >> 2] <= dut.mem_wr_data;
            end else begin
                force dut.mem_rd_resp = 0;
            end
        end
    end

    // Monitoring
    always_ff @(posedge clk) begin
        if (dut.rob[dut.rob_head].done && !rst) begin
            $display("Cycle %0d: Committing PC %h", $time/10, dut.rob[dut.rob_head].pc);
        end
    end
endmodule
