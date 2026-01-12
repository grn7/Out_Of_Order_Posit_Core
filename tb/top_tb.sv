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
        
        // Fill rest with NOPs to prevent invalid instruction fetch
        for (int i = 8; i < 256; i++) inst_mem[i] = 32'h00000013; // addi x0, x0, 0 (NOP)
    end
    
    // Data memory - no longer needed as mem.sv has its own storage
    // The actual memory is inside dut.mem_u.mem_array
    
    // Simulation control
    initial begin
        rst = 1;
        
        // Initialize architectural registers DURING reset
        // Use force to set initial values for testing
        // Do this during reset so values are stable before first instruction
        #5; // Small delay to ensure reset is active
        force dut.phys_reg_file[0] = 32'h00000000;  // x0 = 0 (hardwired zero)
        force dut.phys_reg_file[1] = 32'h00000000;  // x1 = 0 (base address for lw x5, 0(x1))
        force dut.phys_reg_file[2] = 32'h00000010;  // x2 = 16
        force dut.phys_reg_file[3] = 32'h00000005;  // x3 = 5
        force dut.phys_reg_file[4] = 32'h00000003;  // x4 = 3
        force dut.phys_reg_file[9] = 32'h00000020;  // x9 = 32
        force dut.phys_reg_file[11] = 32'h00000100; // x11 = 256
        force dut.phys_reg_file[12] = 32'h00000200; // x12 = 512
        force dut.phys_reg_file[13] = 32'h00000000; // x13 = 0 (base address for sw x10, 4(x13))
        force dut.phys_reg_file[15] = 32'h00000050; // x15 = 80
        force dut.phys_reg_file[17] = 32'h00000005; // x17 = 5
        
        // Also mark these registers as valid
        force dut.phys_reg_valid[0] = 1'b1;
        force dut.phys_reg_valid[1] = 1'b1;
        force dut.phys_reg_valid[2] = 1'b1;
        force dut.phys_reg_valid[3] = 1'b1;
        force dut.phys_reg_valid[4] = 1'b1;
        force dut.phys_reg_valid[9] = 1'b1;
        force dut.phys_reg_valid[11] = 1'b1;
        force dut.phys_reg_valid[12] = 1'b1;
        force dut.phys_reg_valid[13] = 1'b1;
        force dut.phys_reg_valid[15] = 1'b1;
        force dut.phys_reg_valid[17] = 1'b1;
        
        #15 rst = 0;  // Release reset after registers are initialized

        #1000;
        
        // Display final register values
        $display("\n========== SIMULATION RESULTS ==========");
        $display("Time: %0t ns", $time);
        $display("\n--- Architectural Register Values ---");
        $display("x1:  %h (Expected: 00000000)", dut.phys_reg_file[dut.rename_table[1]]);
        $display("x2:  %h (Expected: 00000010)", dut.phys_reg_file[dut.rename_table[2]]);
        $display("x3:  %h (Expected: 00000005)", dut.phys_reg_file[dut.rename_table[3]]);
        $display("x4:  %h (Expected: 00000003)", dut.phys_reg_file[dut.rename_table[4]]);
        $display("x5:  %h (Expected: 12345678 - loaded from mem[0])", dut.phys_reg_file[dut.rename_table[5]]);
        $display("x6:  %h (Expected: 12345688 - x5+x2)", dut.phys_reg_file[dut.rename_table[6]]);
        $display("x7:  %h (Expected: 0000000F - x3*x4)", dut.phys_reg_file[dut.rename_table[7]]);
        $display("x8:  %h (Expected: 0000002F - x7+x9)", dut.phys_reg_file[dut.rename_table[8]]);
        $display("x10: %h (Expected: 00000300 - x11+x12)", dut.phys_reg_file[dut.rename_table[10]]);
        $display("x14: %h (Expected: 0000005A - x15+10)", dut.phys_reg_file[dut.rename_table[14]]);
        $display("x16: %h (Expected: 0000005F - x14+x17)", dut.phys_reg_file[dut.rename_table[16]]);
        
        $display("\n--- Memory Values ---");
        $display("mem[0]: %h (Initial value)", dut.mem_u.mem_array[0]);
        $display("mem[1]: %h (Should be 00000300 - stored by sw x10, 4(x13))", dut.mem_u.mem_array[1]);
        
        $display("\n--- Pipeline Status ---");
        $display("ROB head: %0d, tail: %0d", dut.rob_head, dut.rob_tail);
        $display("IF PC: %h", dut.if_pc);
        $display("IF valid: %b", dut.if_valid);
        $display("IF stall: %b", dut.if_stall);
        $display("Imem req valid: %b", dut.imem_req_valid);
        $display("Imem resp valid: %b", dut.imem_resp_valid);
        
        $display("\n--- Register Rename Table ---");
        $display("x1->p%0d, x2->p%0d, x3->p%0d, x4->p%0d, x5->p%0d", 
            dut.rename_table[1], dut.rename_table[2], dut.rename_table[3], 
            dut.rename_table[4], dut.rename_table[5]);
        $display("x6->p%0d, x7->p%0d, x8->p%0d, x9->p%0d, x10->p%0d", 
            dut.rename_table[6], dut.rename_table[7], dut.rename_table[8], 
            dut.rename_table[9], dut.rename_table[10]);
        
        $display("\n--- ROB Status (first 8 entries) ---");
        for (int i = 0; i < 8; i++) begin
            if (dut.rob[i].valid) begin
                $display("ROB[%0d]: valid=%b done=%b pc=%h rd=x%0d->p%0d result=%h", 
                    i, dut.rob[i].valid, dut.rob[i].done, dut.rob[i].pc,
                    dut.rob[i].logical_rd, dut.rob[i].phys_rd, dut.rob[i].result);
            end
        end
        $display("========================================\n");
        
        $finish;
    end
    
    // Automatically add waveforms at start of simulation
    initial begin
        #1; // Wait 1 time unit for simulation to start
        $display("Adding memory array to waveform...");
        // Add memory array to waveform window
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
    // Data memory interface - mem module now handles this internally
    // No need to force signals since mem.sv is properly instantiated
    // =====================================================================

    // Simple simulation - no debug output
    initial begin
        #1000;
        $finish;
    end

endmodule
