module boom_core_tb();

    logic clk;
    logic reset;
    
    // Instruction memory interface
    logic [31:0] imem_addr;
    logic [31:0] imem_data;
    logic        imem_req;
    logic        imem_ready;
    
    // Data memory interface
    logic [31:0] dmem_addr;
    logic [31:0] dmem_wdata;
    logic [31:0] dmem_rdata;
    logic        dmem_req;
    logic        dmem_ready;
    logic        dmem_we;
    
    // Instantiate the core
    boom_core core (
        .clk(clk),
        .reset(reset),
        .imem_addr(imem_addr),
        .imem_data(imem_data),
        .imem_req(imem_req),
        .imem_ready(imem_ready),

        .dmem_addr(dmem_addr),
        .dmem_wdata(dmem_wdata),
        .dmem_rdata(dmem_rdata),
        .dmem_we(dmem_we),
        .dmem_req(dmem_req),
        .dmem_ready(dmem_ready)
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
        inst_mem[4] = 32'b0001100_00011_00010_000_00001_1010011; // fdiv.s f1, f2, f3
        inst_mem[5] = 32'b0000000_00101_00001_000_00100_1010011; // fadd.s f4, f1, f5
        inst_mem[6] = 32'b0000000_01100_01011_000_01010_0110011; // add x10, x11, x12
        inst_mem[7] = 32'b0000000_01010_01101_010_00100_0100011; // sw x10, 4(x13)
        inst_mem[8] = 32'b000000001010_01111_000_01110_0010011; // addi x14, x15, 10
        inst_mem[9] = 32'b0000000_10001_01110_000_10000_0110011; // add x16, x14, x17

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
        reset = 1;
        #20 reset = 0;

        #1000;
        
        // Display final register values
        $display("Final register values:");
        $display("x5: %h", core.phys_reg_file[core.rename_table[5]]);
        $display("x6: %h", core.phys_reg_file[core.rename_table[6]]);
        $display("x7: %h", core.phys_reg_file[core.rename_table[7]]);
        $display("x8: %h", core.phys_reg_file[core.rename_table[8]]);
        // if (33 < core.rename_table.size())
        //     $display("f1: %h", core.phys_reg_file[core.rename_table[33]]);
        // if (36 < core.rename_table.size())
        //     $display("f4: %h", core.phys_reg_file[core.rename_table[36]]);
        $display("x10: %h", core.phys_reg_file[core.rename_table[10]]);
        $display("x14: %h", core.phys_reg_file[core.rename_table[14]]);
        $display("x16: %h", core.phys_reg_file[core.rename_table[16]]);
        
        $display("Stored value at x13+4: %h", data_mem[1]);
        
        $finish;
    end
    
    // =====================================================================
    // Instruction memory interface with 1-cycle startup delay
    // =====================================================================
    logic startup_done;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            imem_ready   <= 0;
            imem_data    <= 0;
            startup_done <= 0;
        end else begin
            if (!startup_done) begin
                startup_done <= 1;
                imem_ready   <= 0;
            end else begin
                if (imem_req) begin
                    imem_data  <= inst_mem[imem_addr >> 2];
                    imem_ready <= 1;  // Always ready after startup
                end else begin
                    imem_ready <= 0;
                end
            end
        end
    end
    
    // =====================================================================
    // Data memory interface
    // =====================================================================
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            dmem_rdata <= 0;
            dmem_ready <= 0;
        end else begin
            if (dmem_req && !dmem_we) begin
                dmem_rdata <= data_mem[dmem_addr >> 2];
                dmem_ready <= 1;
            end else if (dmem_req && dmem_we) begin
                data_mem[dmem_addr >> 2] <= dmem_wdata;
                dmem_ready <= 1;
            end else begin
                dmem_ready <= 0;
            end
        end
    end

    // Monitoring
    always_ff @(posedge clk) begin
        if (core.rob[core.rob_head].done && !reset) begin
            $display("Cycle %0d: Committing PC %h", $time/10, core.rob[core.rob_head].pc);
        end
    end
endmodule
