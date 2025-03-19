`timescale 1ps/1ps

module cpu_tb;

    logic clk;
    logic rst_n;

    logic [31:0] dmem_addr;
    logic [31:0] dmem_write_data;
    logic [31:0] dmem_read_data;
    logic        dmem_write;
    logic        dmem_read;
    logic [2:0]  dmem_funct3;
    logic        dmem_ready;

    cpu_top cpu (
        .clk(clk),
        .rst_n(rst_n),
        .dmem_addr(dmem_addr),
        .dmem_write_data(dmem_write_data),
        .dmem_read_data(dmem_read_data),
        .dmem_write(dmem_write),
        .dmem_read(dmem_read),
        .dmem_funct3(dmem_funct3)
    );

    data_memory dmem (
        .clk(clk),
        .rst_n(rst_n),
        .addr(dmem_addr),
        .write_data(dmem_write_data),
        .mem_write(dmem_write),
        .mem_read(dmem_read),
        .funct3(dmem_funct3),
        .read_data(dmem_read_data),
        .mem_ready(dmem_ready)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end
    
    // Reset generation
    initial begin
        rst_n = 0;
        #20 rst_n = 1; // Release reset after 20ns
    end
    
    // Signal monitoring for terminal output
    // initial begin
    //     $display("========== RISC-V CPU Simulation Started ==========");
    //     $display("Time\tPC\t\tInstruction\tALU Result");
    //     $monitor("%0t\t0x%h\t0x%h\t0x%h", 
    //              $time, 
    //              cpu_top.pc_current, 
    //              cpu_top.instruction_id, 
    //              cpu_top.alu_result_ex);
    // end
    
    // Generate VCD file for waveform viewing
    initial begin
        $dumpfile("build/riscv_cpu_waveform.vcd");
        $dumpvars(0, cpu_tb);
    end
    
    // Register file monitoring
    // initial begin
    //     integer i;
    //     #100; // Wait for some instructions to execute
        
    //     $display("\n========== Register File Contents ==========");
    //     for (i = 0; i < 32; i++) begin
    //         $display("x%0d: 0x%h", i, cpu_top.reg_file.registers[i]);
    //     end
    // end
    
    // Memory monitoring
    // initial begin
    //     int i;
    //     #150; // Wait for some memory operations
        
    //     $display("\n========== Data Memory Contents (First 10 Words) ==========");
    //     for (i = 0; i < 10; i++) begin
    //         $display("Addr 0x%h: 0x%h %h %h %h", 
    //                  i*4, 
    //                  cpu_top.dmem.mem[i*4+3], 
    //                  cpu_top.dmem.mem[i*4+2], 
    //                  cpu_top.dmem.mem[i*4+1], 
    //                  cpu_top.dmem.mem[i*4]);
    //     end
    // end
    
    // End simulation
    initial begin
        #1000 // Run for 1000ns
        $display("\n========== RISC-V CPU Simulation Finished ==========");
        $finish;
    end

endmodule

