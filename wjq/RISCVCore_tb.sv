module RISCVCore_tb;

    reg CLK;
    reg Reset;

    // 实例化被测试的RISC-V核
    RISCVCore uut (
        .CLK(CLK),
        .Reset(Reset)
    );

    // 时钟生成（周期=10ns）
    always #5 CLK = ~CLK;

    // 测试序列
    initial begin
        // 波形记录
        $dumpfile("riscv_core.vcd");
        $dumpvars(0, RISCVCore_tb);
        
        // 初始化信号
        CLK = 0;
        Reset = 1;
        
        // 复位操作
        #12 Reset = 0;  // 保持1.2个时钟周期的复位

        // 运行足够时钟周期（每个指令约2个时钟周期）
        #200; 
        
        // 自动化验证
        $display("\n=== 寄存器验证 ===");
        verify_reg(1, 32'd5);   // x1 = 5
        verify_reg(2, 32'd3);   // x2 = 3
        verify_reg(3, 32'd8);   // x3 = 5+3=8
        verify_reg(4, 32'd8);   // x4从内存加载值8
        verify_reg(5, 32'd1);   // 分支成功时x5=1
        
        $display("\n=== 存储器验证 ===");
        verify_mem(0, 32'd8);   // 地址0存储值8
        
        $display("\n=== 测试完成 ===");
        $finish;
    end

    // 自动化验证任务
    task verify_reg;
        input [4:0] regnum;
        input [31:0] expected;
        begin
            if (uut.reg_file.regFile[regnum] !== expected) begin
                $display("错误: x%0d = 0x%h (期望值0x%h)", 
                        regnum, uut.reg_file.regFile[regnum], expected);
            end else begin
                $display("x%0d 验证通过: 0x%h", regnum, expected);
            end
        end
    endtask

    task verify_mem;
        input [31:0] address;
        input [31:0] expected;
        begin
            if ({uut.data_mem.ram[address+3], 
                 uut.data_mem.ram[address+2],
                 uut.data_mem.ram[address+1],
                 uut.data_mem.ram[address]} !== expected) begin
                $display("错误: MEM[0x%h] = 0x%h (期望值0x%h)", 
                        address,
                        {uut.data_mem.ram[address+3],
                         uut.data_mem.ram[address+2],
                         uut.data_mem.ram[address+1],
                         uut.data_mem.ram[address]},
                        expected);
            end else begin
                $display("MEM[0x%h] 验证通过: 0x%h", address, expected);
            end
        end
    endtask

endmodule