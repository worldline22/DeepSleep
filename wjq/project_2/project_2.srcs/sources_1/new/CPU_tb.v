`timescale 1ns/1ps

module CPU_tb();

reg clk;
reg rst;
reg global_en;

// 指令存储器接口
wire [31:0] imem_raddr;
wire [31:0] imem_rdata;

// 数据存储器接口
wire [31:0] dmem_rdata = 0; // 测试不涉及数据加载
wire dmem_we;
wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;

// 调试信号
wire commit;
wire [31:0] commit_pc;
wire [31:0] commit_inst;
wire commit_halt;
wire commit_reg_we;
wire [4:0] commit_reg_wa;
wire [31:0] commit_reg_wd;

reg [4:0]debug_reg_ra;
wire [31:0]debug_reg_rd;

// 实例化被测CPU
CPU uut (
    .clk(clk),
    .rst(rst),
    .global_en(global_en),
    
    .imem_raddr(imem_raddr),
    .imem_rdata(imem_rdata),
    
    .dmem_rdata(dmem_rdata),
    .dmem_we(dmem_we),
    .dmem_addr(dmem_addr),
    .dmem_wdata(dmem_wdata),
    
    .commit(commit),
    .commit_pc(commit_pc),
    .commit_inst(commit_inst),
    .commit_halt(commit_halt),
    .commit_reg_we(commit_reg_we),
    .commit_reg_wa(commit_reg_wa),
    .commit_reg_wd(commit_reg_wd),
    
    .debug_reg_ra(debug_reg_ra), // 测试时先读x1，后改读x2
    .debug_reg_rd(debug_reg_rd)
);

// 时钟生成
always #5 clk = ~clk;

initial begin
    // 初始化信号
    clk = 0;
    rst = 1;
    global_en = 1;
    
    // 复位阶段
    #10 rst = 0;
    
    // 等待ADDI执行完成（约5个周期）
    repeat(8) @(posedge clk); // 保证进入WB阶段
    
    // 验证x1 = 5
    debug_reg_ra = 5'h1; // 读取x1
    #1;
    $display("x1 = %d (Expected: 4)", uut.debug_reg_rd);
    
    // 等待ADD执行完成（再等4个周期）
    repeat(4) @(posedge clk);
    
    // 验证x2 = 10
    debug_reg_ra = 5'h2; // 读取x2
    #1;
    $display("x2 = %d (Expected: 10)", uut.debug_reg_rd);
    
    $finish;
end

// 监视提交信息
always @(posedge clk) begin
    if (commit) begin
        $display("[COMMIT] PC: %h, INST: %h", commit_pc, commit_inst);
    end
end

endmodule