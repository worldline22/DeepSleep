`timescale 1ns / 1ps
module IM(    
    input clk,
    input rst,
    input we,  //写使能（暂时不用）
    input wd,   //写入指令（暂时不用）
    input [31:0] pc,
    output [31:0] inst
);  //instruction memory
integer i; 
wire [31:0] addr;
assign addr = (pc - 32'h00400000) / 4;
reg [31:0] im [128:0];  //允许塞128指令
initial begin
    im[0] = 32'b00000000010000000000000010010011;   //ADDI x1, x0; 将零寄存器(x0)的值 +4 存入x1    
end
always@(posedge clk) begin
    if (rst) begin
        im[0] <= 32'b00000000010000000000000010010011;   //ADDI x1, x0; 将零寄存器(x0)的值 +4 存入x1
        
        for (i = 1; i < 128; i = i + 1) begin
            im[i] <= 0;
        end 
    end
end
assign inst = im[addr];  //指令存储器输出

endmodule
