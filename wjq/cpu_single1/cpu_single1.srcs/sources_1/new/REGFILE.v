`timescale 1ns / 1ps

module REGFILE (
    input                   [ 0 : 0]         clk,
    input                   [ 0 : 0]         rst,
 
    input                   [ 4 : 0]         rf_ra0,  //读寄存器地址1
    input                   [ 4 : 0]         rf_ra1,   //读寄存器地址2  //如果卡功耗这里或许可以加上读使能信号
    input                   [ 4 : 0]         rf_wa,  //写寄存器地址
    input                   [ 0 : 0]         rf_we,  //写使能信号
    input                   [ 31 : 0]        rf_wd,  //写寄存器数据
    input                   [ 4 : 0]         debug_reg_ra,  //测试读取寄存器地址
 
    output                  [ 31 : 0]        debug_reg_rd,   //测试读取寄存器数据
    output                  [ 31 : 0]        rf_rd0,     //地址1读取数据
    output                  [ 31 : 0]        rf_rd1      //地址2读取数据
);
 
    reg [31 : 0] reg_file [0 : 31];
 
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            reg_file[i] = 0;
    end
 
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                reg_file[i] <= 0;
        end
        else begin
            if(rf_we)begin  //写
                if(rf_wa !=0 )begin    //注意0寄存器不能写
                    reg_file[rf_wa] <= rf_wd;
                end
            end
        end
    end
 
    assign rf_rd0 = reg_file[rf_ra0];   //此处为单周期就不管时序延迟了
    assign rf_rd1 = reg_file[rf_ra1];
    assign debug_reg_rd = reg_file[debug_reg_ra];
    
endmodule
