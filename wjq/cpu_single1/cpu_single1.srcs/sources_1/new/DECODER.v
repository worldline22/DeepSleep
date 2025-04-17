`timescale 1ns / 1ps
module DECODER (
    input                   [31 : 0]            inst,  //32位指令

    output     reg             [ 4 : 0]            alu_op,  //传给ALU，表示运算类型
    output     reg             [31 : 0]            imm,     //统一化为32bit的立即数
 
    output     reg             [ 4 : 0]            rf_ra0,  //读寄存器地址
    output     reg             [ 4 : 0]            rf_ra1,  //读寄存器地址
    output     reg             [ 4 : 0]            rf_wa,    //写寄存器地址
    output     reg             [ 0 : 0]            rf_we,    //写使能信号
 
    output      reg            [ 0 : 0]            alu_src0_sel,   
    output      reg            [ 0 : 0]            alu_src1_sel
);
 
always @(*)begin
    //add
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b000)begin
        alu_op = 5'b00000;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
    //addi
    if(inst[6:0] == 7'b0010011 && inst[14:12] == 3'b000)begin
        alu_op = 5'b00000;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //sub
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0100000 && inst[14:12] == 3'b000)begin
        alu_op = 5'b00010;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
    //slt
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b010)begin
        alu_op = 5'b00100;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
    //sltu
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b011)begin
        alu_op = 5'b00101;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
    //and
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b111)begin
        alu_op = 5'b01001;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
    //or
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b110)begin
        alu_op = 5'b01010;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
    //xor
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b100)begin
        alu_op = 5'b01011;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end  
    //sll
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b001)begin
        alu_op = 5'b01110;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end 
    //srl
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b01111;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end 
    //sra
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0100000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b10000;
        imm = 0;
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
    //slli
    if(inst[6:0] == 7'b0010011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b001)begin
        alu_op = 5'b01110;
        imm = {{27{1'b0}},inst[24:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //srli
    if(inst[6:0] == 7'b0010011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b01111;
        imm = {{27{1'b0}},inst[24:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //srai
    if(inst[6:0] == 7'b0010011 && inst[31:25] == 7'b0100000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b10000;
        imm = {{27{1'b0}},inst[24:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //slti
    if(inst[6:0] == 7'b0010011 &&  inst[14:12] == 3'b010)begin
        alu_op = 5'b00100;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //sltiu
    if(inst[6:0] == 7'b0010011 &&  inst[14:12] == 3'b011)begin
        alu_op = 5'b00101;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //andi
    if(inst[6:0] == 7'b0010011 &&  inst[14:12] == 3'b111)begin
        alu_op = 5'b01001;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end  
    //ori
    if(inst[6:0] == 7'b0010011 &&  inst[14:12] == 3'b110)begin
        alu_op = 5'b01010;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end 
    //xori
    if(inst[6:0] == 7'b0010011 &&  inst[14:12] == 3'b100)begin
        alu_op = 5'b01011;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1 = inst[24:20];
        rf_ra0 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //lui
    if(inst[6:0] == 7'b0110111)begin
        alu_op = 5'b00000;
        imm = {{inst[31:12]},{12{1'b0}}};
        rf_ra0 = 0;
        rf_ra1 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 0;
        alu_src1_sel = 1;
    end
    //auipc
    if(inst[6:0] == 7'b0010111)begin
        alu_op = 5'b00000;
        imm = {{inst[31:12]},{12{1'b0}}};
        rf_ra0 = inst[24:20];
        rf_ra1 = inst[19:15];
        rf_wa = inst[11:7];
        rf_we = 1;
        alu_src0_sel = 1;
        alu_src1_sel = 1;
    end
    //ebreak
    if(inst == 32'h00100073)begin
        alu_op = 5'b11111;
        imm = 0;
        rf_ra0 = 0;
        rf_ra1 = 0;
        rf_wa = 0;
        rf_we = 0;
        alu_src0_sel = 0;
        alu_src1_sel = 0;
    end
end
endmodule
