`timescale 1ns / 1ps
//顶层模块
//这个模块目前是ai生成的，应该有很多错
module RISCVCore(
    input CLK,
    input Reset
);
    
    wire [31:0] curPC;
    wire [31:0] instr;
    wire [7:0] op;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire [4:0] rs1, rs2, rd;
    wire [24:0] imm;
    wire [31:0] extend;
    wire Sign;
    wire [2:0] ExtSel;
    wire [31:0] ReadData1, ReadData2;
    wire [31:0] AluOutput;
    wire PCSrc;
    wire immres;
    wire [1:0] RegDst;
    wire RegWr;
    wire [3:0] AluOp;
    wire Alu1Src, Alu2Src;
    wire [31:0] AluInput1, AluInput2;
    wire [31:0] DataMemOut;
    
    PC pc_inst(
        .CLK(CLK),
        .Reset(Reset),
        .PCSrc(PCSrc),
        .AluOutput(AluOutput),
        .PCdelay(1'b0),
        .prePC(32'b0),
        .curPC(curPC)
    );
    
    InsMEM insmem_inst(
        .CLK(CLK),
        .curPC(curPC),
        .op(op),
        .funct3(funct3),
        .funct7(funct7),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .imm(imm),
        .instr(instr)
    );
    
    Extend extend_inst(
        .imm(imm),
        .Sign(Sign),
        .ExtSel(ExtSel),
        .extend(extend)
    );
    
    RegisterFile regfile_inst(
        .CLK(CLK),
        .Mwk(1'b1),
        .immres(immres),
        .rs1(rs1),
        .rs2(rs2),
        .WriteReg(rd),
        .AluOutput(AluOutput),
        .extend(extend),
        .Datain(DataMemOut),
        .PC(curPC),
        .cmp(2'b00),
        .RegDst(RegDst),
        .RegWr(RegWr),
        .DB(),
        .ReadData1(ReadData1),
        .ReadData2(ReadData2)
    );
    
    ControlUnit control_inst(
        .CLK(CLK),
        .op(op),
        .funct3(funct3),
        .funct7(funct7),
        .AluOp(AluOp),
        .Alu1Src(Alu1Src),
        .Alu2Src(Alu2Src),
        .RegDst(RegDst),
        .ExtSel(ExtSel),
        .Sign(Sign),
        .Digit(),
        .DataWr(),
        .immres(immres)
    );
    
    assign AluInput1 = Alu1Src ? curPC : ReadData1;
    assign AluInput2 = Alu2Src ? extend : ReadData2;
    
    ALU alu_inst(
        .input1(AluInput1),
        .input2(AluInput2),
        .aluOp(AluOp),
        .result(AluOutput)
    );
    
    DataMemory datamem_inst(
        .CLK(CLK),
        .Address(AluOutput),
        .WriteData(ReadData2),
        .MemWrite(1'b0),
        .MemRead(1'b1),
        .ReadData(DataMemOut)
    );
    
endmodule


module PC(    //PC控制取指地址，送给指令存储器调出指令
        input CLK,               //时钟
        input Reset,             //是否重置地址。1-初始化PC，否则接受新地址
        input PCSrc,             //数据选择器输入，当为1时，PC接受ALU计算结果，否则PC+4自然向后跳转
        input [31:0] AluOutput,  //ALU计算结果
        input PCdelay,           //这是用来处理冒险的，1：暂停PC更新（锁定在之前的prePC) 0：正常更新PC
        input [31:0] prePC,
        output reg[31:0] curPC   //当前指令的地址
    );

    initial begin
        curPC <= -4; //初始值为-4
    end
    reg [31:0] tmp;

    always @(tmp)  begin
        curPC <= tmp;   //下一拍将当前指令地址更新为tmp(32位)
    end
    
    //检测时钟上升沿计算新指令地址 
    always@(posedge CLK)
    begin
        if (Reset | PCdelay) begin
            if (Reset) begin
                tmp <= 0;
            end
            else tmp <= prePC;
        end
        else begin
            case(PCSrc)   //仿真时
                1'b0:   tmp <= curPC + 4;
                1'b1:   tmp <= AluOutput;
            endcase
        end
    end
endmodule


module InsMEM(    //指令存储器
        input CLK,
        input [31:0] curPC,      //PC值
        output reg[7:0] op,      //操作码位段
        output reg[2:0] funct3,  //3位功能码位段
        output reg[6:0] funct7,  //7位功能码位段
        output reg[4:0] rs1,     //rs1地址位段
        output reg[4:0] rs2,     //rs2地址位段
        output reg[4:0] rd,      //rd地址位段
        output reg[24:0] imm,    //立即数位段传给extend模块拼接扩展
        output reg [31:0] instr  //读取得到32位指令
    );
    reg [7:0] rom[128:0];  //存储器定义必须用reg类型，存储器存储单元8位长度，共128个存储单元，可以存32条指令
    //这里存储器大小可调
    // 加载数据到存储器rom（仿真检测时可用）
    initial begin
        {rom[3],rom[2],rom[1],rom[0]}     = 32'b00000000010000000000000010010011;
        {rom[7],rom[6],rom[5],rom[4]}     = 32'b00000000000000001000000100110011;
        {rom[11],rom[10],rom[9],rom[8]}   = 32'b00000000001000001010000100100011;
        {rom[15],rom[14],rom[13],rom[12]} = 32'b00000000001000001010000110000011;
        {rom[19],rom[18],rom[17],rom[16]} = 32'b00000000001000011000001000110011;
        {rom[23],rom[22],rom[21],rom[20]} = 32'b00000000100000000000001010010011;
        {rom[27],rom[26],rom[25],rom[24]} = 32'b01000000000100101000001100110011;
        {rom[31],rom[30],rom[29],rom[28]} = 32'b00000000000000000000000001100011;

        op = 7'b0000000;
        funct3 = 3'b000;
        funct7 = 7'b0000000;
        rs1 = 5'b00000;
        rs2 = 5'b00000;
        imm = 20'b00000000000000000000;
    end

    //小端模式,下降沿写入指令(应该是为了和controlunit合在一拍完成（即下降沿切分好指令，然后上升沿完成全部译码工作)
    always@(negedge CLK)   
    begin
        //取指令
        begin
            instr[7:0] = rom[curPC];
            instr[15:8] = rom[curPC + 1];
            instr[23:16] = rom[curPC + 2];
            instr[31:24] = rom[curPC + 3];
        end 
    end
    //切割指令
    always_comb   //组合逻辑（可参考instruction_split.png)
    begin
        op = instr[6:0];
        rs1 = instr[19:15];
        rs2 = instr[24:20];
        rd = instr[11:7];
        funct3 = instr[14:12];
        funct7 = instr[31:26];
        imm = instr[31:7];  //这里是没有被处理过的数段(其中可能只有一部分代表立即数)
    end
    //R型指令：寄存器类型指令，用于在寄存器之间执行算术、逻辑和比较运算。这些指令使用三个寄存器参数。
    //I型指令：立即数类型指令，用于在寄存器和立即数（常数）之间执行算术、逻辑、移位和分支等操作。这些指令使用两个寄存器参数和一个立即数参数。
    //S型指令：存储类型指令，用于将寄存器中的数据存储到存储器中。这些指令使用两个寄存器参数和一个偏移量参数。
    //B型指令：分支类型指令，用于根据条件跳转到不同的代码段。这些指令使用两个寄存器参数和一个偏移量参数。
    //U型指令：无条件跳转类型指令，用于无条件跳转到指定的地址。这些指令使用一个立即数参数。
    //J型指令：跳转类型指令，用于跳转到指定的地址。这些指令使用一个寄存器参数和一个偏移量参数。

endmodule


//用于处理立即数（应用于含立即数指令类（I、S、B、U、J））,主要就是把立即数拼回去
module Extend(      
        input [24:0] imm,    //立即数位段
        input Sign,          //扩展符号控制信号  //这个应该是用于处理减法？
        input [2:0] ExtSel,  //立即数拼接方式
        output reg [31:0] extend //扩展完成立即数
    );

    always@(imm or ExtSel or Sign)  //应该会综合为组合逻辑
    begin
        //$display("imm:%d,extsel:%d,sign:%d",imm,ExtSel,Sign);
        case (ExtSel)
            3'b000://I指令直接拓展
            begin
                extend[11:0] = imm[24:13];
                extend[31:12] = Sign ? (imm[24] ? 20'hfffff : 20'h00000) : 20'h00000;
            end
            3'b101://I指令截取
            begin
                extend[4:0] = imm[16:13];
                extend[31:5] = 27'b000000000000000000000000000;
            end
            3'b001://S指令直接
            begin
                extend[4:0] = imm[4:0];
                extend[11:5] = imm[24:18];
                extend[31:12] = Sign ? (imm[24] ? 20'hfffff : 20'h00000) : 20'h00000;
            end
            3'b010://B指令直接
            begin
                extend[0] = 0;
                extend[11] = imm[0];
                extend[4:1] = imm[4:1];
                extend[10:5] = imm[23:18];
                extend[12] = imm[24];
                extend[31:13] = Sign ? (imm[24] ? 19'b1111111111111111111 : 19'b0000000000000000000) : 19'b0000000000000000000;
            end
            3'b011://U指令直接
            begin
                extend[11:0] = 12'h000;
                extend[31:12] = imm[24:5];
            end
            3'b100://J指令直接
            begin
                extend[0] = 0;
                extend[19:12] = imm[12:5];
                extend[11] = imm[13];
                extend[10:1] = imm[23:14];
                extend[20] = imm[24];
                extend[31:21] = Sign ? (imm[24] ? 11'b11111111111 : 11'b00000000000) : 11'b00000000000;
            end
        endcase
        //$display("extend:%d",extend);
    end
endmodule


//寄存器组
module RegisterFile(
        input CLK,              //时钟信号
        input Mwk,              //工作使能(为1时才能对寄存器组做操作，应该是和空指令（流水停滞)配合），不考虑冒险时可以先置1测试
        input immres,           //立即数直接写入信号
        input [4:0] rs1,        //rs1寄存器地址输入端口
        input [4:0] rs2,        //rs2寄存器地址输入端口
        input [4:0] WriteReg,   //rd输入地址
        input [31:0] AluOutput, //ALU输出
        input [31:0] extend,    //立即数扩展器输出
        input [31:0] Datain,    //存储器输出
        input [31:0] PC,        //当前PC值
        input [1:0] cmp,        //比较器输出
        input [1:0] RegDst,     //输入具体数据位选
        input RegWr,            //写使能信号，时钟下降沿触发写入
        output reg[31:0] DB,    //输入总线数据
        output [31:0] ReadData1, //rs1寄存器数据输出端口
        output [31:0] ReadData2  //rs2寄存器数据输出端口
    );

    reg [31:0] regFile[0:31]; //声明32个32位寄存器
    integer i;
    
    initial begin
        for (i = 0; i < 32; i = i+ 1) regFile[i] <= 0;//初始化  
    end

    assign ReadData1 = regFile[rs1];
    assign ReadData2 = regFile[rs2];

    always@(posedge CLK) 
    begin
        if (Mwk) begin
            if(immres & RegWr) begin
                regFile[WriteReg] = extend;
                DB = extend;
            end
            else begin
                if(RegWr)begin
                    case (RegDst)
                        2'b00: begin
                            regFile[WriteReg] = AluOutput;
                            DB = AluOutput;
                        end 
                        2'b01: begin
                            regFile[WriteReg] = Datain;
                            DB = Datain;
                        end 
                        2'b10: begin
                            regFile[WriteReg] = PC+4;
                            DB = PC+4;
                        end 
                        2'b11: begin
                            regFile[WriteReg][31:1] = 0;
                            regFile[WriteReg][0] = cmp[0];
                            DB[31:1] = 0;
                            DB[0] = cmp[0];
                            //$display("cmp:%d, db:%d",cmp,DB);
                        end
                    endcase
                end
            end
        end
        //$display("immres = %d ; in %d write %d",immres,WriteReg,regFile[WriteReg]);
    end
endmodule


//Control Unit
module ControlUnit(
        input CLK,              //时钟信号
        input [7:0] op,         //操作码
        input [2:0] funct3,     //3位功能码
        input [6:0] funct7,     //7位功能码
        output reg[2:0] AluOp,  //ALU操作方式
        output reg Alu1Src,     //ALU1口位选
        output reg Alu2Src,     //ALU2口位选
        output reg[1:0] RegDst, //Rd输入数据来源
        output reg[2:0] ExtSel, //立即数拼接方式，和指令型有关
        output reg Sign,        //立即数符号扩展信号
        output reg[1:0] Digit,  //读写位数
        output reg DataWr,      //存储器写使能
        output reg immres       //rd是否选择imm直接作为数据
    );
    
    always @(*) begin
        // 默认值
        AluOp = 3'b000;
        Alu1Src = 0;
        Alu2Src = 0;
        RegDst = 2'b00;
        ExtSel = 3'b000;
        Sign = 0;
        Digit = 2'b10;
        DataWr = 0;
        immres = 0;

        case (op)
            // R-type指令（ADD、SUB等）
            7'b0110011: begin
                Alu1Src = 0;    // rs1
                Alu2Src = 0;    // rs2
                RegDst = 2'b00; // ALU结果
                case (funct3)
                    3'b000: // ADD/SUB
                        AluOp = (funct7[5]) ? 3'b001 : 3'b000;
                    3'b001: // SLL
                        AluOp = 3'b010;
                    3'b010: // SLT
                        AluOp = 3'b011;
                    3'b011: // SLTU
                        AluOp = 3'b100;
                    3'b100: // XOR
                        AluOp = 3'b101;
                    3'b101: // SRL/SRA
                        AluOp = (funct7[5]) ? 3'b111 : 3'b110;
                    3'b110: // OR
                        AluOp = 3'b110;
                    3'b111: // AND
                        AluOp = 3'b111;
                endcase
            end

            // I-type指令（ADDI、SLLI等）
            7'b0010011: begin
                Alu1Src = 0;    // rs1
                Alu2Src = 1;    // 立即数
                RegDst = 2'b00; // ALU结果
                ExtSel = 3'b000; // I型
                case (funct3)
                    3'b000: begin // ADDI
                        AluOp = 3'b000;
                        Sign = 1;
                    end
                    3'b001: begin // SLLI
                        AluOp = 3'b010;
                        Sign = 0; // 零扩展
                    end
                    3'b010: begin // SLTI
                        AluOp = 3'b011;
                        Sign = 1;
                    end
                    3'b011: begin // SLTIU
                        AluOp = 3'b100;
                        Sign = 1;
                    end
                    3'b100: begin // XORI
                        AluOp = 3'b101;
                        Sign = 1;
                    end
                    3'b101: begin // SRLI/SRAI
                        AluOp = (funct7[5]) ? 3'b111 : 3'b110;
                        Sign = 0; // 零扩展
                    end
                    3'b110: begin // ORI
                        AluOp = 3'b110;
                        Sign = 1;
                    end
                    3'b111: begin // ANDI
                        AluOp = 3'b111;
                        Sign = 1;
                    end
                endcase
            end

            // Load指令（LW、LH等）
            7'b0000011: begin
                Alu1Src = 0;    // rs1（基址）
                Alu2Src = 1;    // 立即数（偏移）
                RegDst = 2'b01; // 存储器数据
                ExtSel = 3'b000; // I型
                Sign = 1;       // 符号扩展
                case (funct3)
                    3'b000: Digit = 2'b00; // LB
                    3'b001: Digit = 2'b01; // LH
                    3'b010: Digit = 2'b10; // LW
                    3'b100: Digit = 2'b00; // LBU（无符号）
                    3'b101: Digit = 2'b01; // LHU（无符号）
                endcase
            end

            // Store指令（SW、SH等）
            7'b0100011: begin
                Alu1Src = 0;    // rs1（基址）
                Alu2Src = 1;    // 立即数（偏移）
                ExtSel = 3'b001; // S型
                Sign = 1;       // 符号扩展
                DataWr = 1;     // 写使能
                case (funct3)
                    3'b000: Digit = 2'b00; // SB
                    3'b001: Digit = 2'b01; // SH
                    3'b010: Digit = 2'b10; // SW
                endcase
            end

            // B-type指令（BEQ、BNE等）
            7'b1100011: begin
                Alu1Src = 0;    // rs1
                Alu2Src = 0;    // rs2
                ExtSel = 3'b010; // B型
                Sign = 1;       // 符号扩展
            end

            // LUI指令
            7'b0110111: begin
                RegDst = 2'b00; // 结果来自立即数
                ExtSel = 3'b011; // U型
                immres = 1;     // 立即数直写
            end

            // JAL指令
            7'b1101111: begin
                RegDst = 2'b10; // PC+4
                ExtSel = 3'b100; // J型
                Sign = 1;       // 符号扩展
            end

            // JALR指令
            7'b1100111: begin
                Alu1Src = 0;    // rs1
                Alu2Src = 1;    // 立即数
                RegDst = 2'b10; // PC+4
                ExtSel = 3'b000; // I型
                Sign = 1;       // 符号扩展
                AluOp = 3'b000; // ADD（计算目标地址）
            end

            // 其他指令（默认处理）
            default: begin
                // 保持默认值
            end
        endcase
    end
endmodule






//位于ID译码上升沿阶段，当上一条指令是load且出现RAW时
//把本条PC送回PC模块使PC延时
//把功能块工作使能 Mwk(module work)置0，使该指令对后续功能块失效
//以硬件结构分析指令判断是否有数据冒险，而不是编译器
module Bubble(
        input CLK,  //时钟脉冲
        input clear,
        input [7:0] preop,
        input [4:0] prerd,
        input [4:0] rs1,
        input [4:0] rs2,
        output reg Mwk,
        output reg PCdelay
    );
    reg Mwktmp;
    initial begin
        Mwk <= 0;
        PCdelay <= 0;
    end
    always @(negedge CLK) begin
        Mwk <= Mwktmp;
    end
    //检测上一条指令是load（I type2指令）
    always @(preop or clear) begin
        if (clear) begin
            Mwktmp <= 0;
            PCdelay <= 0;
        end
        else begin
            if (preop == 7'b0000011) begin
                if (rs1 == prerd) begin
                    Mwktmp <= 1;
                    PCdelay <= 1;
                end
                else if (rs2 == prerd) begin
                    Mwktmp <= 1;
                    PCdelay <= 1;
                end
                else begin
                    Mwktmp <= 1;
                    PCdelay <= 0;
                end
            end
            else begin
                Mwktmp <= 1;
                PCdelay <= 0;
            end
        end
    end
endmodule










//以下部分尚没有修改


//转发模块
//在EXE阶段的上升沿开始工作
//用于解决数据冒险（Data Hazard）问题，
//当 EXE 阶段需要使用之前尚未写回寄存器的运算结果时，数据转发逻辑可以将数据直接从 ALU 或数据存储器中取出，而不是等待写回阶段完成。
//preMwk：标志前一条指令是否有效。
//preRegWr：前一条指令是否需要写回寄存器。
//preRegDst：指示前一条指令写回的数据来源：
module Forwarding(
        input CLK,  //时钟脉冲
        input [4:0] rs1,
        input [4:0] rs2,
        input [31:0] ReadData1,
        input [31:0] ReadData2,
        input preMwk, //pre
        input preRegWr,
        input [1:0] preRegDst,
        input [4:0] prerd,
        input [1:0] precmp,
        input [31:0] preAluOutput,
        input ppreMwk,//ppre
        input ppreRegWr,
        input [1:0] ppreRegDst,
        input [4:0] pprerd,
        input [1:0] pprecmp,
        input [31:0] ppreAluOutput,
        input [31:0] ppreDataOut,

        output reg [31:0] RD1,
        output reg [31:0] RD2
    );
    initial begin
        RD1 <= 0;
        RD2 <= 0;
    end
    always @(posedge CLK) begin
        //处理rs1冒险
        RD1 = ReadData1;
        if (ppreMwk) begin
            if (pprerd == rs1) begin
                if(ppreRegWr)begin
                    case (ppreRegDst)
                        2'b00: begin
                            RD1 = ppreAluOutput;
                        end 
                        2'b01: begin
                            RD1 = ppreDataOut;
                        end 
                        2'b10: begin
                            // RD1 = PC+4;
                            RD1 = ReadData1;
                        end 
                        2'b11: begin
                            RD1[31:1] = 0;
                            RD1[0] = pprecmp[0];
                        end
                    endcase
                end
            end
        end

        if (preMwk) begin
            if (prerd == rs1) begin
                if(preRegWr)begin
                    case (preRegDst)
                        2'b00: begin
                            RD1 = preAluOutput;
                        end 
                        2'b10: begin
                            // RD1 = PC+4;
                            RD1 = ReadData1;
                        end 
                        2'b11: begin
                            RD1[31:1] = 0;
                            RD1[0] = precmp[0];
                        end
                    endcase
                end
            end
        end
        //处理rs2冒险
        RD2 = ReadData2;
        if (ppreMwk) begin
            if (pprerd == rs2) begin
                if(ppreRegWr)begin
                    case (ppreRegDst)
                        2'b00: begin
                            RD2 = ppreAluOutput;
                        end 
                        2'b01: begin
                            RD2 = ppreDataOut;
                        end 
                        2'b10: begin
                            // RD2 = PC+4;
                            RD2 = ReadData2;
                        end 
                        2'b11: begin
                            RD2[31:1] = 0;
                            RD2[0] = pprecmp[0];
                        end
                    endcase
                end
            end
        end

        if (preMwk) begin
            if (prerd == rs2) begin
                if(preRegWr)begin
                    case (preRegDst)
                        2'b00: begin
                            RD2 = preAluOutput;
                        end 
                        2'b10: begin
                            // RD2 = PC+4;
                            RD2 = ReadData2;
                        end 
                        2'b11: begin
                            RD2[31:1] = 0;
                            RD2[0] = precmp[0];
                        end
                    endcase
                end
            end
        end
    end
endmodule
5-7 ALU内控制信号部分

    always @(negedge CLK) begin
        if (Mwk) begin
            PCSrc <= PCSrctmp;
        end
        else begin
            PCSrc <= 0;
        end
    end
    always @(cmp or op or funct3 or Mwk) begin
        if (Mwk) begin
            clear = 0;
            if(op == 7'b1100011) begin
                RegWr <= 0;
                case(funct3)
                    //beq
                    3'b000:
                    begin
                        if (cmp == 2'b00)
                        begin
                            PCSrctmp = 1;
                            clear = 1;
                        end
                        else 
                            PCSrctmp = 0;
                    end
                    //bne
                    3'b001:
                    begin
                        if (cmp != 2'b00) begin
                            PCSrctmp = 1;
                            clear = 1;
                        end
                        else 
                            PCSrctmp = 0;
                    end
                    //blt
                    3'b100:
                    begin
                        if (cmp == 2'b01) begin
                            PCSrctmp = 1;
                            clear = 1;
                        end
                        else
                            PCSrctmp = 0;
                    end
                    //bge
                    3'b101:
                    begin
                        if (cmp == 2'b10) begin
                            PCSrctmp = 1;
                            clear = 1;
                        end
                        else
                            PCSrctmp = 0;
                    end
                    //bltu
                    3'b110:
                    begin
                        if (cmp == 2'b01) begin
                            PCSrctmp = 1;
                            clear = 1;
                        end
                        else
                            PCSrctmp = 0;
                    end
                    //bgeu
                    3'b111:
                    begin
                        if (cmp == 2'b10) begin
                            PCSrctmp = 1;
                            clear = 1;
                        end
                        else
                            PCSrctmp = 0;
                    end
                endcase
            end
            else begin
                if (op == 7'b0100011) begin
                    RegWr <= 0;
                    PCSrctmp <= 0;
                end
                else begin
                    //jal
                    if (op == 7'b1101111) begin
                        PCSrctmp <= 1;
                        clear <= 1;
                        RegWr <= 1;
                    end
                    else begin
                        if (op == 7'b1100111) begin
                            PCSrctmp <= 1;
                            clear <= 1;
                            RegWr <= 1;
                        end
                        else begin
                            PCSrctmp <= 0;
                            RegWr <= 1;
                            clear <= 0;
                        end
                    end
                end
            end
        end
        else begin
            clear <= 0;
            RegWr <= 0;
        end
    end
5-8 DataMEM实现代码

`timescale 1ns / 1ps
//data memory 数据存储器
module DataMEM(
        input Mwk,
        input DataWr,        //存储器写使能
        input [1:0] Digit,   //读写位数，00为8位，01为16位，10为32位
        input CLK,           //时钟信号，下降沿写入数据
        input [31:0] DAddr,  //写入存储器地址
        input [31:0] DataIn, //写入存储器数据
        output reg[31:0] DataOut //输出数据
    );

    reg [7:0] ram [0:31];     // 存储器
    
    //下降沿读出数据写入寄存器
    always@(negedge CLK)
    begin
        DataOut[7:0] <= ram[DAddr + 3];
        DataOut[15:8] <= ram[DAddr + 2];     
        DataOut[23:16] <= ram[DAddr + 1];     
        DataOut[31:24] <= ram[DAddr];
    end

    //上升沿把数据写入MEM
    always@(posedge CLK)
    begin   
        if (Mwk) begin
            if(DataWr) //写使能为1时写入数据
            begin
                ram[DAddr] <= DataIn[31:24];    
                ram[DAddr + 1] <= DataIn[23:16];
                ram[DAddr + 2] <= DataIn[15:8];     
                ram[DAddr + 3] <= DataIn[7:0];    
            end
        end
    end
endmodule

