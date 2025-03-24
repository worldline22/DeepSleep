`timescale 1ns / 1ps
module RISCVCore(
    input CLK,
    input Reset
);

    // PC模块信号
    wire [31:0] current_pc;
    reg [31:0] pre_pc;

    // InsMEM信号
    wire [7:0] op;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire [4:0] rs1, rs2, rd;
    wire [24:0] imm;

    // ControlUnit信号
    wire [2:0] alu_op;
    wire alu1_src, alu2_src;
    wire [1:0] reg_dst;
    wire [2:0] ext_sel;
    wire sign_ext;
    wire [1:0] digit;
    wire data_wr;
    wire imm_res;
    wire reg_wr;

    // Extend信号
    wire [31:0] extended_imm;

    // RegisterFile信号
    wire [31:0] db;
    wire [31:0] read_data1, read_data2;

    // ALU信号
    wire [31:0] alu_result;
    wire [1:0] alu_cmp;

    // Forwarding信号
    wire [31:0] read_data1_forwarded, read_data2_forwarded;
    reg [4:0] pre_rd, ppre_rd;
    reg pre_reg_wr, ppre_reg_wr;
    reg [1:0] pre_reg_dst, ppre_reg_dst;
    reg [1:0] pre_cmp, ppre_cmp;
    reg [31:0] pre_alu_output, ppre_alu_output;
    reg [31:0] ppre_data_out;

    // DataMEM信号
    wire [31:0] data_out;

    // Bubble信号
    wire mwk, pc_delay;
    reg [7:0] id_op;
    reg [4:0] id_rd;

    // 控制信号
    wire pcsrc;
    reg branch_cond_met;

    // PC模块实例化
    PC pc(
        .CLK(CLK),
        .Reset(Reset),
        .PCSrc(pcsrc),
        .AluOutput(alu_result),
        .PCdelay(pc_delay),
        .prePC(pre_pc),
        .curPC(current_pc)
    );

    // 指令存储器实例化
    InsMEM ins_mem(
        .CLK(CLK),
        .curPC(current_pc),
        .op(op),
        .funct3(funct3),
        .funct7(funct7),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .imm(imm),
        .instr()  // 可省略instr输出
    );

    // 控制单元实例化
    ControlUnit control(
        .CLK(CLK),
        .op(op),
        .funct3(funct3),
        .funct7(funct7),
        .AluOp(alu_op),
        .Alu1Src(alu1_src),
        .Alu2Src(alu2_src),
        .RegDst(reg_dst),
        .ExtSel(ext_sel),
        .Sign(sign_ext),
        .Digit(digit),
        .DataWr(data_wr),
        .immres(imm_res),
        .RegWr(reg_wr)
    );

    // 立即数扩展模块
    Extend extend(
        .imm(imm),
        .Sign(sign_ext),
        .ExtSel(ext_sel),
        .extend(extended_imm)
    );

    // 寄存器文件实例化
    RegisterFile reg_file(
        .CLK(CLK),
        .Mwk(mwk),
        .immres(imm_res),
        .rs1(rs1),
        .rs2(rs2),
        .WriteReg(rd),
        .AluOutput(alu_result),
        .extend(extended_imm),
        .Datain(data_out),
        .PC(current_pc),
        .cmp(alu_cmp),
        .RegDst(reg_dst),
        .RegWr(reg_wr),
        .DB(db),
        .ReadData1(read_data1),
        .ReadData2(read_data2)
    );

    // ALU输入多路选择
    wire [31:0] alu_in1 = alu1_src ? current_pc : read_data1_forwarded;
    wire [31:0] alu_in2 = alu2_src ? extended_imm : read_data2_forwarded;

    // ALU实例化
    ALU alu(
        .CLK(CLK),
        .operand1(alu_in1),
        .operand2(alu_in2),
        .AluOp(alu_op),
        .result(alu_result),
        .cmp(alu_cmp)
    );

    // 数据存储器实例化
    DataMEM data_mem(
        .Mwk(mwk),
        .DataWr(data_wr),
        .Digit(digit),
        .CLK(CLK),
        .DAddr(alu_result),
        .DataIn(read_data2_forwarded),
        .DataOut(data_out)
    );

    // 前递模块实例化
    Forwarding forwarding(
        .CLK(CLK),
        .rs1(rs1),
        .rs2(rs2),
        .ReadData1(read_data1),
        .ReadData2(read_data2),
        .preMwk(mwk),             // 假设前一周期的Mwk
        .preRegWr(pre_reg_wr),
        .preRegDst(pre_reg_dst),
        .prerd(pre_rd),
        .precmp(pre_cmp),
        .preAluOutput(pre_alu_output),
        .ppreMwk(1'b0),          // 暂时简化
        .ppreRegWr(ppre_reg_wr),
        .ppreRegDst(ppre_reg_dst),
        .pprerd(ppre_rd),
        .pprecmp(ppre_cmp),
        .ppreAluOutput(ppre_alu_output),
        .ppreDataOut(ppre_data_out),
        .RD1(read_data1_forwarded),
        .RD2(read_data2_forwarded)
    );

    // 流水暂停模块实例化
    always @(posedge CLK) begin
        if (Reset) begin
            id_op <= 0;
            id_rd <= 0;
        end else begin
            id_op <= op;
            id_rd <= rd;
        end
    end

    Bubble bubble(
        .CLK(CLK),
        .clear(Reset),
        .preop(id_op),
        .prerd(id_rd),
        .rs1(rs1),
        .rs2(rs2),
        .Mwk(mwk),
        .PCdelay(pc_delay)
    );

    // 分支条件判断逻辑
    always @(*) begin
        case (funct3)
            3'b000: branch_cond_met = (alu_cmp == 2'b00); // BEQ
            3'b001: branch_cond_met = (alu_cmp != 2'b00); // BNE
            3'b100: branch_cond_met = (alu_cmp == 2'b01); // BLT（有符号）
            3'b101: branch_cond_met = (alu_cmp == 2'b10); // BGE（有符号）
            3'b110: branch_cond_met = (read_data1_forwarded < read_data2_forwarded); // BLTU（无符号）
            3'b111: branch_cond_met = (read_data1_forwarded >= read_data2_forwarded); // BGEU（无符号）
            default: branch_cond_met = 0;
        endcase
    end

    // PCSrc生成
    assign pcsrc = (op == 7'b1100011 && branch_cond_met) ||  // B型
                   (op == 7'b1101111) ||                     // JAL
                   (op == 7'b1100111);                       // JALR

    // 流水线寄存器（存储前两周期信息）
    always @(posedge CLK) begin
        if (Reset) begin
            pre_rd <= 0;
            pre_reg_wr <= 0;
            pre_reg_dst <= 0;
            pre_cmp <= 0;
            pre_alu_output <= 0;
            ppre_rd <= 0;
            ppre_reg_wr <= 0;
            ppre_reg_dst <= 0;
            ppre_cmp <= 0;
            ppre_alu_output <= 0;
            ppre_data_out <= 0;
        end else begin
            // 更新前一周期信息
            pre_rd <= rd;
            pre_reg_wr <= reg_wr;
            pre_reg_dst <= reg_dst;
            pre_cmp <= alu_cmp;
            pre_alu_output <= alu_result;
            // 更新前两周期信息
            ppre_rd <= pre_rd;
            ppre_reg_wr <= pre_reg_wr;
            ppre_reg_dst <= pre_reg_dst;
            ppre_cmp <= pre_cmp;
            ppre_alu_output <= pre_alu_output;
            ppre_data_out <= data_out;
        end
    end

    // 更新pre_pc
    always @(posedge CLK) begin
        pre_pc <= current_pc;
    end

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
        {rom[3],rom[2],rom[1],rom[0]}     = 32'b00000000010000000000000010010011;   //ADDI x1, x0; 将零寄存器(x0)的值 +4 存入x1 
        {rom[7],rom[6],rom[5],rom[4]}     = 32'b00000000000000001000000100110011;   //ADD x3, x0, x1; 将x0与x1相加结果存入x3 
        {rom[11],rom[10],rom[9],rom[8]}   = 32'b00000000001000001010000100100011;   //SW x10, 4(x1); 将x10的值存入内存地址 x1 +4 =8
        {rom[15],rom[14],rom[13],rom[12]} = 32'b00000000001000001010000110000011;   //LW x1, 4(x1); 从内存地址 x1 +4 =8 加载数据到x1 → x1 = MEM[8]
        {rom[19],rom[18],rom[17],rom[16]} = 32'b00000000001000011000001000110011;   //ADD x5, x2, x17 ; 将x2与x17相加结果存入x5
        {rom[23],rom[22],rom[21],rom[20]} = 32'b00000000100000000000001010010011;   //ADDI x2, x0, 8; 将零寄存器(x0)的值 +8 存入x2
        {rom[27],rom[26],rom[25],rom[24]} = 32'b01000000000100101000001100110011;   //SUB x6, x1, x4; 将x1与x4相减结果存入x6
        {rom[31],rom[30],rom[29],rom[28]} = 32'b00000000000000000000000001100011;   //BEQ x0, x0, 12; 若x0与x0相等则跳转到PC+12
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
    //组合逻辑（可参考instruction_split.png)
    assign op = instr[6:0];
    assign rs1 = instr[19:15];
    assign rs2 = instr[24:20];
    assign rd = instr[11:7];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];
    assign imm = instr[31:7];  //这里是没有被处理过的数段(其中可能只有一部分代表立即数)
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
        output reg immres,       //rd是否选择imm直接作为数据
        output reg RegWr
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
                RegWr = 1;  // 需要写回寄存器
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
                RegWr = 1;
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
                RegWr = 1;  // 需要写回寄存器
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
                RegWr = 0;  
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
                RegWr = 0;  
            end

            // LUI指令
            7'b0110111: begin
                RegDst = 2'b00; // 结果来自立即数
                ExtSel = 3'b011; // U型
                immres = 1;     // 立即数直写
                RegWr = 1;  // 需要写回寄存器
            end

            // JAL指令
            7'b1101111: begin
                RegDst = 2'b10; // PC+4
                ExtSel = 3'b100; // J型
                Sign = 1;       // 符号扩展
                RegWr = 1;
            end

            // JALR指令
            7'b1100111: begin
                Alu1Src = 0;    // rs1
                Alu2Src = 1;    // 立即数
                RegDst = 2'b10; // PC+4
                ExtSel = 3'b000; // I型
                Sign = 1;       // 符号扩展
                AluOp = 3'b000; // ADD（计算目标地址）
                RegWr = 1;
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
// 补全的ALU模块
module ALU(
    input CLK,
    input [31:0] operand1,
    input [31:0] operand2,
    input [2:0] AluOp,
    output reg [31:0] result,
    output reg [1:0] cmp  // 00:相等 01:小于 10:大于
);
    always @(*) begin
        case (AluOp)
            3'b000: result = operand1 + operand2;      // ADD
            3'b001: result = operand1 - operand2;      // SUB
            3'b010: result = operand1 << operand2[4:0];// SLL
            3'b011: result = ($signed(operand1) < $signed(operand2)) ? 1 : 0; // SLT
            3'b100: result = (operand1 < operand2) ? 1 : 0; // SLTU
            3'b101: result = operand1 ^ operand2;      // XOR
            3'b110: result = operand1 >> operand2[4:0];// SRL
            3'b111: result = $signed(operand1) >>> operand2[4:0]; // SRA
            default: result = 32'b0;
        endcase
    end

    // 比较逻辑
    always @(*) begin
        if (operand1 == operand2)
            cmp = 2'b00;
        else if ($signed(operand1) < $signed(operand2))
            cmp = 2'b01;
        else
            cmp = 2'b10;
    end
endmodule

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

