`timescale 1ns / 1ps


module CPU (
    input                   [ 0 : 0]            clk,
    input                   [ 0 : 0]            rst,

    input                   [ 0 : 0]            global_en,

/* ------------------------------ Memory (inst) ----------------------------- */
    output                  [31 : 0]            imem_raddr,
    input                   [31 : 0]            imem_rdata,

/* ------------------------------ Memory (data) ----------------------------- */
    input                   [31 : 0]            dmem_rdata, 
    output                  [ 0 : 0]            dmem_we,    
    output                  [31 : 0]            dmem_addr,  
    output                  [31 : 0]            dmem_wdata, 

/* ---------------------------------- Debug --------------------------------- */
    output                  [ 0 : 0]            commit,
    output                  [31 : 0]            commit_pc,
    output                  [31 : 0]            commit_inst,
    output                  [ 0 : 0]            commit_halt,
    output                  [ 0 : 0]            commit_reg_we,
    output                  [ 4 : 0]            commit_reg_wa,
    output                  [31 : 0]            commit_reg_wd,
    output                  [ 0 : 0]            commit_dmem_we,
    output                  [31 : 0]            commit_dmem_wa,
    output                  [31 : 0]            commit_dmem_wd,

    input                   [ 4 : 0]            debug_reg_ra,
    output                  [31 : 0]            debug_reg_rd
);
wire [ 0 : 0] commit_if     ;


reg [0:0] flush_fd = 0;

//IF
wire [31:0] pcadd4_if,pc_if,inst_if;

//ID
wire [31:0] pcadd4_id,pc_id,inst_id,imm_id,rf_rd0_id,rf_rd1_id;
wire [4:0]  alu_op_id,rf_wa_id,rf_ra0_id,rf_ra1_id;
wire [3:0]  dmem_access_id,br_type_id;
wire [1:0]  rf_wd_sel_id;
wire [0:0]  commit_id,rf_we_id,alu_src0_sel_id,alu_src1_sel_id;
//EX
wire [31:0] pcadd4_ex,pc_ex,inst_ex,rf_rd0_ex,rf_rd1_ex,imm_ex,alu_res_ex,rf_rd0_raw_ex,rf_rd1_raw_ex;
wire [4:0]  alu_op_ex,rf_ra0_ex,rf_ra1_ex,rf_wa_ex;
wire [3:0]  dmem_access_ex,br_type_ex;
wire [1:0]  rf_wd_sel_ex,npc_sel_ex;
wire [0:0]  alu_src0_sel_ex,alu_src1_sel_ex,rf_we_ex,commit_ex;
//MEM
wire [31:0] pcadd4_mem,pc_mem,inst_mem,alu_res_mem,dmem_rd_out_mem,rf_rd1_mem;
wire [4:0]  rf_wa_mem;
wire [3:0]  dmem_access_mem;
wire [1:0]  rf_wd_sel_mem;
wire [0:0]  rf_we_mem,commit_mem;
//WB
wire [31:0] pcadd4_wb,pc_wb,inst_wb,alu_res_wb,dmem_rd_out_wb;
wire [4:0]  rf_wa_wb;
wire [1:0]  rf_wd_sel_wb;
wire [0:0]  rf_we_wb,commit_wb;

wire [31:0] cur_npc;

wire [0:0] dmem_en,rf_rd0_fe,rf_rd1_fe;
wire [31:0] data_r0,data_r1,res0,res1,alu_res,wd_res,rf_rd0_fd,rf_rd1_fd;
wire [31:0] dmem_rd_out,dmem_rd_in,dmem_wd_out;

wire [0:0] stall_pc,stall_if_id,flush_if_id,flush_id_ex;

wire [1:0] npc_sel;

//段间寄存器
IF_ID cpu_if_id(
    .clk(clk),
    .en(global_en),
    .rst(rst),
    .pcadd4_if(pcadd4_if),
    .pc_if(pc_if),
    .inst_if(inst_if),
    .stall(stall_if_id),
    .flush(flush_if_id),  
    .commit_if(commit_if),
    .pcadd4_id(pcadd4_id),
    .pc_id(pc_id),
    .inst_id(inst_id),
    .commit_id(commit_id)
);

ID_EX cpu_id_ex(
    .clk(clk),
    .en(global_en),
    .rst(rst),
    .pcadd4_id(pcadd4_id),
    .pc_id(pc_id),
    .inst_id(inst_id),
    .rf_rd0_id(rf_rd0_id),
    .rf_rd1_id(rf_rd1_id),
    .rf_ra0_id(rf_ra0_id),
    .rf_ra1_id(rf_ra1_id),
    .imm_id(imm_id),
    .rf_wa_id(rf_wa_id),
    .rf_we_id(rf_we_id),
    .stall(0),
    .flush(flush_id_ex), 
    .commit_id(commit_id),
    .alu_op_id(alu_op_id),
    .dmem_access_id(dmem_access_id),
    .br_type_id(br_type_id),
    .rf_wd_sel_id(rf_wd_sel_id),
    .alu_src0_sel_id(alu_src0_sel_id),
    .alu_src1_sel_id(alu_src1_sel_id),
    .pcadd4_ex(pcadd4_ex),
    .pc_ex(pc_ex),
    .inst_ex(inst_ex),
    .rf_rd0_ex(rf_rd0_raw_ex),
    .rf_rd1_ex(rf_rd1_raw_ex),
    .rf_ra0_ex(rf_ra0_ex),
    .rf_ra1_ex(rf_ra1_ex),
    .imm_ex(imm_ex),
    .rf_wa_ex(rf_wa_ex),
    .commit_ex(commit_ex),
    .rf_we_ex(rf_we_ex),
    .alu_op_ex(alu_op_ex),
    .dmem_access_ex(dmem_access_ex),
    .br_type_ex(br_type_ex),
    .rf_wd_sel_ex(rf_wd_sel_ex),
    .alu_src0_sel_ex(alu_src0_sel_ex),
    .alu_src1_sel_ex(alu_src1_sel_ex)
);

EX_MEM cpu_ex_mem(
    .clk(clk),
    .en(global_en),
    .rst(rst),
    .pcadd4_ex(pcadd4_ex),
    .pc_ex(pc_ex),
    .inst_ex(inst_ex),
    .alu_res_ex(alu_res_ex),
    .rf_rd1_ex(rf_rd1_ex),
    .rf_wa_ex(rf_wa_ex),
    .rf_we_ex(rf_we_ex),
    .rf_wd_sel_ex(rf_wd_sel_ex),
    .dmem_access_ex(dmem_access_ex),
    .stall(0),
    .flush(0), 
    .commit_ex(commit_ex),
    .pcadd4_mem(pcadd4_mem),
    .pc_mem(pc_mem),
    .inst_mem(inst_mem),
    .alu_res_mem(alu_res_mem),
    .rf_rd1_mem(rf_rd1_mem),
    .rf_wa_mem(rf_wa_mem),
    .commit_mem(commit_mem),
    .rf_we_mem(rf_we_mem),
    .rf_wd_sel_mem(rf_wd_sel_mem),
    .dmem_access_mem(dmem_access_mem)
);

MEM_WB cpu_mem_wb(
    .clk(clk),
    .en(global_en),
    .rst(rst),
    .pcadd4_mem(pcadd4_mem),
    .pc_mem(pc_mem),
    .inst_mem(inst_mem),
    .alu_res_mem(alu_res_mem),
    .dmem_rd_out_mem(dmem_rd_out_mem),
    .rf_wa_mem(rf_wa_mem),
    .rf_we_mem(rf_we_mem),
    .rf_wd_sel_mem(rf_wd_sel_mem),
    .stall(0),
    .flush(0), 
    .commit_mem(commit_mem),
    .pcadd4_wb(pcadd4_wb),
    .pc_wb(pc_wb),
    .inst_wb(inst_wb),
    .alu_res_wb(alu_res_wb),
    .dmem_rd_out_wb(dmem_rd_out_wb),
    .rf_wa_wb(rf_wa_wb),
    .commit_wb(commit_wb),
    .rf_we_wb(rf_we_wb),
    .rf_wd_sel_wb(rf_wd_sel_wb)
);

Fowarding my_foward(
    .rf_we_mem(rf_we_mem),
    .rf_we_wb(rf_we_wb),
    .rf_wa_mem(rf_wa_mem),
    .rf_wa_wb(rf_wa_wb),
    .rf_wd_mem(alu_res_mem),
    .rf_wd_wb(wd_res),
    .rf_ra0_ex(rf_ra0_ex),
    .rf_ra1_ex(rf_ra1_ex),  
    .rf_rd0_fe(rf_rd0_fe),
    .rf_rd1_fe(rf_rd1_fe),
    .rf_rd0_fd(rf_rd0_fd),
    .rf_rd1_fd(rf_rd1_fd)
);

SegCtrl my_seg(
    .rf_we_ex(rf_we_ex),
    .rf_wd_sel_ex(rf_wd_sel_ex),
    .rf_wa_ex(rf_wa_ex),
    .rf_ra0_id(rf_ra0_id),
    .rf_ra1_id(rf_ra1_id),
    .npc_sel_ex(npc_sel),
    .stall_pc(stall_pc),
    .stall_if_id(stall_if_id),
    .flush_if_id(flush_if_id),
    .flush_id_ex(flush_id_ex)
);

PC my_pc (
    .clk    (clk        ),
    .rst    (rst        ),
    .en     (global_en  ), 
    .stall_pc(stall_pc),  
    .npc    (cur_npc    ),
    .pc     (pc_if     )
);

pc_adder my_adder(
    .pc(pc_if),
    .npc(pcadd4_if)
);

npc_mux my_npc_mux(
    .pc_add4(pcadd4_if),
    .pc_offset(alu_res_ex),
    .pc_j(alu_res_ex&-1),
    //.npc_sel(npc_sel_ex),
    .npc_sel(npc_sel),
    .npc(cur_npc)
);

INST_MEM my_im(
    .clk(clk),
    .a((pc_if - 32'h00400000) / 4),
    .d(0),
    .we(0),
    .spo(inst_if)
);

DATA_MEM my_dm(
    .clk(clk),
    //.a((alu_res_mem - 32'h80000000) / 4),
    .a(alu_res_mem / 4),
    .d(dmem_wd_out),
    .we(dmem_en),
    .spo(dmem_rd_in)
);

DECODER my_decoder(
    .inst(inst_id),
    .alu_op(alu_op_id),
    .dmem_access(dmem_access_id),
    .imm(imm_id),
    .rf_ra0(rf_ra0_id),
    .rf_ra1(rf_ra1_id),
    .rf_wa(rf_wa_id),
    .rf_we(rf_we_id),
    .rf_wd_sel(rf_wd_sel_id),
    .alu_src0_sel(alu_src0_sel_id),
    .alu_src1_sel(alu_src1_sel_id),
    .br_type(br_type_id)
);

MUX1 my_mux0(
    .src0(rf_rd0_ex),
    .src1(pc_ex),
    .sel(alu_src0_sel_ex),
    .res(res0)
);

MUX1 my_mux1(
    .src0(rf_rd1_ex),
    .src1(imm_ex),
    .sel(alu_src1_sel_ex),
    .res(res1)
);

MUX1 mux_rd0(
    .src0(rf_rd0_raw_ex),
    .src1(rf_rd0_fd),
    .sel(rf_rd0_fe),
    .res(rf_rd0_ex)
);

MUX1 mux_rd1(
    .src0(rf_rd1_raw_ex),
    .src1(rf_rd1_fd),
    .sel(rf_rd1_fe),
    .res(rf_rd1_ex)
);

MUX2 my_mux(
    .src0(pcadd4_wb),
    .src1(alu_res_wb),
    .src2(dmem_rd_out_wb),
    .src3(0),
    .sel(rf_wd_sel_wb),
    .res(wd_res)
);

BRANCH my_branch(
    .br_type(br_type_ex),
    .br_src0(rf_rd0_ex),
    .br_src1(rf_rd1_ex),
    .npc_sel(npc_sel)
);

SLU my_slu(
    .addr(alu_res_mem),
    .dmem_access(dmem_access_mem),   
    .rd_in(dmem_rd_in),
    .wd_in(rf_rd1_mem),
    .rd_out(dmem_rd_out_mem),
    .wd_out(dmem_wd_out),
    .wd_we(dmem_en)
);

ALU my_alu(
    .alu_src0(res0),
    .alu_src1(res1),
    .alu_op(alu_op_ex),
    .alu_res(alu_res_ex)
);

REG_FILE my_reg(
    .clk(clk),
    .rf_ra0(rf_ra0_id),
    .rf_ra1(rf_ra1_id),
    .rf_wa(rf_wa_wb),
    .rf_we(rf_we_wb),
    .rf_wd(wd_res),
    .debug_reg_ra(debug_reg_ra),
    .debug_reg_rd(debug_reg_rd),
    .rf_rd0(rf_rd0_id),
    .rf_rd1(rf_rd1_id)
);

/* -------------------------------------------------------------------------- */
/*                                    Commit                                  */
/* -------------------------------------------------------------------------- */


    assign commit_if = 1'H1;

    reg  [ 0 : 0]   commit_reg          ;
    reg  [31 : 0]   commit_pc_reg       ;
    reg  [31 : 0]   commit_inst_reg     ;
    reg  [ 0 : 0]   commit_halt_reg     ;
    reg  [ 0 : 0]   commit_reg_we_reg   ;
    reg  [ 4 : 0]   commit_reg_wa_reg   ;
    reg  [31 : 0]   commit_reg_wd_reg   ;
    reg  [ 0 : 0]   commit_dmem_we_reg  ;
    reg  [31 : 0]   commit_dmem_wa_reg  ;
    reg  [31 : 0]   commit_dmem_wd_reg  ;

    always @(posedge clk) begin
        if (rst) begin
            commit_reg          <= 1'H0;
            commit_pc_reg       <= 32'H0;
            commit_inst_reg     <= 32'H0;
            commit_halt_reg     <= 1'H0;
            commit_reg_we_reg   <= 1'H0;
            commit_reg_wa_reg   <= 5'H0;
            commit_reg_wd_reg   <= 32'H0;
            commit_dmem_we_reg  <= 1'H0;
            commit_dmem_wa_reg  <= 32'H0;
            commit_dmem_wd_reg  <= 32'H0;
        end
        else if (global_en) begin
            commit_reg          <= commit_wb;
            commit_pc_reg       <= pc_wb;       
            commit_inst_reg     <= inst_wb;      
            commit_halt_reg     <= (inst_wb == 32'h00100073);      
            commit_reg_we_reg   <= rf_we_wb;      
            commit_reg_wa_reg   <= rf_wa_wb;       
            commit_reg_wd_reg   <= wd_res;      
            commit_dmem_we_reg  <= dmem_en;
            commit_dmem_wa_reg  <= alu_res_wb;
            commit_dmem_wd_reg  <= dmem_wd_out;
        end
    end

    assign commit               = commit_reg;
    assign commit_pc            = commit_pc_reg;
    assign commit_inst          = commit_inst_reg;
    assign commit_halt          = commit_halt_reg;
    assign commit_reg_we        = commit_reg_we_reg;
    assign commit_reg_wa        = commit_reg_wa_reg;
    assign commit_reg_wd        = commit_reg_wd_reg;
    assign commit_dmem_we       = commit_dmem_we_reg;
    assign commit_dmem_wa       = commit_dmem_wa_reg;
    assign commit_dmem_wd       = commit_dmem_wd_reg;
endmodule



// 指令存储器（只读，同步读取）
module INST_MEM (
    input         clk,
    input  [31:0] a,    // 字地址（已处理为基于0的地址）
    input  [31:0] d,    // 数据输入（未使用）
    input         we,   // 写使能（始终为0）
    output reg[31:0] spo   // 数据输出
);  
    integer i;
    reg [31:0] mem [0:31];
    initial begin
        for (i=0; i<32; i=i+1)
            mem[i] = 32'h0;
        mem[0] = 32'b00000000010000000000000010010011;   //将零寄存器(x0)的值 +4 存入x1 ;  x1=4
        mem[1] = 32'b00000000100000000000000100010011;   //将零寄存器(x0)的值 +8 存入x2 ;   x2=8
        mem[2] = 32'b00000001000000000000000110010011;   //将零寄存器(x0)的值 +16 存入x3;   x3=16
        mem[3] = 32'b00000000001100010010000000100011;   //将reg[3]的值存入mem(reg[2]+0)  mem[8] = 16;   SW  实际上是存到了data_mem[2]处
        mem[4] = 32'b01000000000100010000001010110011;  //reg[2] - reg[1] 存入 reg[5]   x5=4
        mem[5] = 32'b00000000000000001001100001100011;  //bne  if reg[0] != reg[1]  pc+=16  //实际效果好像是+了8？规范问题？
    end
    
    
    // 同步读取
    always @(posedge clk) begin
        spo <=  mem[a];
    end
    
endmodule

// 数据存储器（读写，同步接口）
module DATA_MEM (
    input         clk,
    input  [31:0] a,     // 字地址（已处理为基于0的地址）
    input  [31:0] d,     // 写入数据
    input         we,    // 写使能
    output reg[31:0] spo    // 读取数据
);
    // 1KB数据存储器（256个32位字）
    reg [31:0] mem [0:31];
    
    // 同步读写操作
    always @(posedge clk) begin
        if (we && a < 256)  // 写操作优先级高于读
            mem[a] <= d;
        spo <= (a < 256) ? mem[a] : 32'h0;
    end
    
    integer i;
    initial begin
        for (i=0; i<32; i=i+1)
            mem[i] = 10*i;
    end
endmodule