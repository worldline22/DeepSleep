module DECODER (
    input                   [31 : 0]            inst,

    output      reg         [ 4 : 0]            alu_op,

    output                  [ 3 : 0]            dmem_access,

    output      reg         [31 : 0]            imm,

    output                  [ 4 : 0]            rf_ra0,
    output                  [ 4 : 0]            rf_ra1,
    output                  [ 4 : 0]            rf_wa,
    output                  [ 0 : 0]            rf_we,
    output      reg         [ 1 : 0]            rf_wd_sel,

    output                  [ 0 : 0]            alu_src0_sel,
    output                  [ 0 : 0]            alu_src1_sel,

    output                  [ 3 : 0]            br_type
);

reg [3:0] dmem_access_reg,br_type_reg;
reg [4:0] rf_ra0_reg,rf_ra1_reg,rf_wa_reg;
reg [0:0] rf_we_reg,alu_src0_sel_reg,alu_src1_sel_reg;

always @(*)begin
    //add
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b000)begin
        alu_op = 5'b00000;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
    //addi
    if(inst[6:0] == 7'b0010011 && inst[14:12] == 3'b000)begin
        alu_op = 5'b00000;
        dmem_access_reg = 4'b1111;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //sub
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0100000 && inst[14:12] == 3'b000)begin
        alu_op = 5'b00010;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
    //slt
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b010)begin
        alu_op = 5'b00100;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
    //sltu
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b011)begin
        alu_op = 5'b00101;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
    //and
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b111)begin
        alu_op = 5'b01001;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
    //or
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b110)begin
        alu_op = 5'b01010;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
    //xor
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b100)begin
        alu_op = 5'b01011;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end  
    //sll
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b001)begin
        alu_op = 5'b01110;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end 
    //srl
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b01111;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end 
    //sra
    if(inst[6:0] == 7'b0110011 && inst[31:25] == 7'b0100000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b10000;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
    //slli
    if(inst[6:0] == 7'b0010011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b001)begin
        alu_op = 5'b01110;
        dmem_access_reg = 4'b1111;
        imm = {{27{1'b0}},inst[24:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //srli
    if(inst[6:0] == 7'b0010011 && inst[31:25] == 7'b0000000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b01111;
        dmem_access_reg = 4'b1111;
        imm = {{27{1'b0}},inst[24:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //srai
    if(inst[6:0] == 7'b0010011 && inst[31:25] == 7'b0100000 && inst[14:12] == 3'b101)begin
        alu_op = 5'b10000;
        dmem_access_reg = 4'b1111;
        imm = {{27{1'b0}},inst[24:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //slti
    if(inst[6:0] == 7'b0010011 && inst[14:12] == 3'b010)begin
        alu_op = 5'b00100;
        dmem_access_reg = 4'b1111;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //sltiu
    if(inst[6:0] == 7'b0010011 && inst[14:12] == 3'b011)begin
        alu_op = 5'b00101;
        dmem_access_reg = 4'b1111;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //andi
    if(inst[6:0] == 7'b0010011 && inst[14:12] == 3'b111)begin
        alu_op = 5'b01001;
        dmem_access_reg = 4'b1111;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end  
    //ori
    if(inst[6:0] == 7'b0010011 && inst[14:12] == 3'b110)begin
        alu_op = 5'b01010;
        dmem_access_reg = 4'b1111;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end 
    //xori
    if(inst[6:0] == 7'b0010011 && inst[14:12] == 3'b100)begin
        alu_op = 5'b01011;
        dmem_access_reg = 4'b1111;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //lui
    if(inst[6:0] == 7'b0110111)begin
        alu_op = 5'b00000;
        dmem_access_reg = 4'b1111;
        imm = {{inst[31:12]},{12{1'b0}}};
        rf_ra0_reg = 0;
        rf_ra1_reg = 0;
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
    //auipc
    if(inst[6:0] == 7'b0010111)begin
        alu_op = 5'b00000;
        dmem_access_reg = 4'b1111;
        imm = {{inst[31:12]},{12{1'b0}}};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b01;
        alu_src0_sel_reg = 1;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
    end
   //lw
   if(inst[6:0] == 7'b0000011 && inst[14:12] == 3'b010)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b0000;
       imm = {{20{inst[31]}},inst[31:20]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 1;
       rf_wd_sel = 2'b10;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1111;
   end
    //lh
   if(inst[6:0] == 7'b0000011 && inst[14:12] == 3'b001)begin
        alu_op = 5'b00000;
        dmem_access_reg = 4'b0001;
        imm = {{20{inst[31]}},inst[31:20]};
        rf_ra1_reg = inst[24:20];
        rf_ra0_reg = inst[19:15];
        rf_wa_reg = inst[11:7];
        rf_we_reg = 1;
        rf_wd_sel = 2'b10;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 1;
        br_type_reg = 4'b1111;
   end
   //lb
   if(inst[6:0] == 7'b0000011 && inst[14:12] == 3'b000)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b0010;
       imm = {{20{inst[31]}},inst[31:20]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 1;
       rf_wd_sel = 2'b10;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1111;
   end
   //lhu
      if(inst[6:0] == 7'b0000011 && inst[14:12] == 3'b101)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b0100;
       imm = {{20{inst[31]}},inst[31:20]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 1;
       rf_wd_sel = 2'b10;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1111;
   end
   //lbu
      if(inst[6:0] == 7'b0000011 && inst[14:12] == 3'b100)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b0011;
       imm = {{20{inst[31]}},inst[31:20]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 1;
       rf_wd_sel = 2'b10;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1111;
   end
   //sw
   if(inst[6:0] == 7'b0100011 && inst[14:12] == 3'b010)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1000;
       imm = {{20{inst[31]}},inst[31:25],inst[11:7]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b11;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1111;
   end
    //sh
   if(inst[6:0] == 7'b0100011 && inst[14:12] == 3'b001)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1001;
       imm = {{20{inst[31]}},inst[31:25],inst[11:7]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b11;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1111;
   end
    //sb
   if(inst[6:0] == 7'b0100011 && inst[14:12] == 3'b000)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1011;
       imm = {{20{inst[31]}},inst[31:25],inst[11:7]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b11;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1111;
   end
   //jalr
   if(inst[6:0] == 7'b1100111 && inst[14:12] == 3'b000)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{20{inst[31]}},inst[31:20]};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 1;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 0;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1001;
   end
   //j&jal
   if(inst[6:0] == 7'b1101111)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{12{inst[31]}},inst[31],inst[19:12],inst[20],inst[30:21],{1'b0}};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 1;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 1;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b1000;
   end
   //beq
   if(inst[6:0] == 7'b1100011 && inst[14:12] == 3'b000)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],{1'b0}};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 1;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b0000;
   end
    //bne
   if(inst[6:0] == 7'b1100011 && inst[14:12] == 3'b001)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],{1'b0}};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 1;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b0001;
   end
   //blt
   if(inst[6:0] == 7'b1100011 && inst[14:12] == 3'b100)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],{1'b0}};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 1;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b0010;
   end
   //bge
   if(inst[6:0] == 7'b1100011 && inst[14:12] == 3'b101)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],{1'b0}};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 1;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b0100;
   end
   //bltu
   if(inst[6:0] == 7'b1100011 && inst[14:12] == 3'b110)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],{1'b0}};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 1;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b0011;
   end
   //bgeu
   if(inst[6:0] == 7'b1100011 && inst[14:12] == 3'b111)begin
       alu_op = 5'b00000;
       dmem_access_reg = 4'b1111;
       imm = {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],{1'b0}};
       rf_ra1_reg = inst[24:20];
       rf_ra0_reg = inst[19:15];
       rf_wa_reg = inst[11:7];
       rf_we_reg = 0;
       rf_wd_sel = 2'b00;
       alu_src0_sel_reg = 1;
       alu_src1_sel_reg = 1;
       br_type_reg = 4'b0110;
   end
    //ebreak
    if(inst == 32'h00100073)begin
        alu_op = 5'b11111;
        dmem_access_reg = 4'b1111;
        imm = 0;
        rf_ra1_reg = 0;
        rf_ra0_reg = 0;
        rf_wa_reg = 0;
        rf_we_reg = 0;
        rf_wd_sel = 2'b00;
        alu_src0_sel_reg = 0;
        alu_src1_sel_reg = 0;
        br_type_reg = 4'b1111;
    end
end

assign dmem_access = dmem_access_reg;
assign rf_ra0 = rf_ra0_reg;
assign rf_ra1 = rf_ra1_reg;
assign rf_wa = rf_wa_reg;
assign rf_we = rf_we_reg;
assign alu_src0_sel = alu_src0_sel_reg;
assign alu_src1_sel = alu_src1_sel_reg;
assign br_type = br_type_reg;

endmodule