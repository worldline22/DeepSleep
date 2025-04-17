module ID_EX(
    input [0:0]     clk,
    input [0:0]     en,
    input [0:0]     rst,
    input [31:0]    pcadd4_id,
    input [31:0]    pc_id,
    input [31:0]    inst_id,
    input [31:0]    rf_rd0_id,
    input [31:0]    rf_rd1_id,
    input [4:0]     rf_ra0_id,
    input [4:0]     rf_ra1_id,
    input [31:0]    imm_id,
    input [4:0]     rf_wa_id,
    input [0:0]     rf_we_id,
    input [0:0]     stall,
    input [0:0]     flush,
    input [0:0]     commit_id,
    input [4:0]     alu_op_id,
    input [3:0]     dmem_access_id,
    input [3:0]     br_type_id,
    input [1:0]     rf_wd_sel_id,
    input [0:0]     alu_src0_sel_id,
    input [0:0]     alu_src1_sel_id,
    
    output  reg [31:0]      pcadd4_ex,
    output  reg [31:0]      pc_ex,
    output  reg [31:0]      inst_ex,
    output  reg [31:0]      rf_rd0_ex,
    output  reg [31:0]      rf_rd1_ex,
    output  reg [4:0]       rf_ra0_ex,
    output  reg [4:0]       rf_ra1_ex,
    output  reg [31:0]      imm_ex,
    output  reg [4:0]       rf_wa_ex,
    output  reg [0:0]       commit_ex,
    output  reg [0:0]       rf_we_ex,
    output  reg [4:0]       alu_op_ex,
    output  reg [3:0]       dmem_access_ex,
    output  reg [3:0]       br_type_ex,
    output  reg [1:0]       rf_wd_sel_ex,
    output  reg [0:0]       alu_src0_sel_ex,
    output  reg [0:0]       alu_src1_sel_ex
);

always @(posedge clk) begin
    if(rst)begin
        pcadd4_ex <= 32'h00400004;
        pc_ex <= 32'h00400000;
        inst_ex <= 32'h00000013;
        rf_rd0_ex <= 0;
        rf_rd1_ex <= 0;
        rf_ra0_ex <= 0;
        rf_ra1_ex <= 0;
        imm_ex <= 0;
        rf_wa_ex <= 0;
        commit_ex <= 0;
        rf_we_ex <= 0;
        alu_op_ex <= 0;
        dmem_access_ex <= 4'b1111;
        br_type_ex <= 4'b1111;
        rf_wd_sel_ex <= 0;
        alu_src0_sel_ex <= 0;
        alu_src1_sel_ex <= 0;
    end
    else if(en)begin
        if(flush)begin
            pcadd4_ex <= 32'h00400004;
            pc_ex <= 32'h00400000;
            inst_ex <= 32'h00000013;
            rf_rd0_ex <= 0;
            rf_rd1_ex <= 0;
            rf_ra0_ex <= 0;
            rf_ra1_ex <= 0;
            imm_ex <= 0;
            rf_wa_ex <= 0;
            commit_ex <= 0;
            rf_we_ex <= 0;
            alu_op_ex <= 0;
            dmem_access_ex <= 4'b1111;
            br_type_ex <= 4'b1111;
            rf_wd_sel_ex <= 0;
            alu_src0_sel_ex <= 0;
            alu_src1_sel_ex <= 0;
        end
        else if(stall)begin
            pcadd4_ex <= pcadd4_ex;
            pc_ex <= pc_ex;
            inst_ex <= inst_ex;
            rf_rd0_ex <= rf_rd0_ex;
            rf_rd1_ex <= rf_rd1_ex;
            rf_ra0_ex <= rf_ra0_ex;
            rf_ra1_ex <= rf_ra1_ex;
            imm_ex <= imm_ex;
            rf_wa_ex <= rf_wa_ex;
            commit_ex <= commit_ex;
            rf_we_ex <= rf_we_ex;
            alu_op_ex <= alu_op_ex;
            dmem_access_ex <= dmem_access_ex;
            br_type_ex <= br_type_ex;
            rf_wd_sel_ex <= rf_wd_sel_ex;
            alu_src0_sel_ex <= alu_src0_sel_ex;
            alu_src1_sel_ex <= alu_src1_sel_ex;
        end
        else begin
            pcadd4_ex <= pcadd4_id;
            pc_ex <= pc_id;
            inst_ex <= inst_id;
            rf_rd0_ex <= rf_rd0_id;
            rf_rd1_ex <= rf_rd1_id;
            rf_ra0_ex <= rf_ra0_id;
            rf_ra1_ex <= rf_ra1_id;
            imm_ex <= imm_id;
            rf_wa_ex <= rf_wa_id;
            commit_ex <= commit_id;
            rf_we_ex <= rf_we_id;
            alu_op_ex <= alu_op_id;
            dmem_access_ex <= dmem_access_id;
            br_type_ex <= br_type_id;
            rf_wd_sel_ex <= rf_wd_sel_id;
            alu_src0_sel_ex <= alu_src0_sel_id;
            alu_src1_sel_ex <= alu_src1_sel_id;
        end
    end
end

endmodule