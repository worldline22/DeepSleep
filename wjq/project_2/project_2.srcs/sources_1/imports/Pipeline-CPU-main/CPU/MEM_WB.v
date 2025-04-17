module MEM_WB(
    input [0:0]     clk,
    input [0:0]     en,
    input [0:0]     rst,
    input [31:0]    pcadd4_mem,
    input [31:0]    pc_mem,
    input [31:0]    inst_mem,
    input [31:0]    alu_res_mem,
    input [31:0]    dmem_rd_out_mem,
    input [4:0]     rf_wa_mem,
    input [0:0]     stall,
    input [0:0]     flush,
    input [0:0]     commit_mem,
    input [0:0]     rf_we_mem,
    input [1:0]     rf_wd_sel_mem,
    
    output  reg [31:0]      pcadd4_wb,
    output  reg [31:0]      alu_res_wb,
    output  reg [31:0]      dmem_rd_out_wb,
    output  reg [4:0]       rf_wa_wb,
    output  reg [0:0]       commit_wb,
    output  reg [0:0]       rf_we_wb,
    output  reg [1:0]       rf_wd_sel_wb,
    output  reg [31:0]      pc_wb,
    output  reg [31:0]      inst_wb
);

always @(posedge clk) begin
    if(rst)begin
        pcadd4_wb <= 32'h00400004;
        alu_res_wb <= 0;
        dmem_rd_out_wb <= 0;
        rf_wa_wb <= 0;
        commit_wb <= 0;
        rf_we_wb <= 0;
        rf_wd_sel_wb <= 0;
        pc_wb <= 32'h00400000;
        inst_wb <= 32'h00000013;
    end
    else if(en)begin
        if(flush)begin
            pcadd4_wb <= 32'h00400004;
            alu_res_wb <= 0;
            dmem_rd_out_wb <= 0;
            rf_wa_wb <= 0;
            commit_wb <= 0;
            rf_we_wb <= 0;
            rf_wd_sel_wb <= 0;
            pc_wb <= 32'h00400000;
            inst_wb <= 32'h00000013;
        end
        else if(stall)begin
            pcadd4_wb <= pcadd4_wb;
            alu_res_wb <= alu_res_wb;
            dmem_rd_out_wb <= dmem_rd_out_wb;
            rf_wa_wb <= rf_wa_wb;
            commit_wb <= commit_wb;
            rf_we_wb <= rf_we_wb;
            rf_wd_sel_wb <= rf_wd_sel_wb;
            pc_wb <= pc_wb;
            inst_wb <= inst_wb;
        end
        else begin
            pcadd4_wb <= pcadd4_mem;
            alu_res_wb <= alu_res_mem;
            dmem_rd_out_wb <= dmem_rd_out_mem;
            rf_wa_wb <= rf_wa_mem;
            commit_wb <= commit_mem;
            rf_we_wb <= rf_we_mem;
            rf_wd_sel_wb <= rf_wd_sel_mem;
            pc_wb <= pc_mem;
            inst_wb <= inst_mem;
        end
    end
end

endmodule