module EX_MEM(
    input [0:0]     clk,
    input [0:0]     en,
    input [0:0]     rst,
    input [31:0]    pcadd4_ex,
    input [31:0]    pc_ex,
    input [31:0]    inst_ex,
    input [31:0]    alu_res_ex,
    input [31:0]    rf_rd1_ex,
    input [4:0]     rf_wa_ex,
    input [0:0]     stall,
    input [0:0]     flush,
    input [0:0]     commit_ex,
    input [0:0]     rf_we_ex,
    input [1:0]     rf_wd_sel_ex,
    input [3:0]     dmem_access_ex,
    
    output  reg [31:0]      pcadd4_mem,
    output  reg [31:0]      alu_res_mem,
    output  reg [31:0]      rf_rd1_mem,
    output  reg [4:0]       rf_wa_mem,
    output  reg [0:0]       commit_mem,
    output  reg [0:0]       rf_we_mem,
    output  reg [1:0]       rf_wd_sel_mem,
    output  reg [3:0]       dmem_access_mem,
    output  reg [31:0]      pc_mem,
    output  reg [31:0]      inst_mem
);

always @(posedge clk) begin
    if(rst)begin
        pcadd4_mem <= 32'h00400004;
        alu_res_mem <= 0;
        rf_rd1_mem <= 0;
        rf_wa_mem <= 0;
        commit_mem <= 0;
        rf_we_mem <= 0;
        rf_wd_sel_mem <= 0;
        dmem_access_mem <= 4'b1111;
        pc_mem <= 32'h00400000;
        inst_mem <= 32'h00000013;
    end
    else if(en)begin
        if(flush)begin
            pcadd4_mem <= 32'h00400004;
            alu_res_mem <= 0;
            rf_rd1_mem <= 0;
            rf_wa_mem <= 0;
            commit_mem <= 0;
            rf_we_mem <= 0;
            rf_wd_sel_mem <= 0;
            dmem_access_mem <= 4'b1111;
            pc_mem <= 32'h00400000;
            inst_mem <= 32'h00000013;
        end
        else if(stall)begin
            pcadd4_mem <= pcadd4_mem;
            alu_res_mem <= alu_res_mem;
            rf_rd1_mem <= rf_rd1_mem;
            rf_wa_mem <= rf_wa_mem;
            commit_mem <= commit_mem;
            rf_we_mem <= rf_we_mem;
            rf_wd_sel_mem <= rf_wd_sel_mem;
            dmem_access_mem <= dmem_access_mem;
            pc_mem <= pc_mem;
            inst_mem <= inst_mem;
        end
        else begin
            pcadd4_mem <= pcadd4_ex;
            alu_res_mem <= alu_res_ex;
            rf_rd1_mem <= rf_rd1_ex;
            rf_wa_mem <= rf_wa_ex;
            commit_mem <= commit_ex;
            rf_we_mem <= rf_we_ex;
            rf_wd_sel_mem <= rf_wd_sel_ex;
            dmem_access_mem <= dmem_access_ex;
            pc_mem <= pc_ex;
            inst_mem <= inst_ex;
        end
    end
end

endmodule