module IF_ID(
    input [0:0]     clk,
    input [0:0]     en,
    input [0:0]     rst,
    input [31:0]    pcadd4_if,
    input [31:0]    pc_if,
    input [31:0]    inst_if,
    input [0:0]     stall,
    input [0:0]     flush,
    input [0:0]     commit_if,
    
    output  reg [31:0]   pcadd4_id,
    output  reg [31:0]   pc_id,
    output  reg [31:0]   inst_id,
    output  reg [0:0]    commit_id
);

always @(posedge clk) begin
    if(rst)begin
        pcadd4_id <= 32'h00400004;
        pc_id <= 32'h00400000;
        inst_id <= 32'h00000013;
        commit_id <= 0;
    end
    else if(en)begin
        if(flush)begin
            pcadd4_id <= 32'h00400004;
            pc_id <= 32'h00400000;
            inst_id <= 32'h00000013;
            commit_id <= 0;
        end
        else if(stall)begin
            pcadd4_id <= pcadd4_id;
            pc_id <= pc_id;
            inst_id <= inst_id;
            commit_id <= commit_id;
        end
        else begin
            pcadd4_id <= pcadd4_if;
            pc_id <= pc_if;
            inst_id <= inst_if;
            commit_id <= commit_if;
        end
    end
end

endmodule