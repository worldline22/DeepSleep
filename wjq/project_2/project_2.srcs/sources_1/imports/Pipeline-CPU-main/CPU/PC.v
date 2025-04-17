module PC (
    input                   [ 0 : 0]            clk,
    input                   [ 0 : 0]            rst,
    input                   [ 0 : 0]            en,
    input                   [ 0 : 0]            stall_pc,
    input                   [31 : 0]            npc,

    output      reg         [31 : 0]            pc
);
initial begin
    pc <= 32'h00400000;
end
always @(posedge clk) begin
    if(rst) begin
        pc <= 32'h00400000;
    end
    else begin
        if (stall_pc) begin
            pc <= pc;
        end
        else begin
            if(en) begin
                pc <= npc;
            end
        end
    end
end

endmodule