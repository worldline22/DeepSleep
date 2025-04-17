module npc_mux(
input [31:0] pc_add4,
input [31:0] pc_offset,
input [31:0] pc_j,
input [1:0] npc_sel,

output [31:0] npc
);

reg [31:0] npc_reg;
always @(*) begin
    case (npc_sel)
        2'b00:npc_reg = pc_add4;
        2'b01:npc_reg = pc_offset;
        2'b10:npc_reg = pc_j;
        default: npc_reg = pc_add4;
    endcase
end

assign npc = npc_reg;

endmodule