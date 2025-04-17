module ADD4(
    input   [31:0] pc,
    output   reg [31:0] npc
);
 
always @(*) begin
    npc = pc + 32'h4;
end
endmodule
