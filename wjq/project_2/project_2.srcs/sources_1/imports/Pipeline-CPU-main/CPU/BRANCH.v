module BRANCH(
    input                   [ 3 : 0]            br_type,

    input      signed       [31 : 0]            br_src0,
    input      signed       [31 : 0]            br_src1,

    output      reg         [ 1 : 0]            npc_sel
);

`define BEQ                4'B0000   
`define BNE                4'B0001   
`define BLT                4'B0010
`define BGE                4'B0100
`define BLTU               4'B0011
`define BGEU               4'B0110
`define JAL                4'B1000
`define JALR               4'B1001

always @(*) begin
    case (br_type)
        `BEQ: npc_sel = (br_src0 == br_src1) ? 2'b01 : 2'b00;
        `BNE: npc_sel = (br_src0 != br_src1) ? 2'b01 : 2'b00;
        `BLT: npc_sel = ((br_src0) < (br_src1)) ? 2'b01 : 2'b00;
        `BGE: npc_sel = (br_src0 >= br_src1) ? 2'b01 : 2'b00;
        `BLTU: npc_sel = ($signed(br_src0) < $signed(br_src1)) ? 2'b01 : 2'b00;
        `BGEU: npc_sel = ($signed(br_src0) >= $signed(br_src1)) ? 2'b01 : 2'b00;
        `JAL:npc_sel = 2'b01;
        `JALR:npc_sel = 2'b10;
        default: npc_sel = 2'b00;
    endcase
end

endmodule