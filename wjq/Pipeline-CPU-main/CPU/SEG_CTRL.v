module SegCtrl (
    input [0:0]     rf_we_ex,
    input [1:0]     rf_wd_sel_ex,
    input [4:0]     rf_wa_ex,
    input [4:0]     rf_ra0_id,
    input [4:0]     rf_ra1_id,
    input [1:0]     npc_sel_ex,

    output reg [0:0]    stall_pc,
    output reg [0:0]    stall_if_id,
    output reg [0:0]    flush_if_id,
    output reg [0:0]    flush_id_ex
);
initial begin
    stall_pc = 0;
    stall_if_id = 0;
    flush_if_id = 0;
    flush_id_ex = 0;
end

always @(*) begin
    if(rf_wd_sel_ex == 2'b10 && rf_we_ex && rf_wa_ex != 0 && (rf_wa_ex == rf_ra0_id || rf_wa_ex == rf_ra1_id))begin
        stall_if_id = 1;
        flush_id_ex = 1;
        stall_pc = 1;
    end
    else if(npc_sel_ex == 2'b01 || npc_sel_ex == 2'b10)begin
        flush_id_ex = 1;
        flush_if_id = 1;
    end
    else begin
        stall_pc = 0;
        stall_if_id = 0;
        flush_if_id = 0;
        flush_id_ex = 0;
    end
end
endmodule