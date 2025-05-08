module SLU (
    input                   [31 : 0]                addr,
    input                   [ 3 : 0]                dmem_access,

    input                   [31 : 0]                rd_in,
    input                   [31 : 0]                wd_in,

    output      reg         [31 : 0]                rd_out,
    output      reg         [31 : 0]                wd_out,
    output      reg         [0:0]                   wd_we,
    output      reg         [1:0]                   mask
);

`define LW      4'B0000
`define LH      4'B0001
`define LB      4'B0010
`define LHU     4'B0100
`define LBU     4'B0011
`define SW      4'B1000
`define SH      4'B1001
`define SB      4'B1011

always@(*) begin
    case(dmem_access)
        `LW: mask = 2'b10;
        `LH: mask = 2'b01;
        `LB: mask = 2'b00;
        `LHU: mask = 2'b01;
        `LBU: mask = 2'b00;
        `SW: mask = 2'b10;
        `SH: mask = 2'b01;
        `SB: mask = 2'b00;
    endcase
end


always @(*) begin
    case (dmem_access)
        `LW:begin
            rd_out = rd_in;
            wd_out = 0;
            wd_we = 0;
        end 
        `LH:begin
            if(addr % 4 == 0)begin
                rd_out = {{16{rd_in[15]}},rd_in[15:0]};
            end
            if(addr % 4 == 1)begin
                rd_out = {{16{rd_in[23]}},rd_in[23:8]};
            end
            if(addr % 4 == 2)begin
                rd_out = {{16{rd_in[31]}},rd_in[31:16]};
            end
            wd_out = 0;
            wd_we = 0;
        end
        `LB:begin
            if(addr % 4 == 0)begin
                rd_out = {{24{rd_in[7]}},rd_in[7:0]};
            end
            if(addr % 4 == 1)begin
                rd_out = {{24{rd_in[15]}},rd_in[15:8]};
            end
            if(addr % 4 == 2)begin
                rd_out = {{24{rd_in[23]}},rd_in[23:16]};
            end
            if(addr % 4 == 3)begin
                rd_out = {{24{rd_in[31]}},rd_in[31:24]};
            end
            wd_out = 0;
            wd_we = 0;
        end
        `LHU:begin
            if(addr % 4 == 0)begin
                rd_out = {{16{1'b0}},rd_in[15:0]};
            end
            if(addr % 4 == 1)begin
                rd_out = {{16{1'b0}},rd_in[23:8]};
            end
            if(addr % 4 == 2)begin
                rd_out = {{16{1'b0}},rd_in[31:16]};
            end
            wd_out = 0;
            wd_we = 0;
        end
        `LBU:begin
            if(addr % 4 == 0)begin
                rd_out = {{24{1'b0}},rd_in[7:0]};
            end
            if(addr % 4 == 1)begin
                rd_out = {{24{1'b0}},rd_in[15:8]};
            end
            if(addr % 4 == 2)begin
                rd_out = {{24{1'b0}},rd_in[23:16]};
            end
            if(addr % 4 == 3)begin
                rd_out = {{24{1'b0}},rd_in[31:24]};
            end
            wd_out = 0; 
            wd_we = 0;          
        end
        `SW:begin
            rd_out = 0;
            wd_out = wd_in;  
            wd_we = 1;
        end
        `SH:begin
            rd_out = 0;
            if (addr % 4 == 0) begin
                wd_out = {rd_in[31:16],wd_in[15:0]};    
            end
            if (addr % 4 == 1) begin
                wd_out = {rd_in[31:24],wd_in[15:0],rd_in[7:0]};    
            end
            if (addr % 4 == 2) begin
                wd_out = {wd_in[15:0],rd_in[15:0]};    
            end
            wd_we = 1;     
        end
        `SB:begin
            rd_out = 0;
            if (addr % 4 == 0) begin
                wd_out = {rd_in[31:8],wd_in[7:0]};    
            end
            if (addr % 4 == 1) begin
                wd_out = {rd_in[31:16],wd_in[7:0],rd_in[7:0]};   
            end
            if (addr % 4 == 2) begin
                wd_out = {rd_in[31:24],wd_in[7:0],rd_in[15:0]};    
            end
            if (addr % 4 == 3) begin
                wd_out = {wd_in[7:0],rd_in[23:0]};    
            end
            wd_we = 1;         
        end
        default: begin
            rd_out = 0;
            wd_out = 0;
            wd_we = 0;
        end
    endcase
end

endmodule