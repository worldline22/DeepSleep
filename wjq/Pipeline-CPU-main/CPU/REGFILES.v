module REG_FILE (
    input                   [ 0 : 0]        clk,

    input                   [ 4 : 0]        rf_ra0,
    input                   [ 4 : 0]        rf_ra1,   
    input                   [ 4 : 0]        rf_wa,
    input                   [ 0 : 0]        rf_we,
    input                   [31 : 0]        rf_wd,
    input                   [4 : 0]         debug_reg_ra,

    output                  [31:0]          debug_reg_rd,
    output                  [31 : 0]        rf_rd0,
    output                  [31 : 0]        rf_rd1
);

    reg [31 : 0] reg_file [0 : 31];

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            reg_file[i] = 0;
    end

    always @(posedge clk) begin
        if(rf_we)begin
            if(rf_wa !=0 )begin
                reg_file[rf_wa] <= rf_wd;
            end
        end
    end

    assign rf_rd0 = (rf_we && rf_wa != 0 && rf_wa == rf_ra0) ? rf_wd : reg_file[rf_ra0];
    assign rf_rd1 = (rf_we && rf_wa != 0 && rf_wa == rf_ra1) ? rf_wd : reg_file[rf_ra1];
    assign debug_reg_rd = reg_file[debug_reg_ra];
    
endmodule