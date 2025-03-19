module register_file (
    input  logic        clk,              // Clock signal
    input  logic        rst_n,            // Active-low reset
    // accept the input from the decode stage
    input  logic [4:0]  rs1_addr,         // Source register 1 address
    input  logic [4:0]  rs2_addr,         // Source register 2 address
    input  logic [4:0]  rd_addr,          // Destination register address
    input  logic [31:0] write_data,       // Data to be written to register, it maybe from the ALU calculation results or the memory
    input  logic        reg_write,        // Register write enable
    output logic [31:0] rs1_data,         // Data from source register 1
    output logic [31:0] rs2_data          // Data from source register 2
);

    // 32 registers of 32 bits each
    logic [31:0] registers [0:31];
    
    // RISC-V specifies that register x0 is hardwired to 0
    // This is handled in the read logic below
    
    // Reset logic
    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all registers to 0
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'h0;
            end
        end
        else if (reg_write && rd_addr != 5'b0) begin
            // Write to register if write is enabled and not writing to x0
            // x0 is hardwired to zero in RISC-V
            registers[rd_addr] <= write_data;
        end
    end
    
    // Read logic - combinational for immediate access within the cycle. The read operation don't need to wait for the next clock cycle
    // Special case for register x0 which is always 0
    assign rs1_data = (rs1_addr == 5'b0) ? 32'h0 : registers[rs1_addr];
    assign rs2_data = (rs2_addr == 5'b0) ? 32'h0 : registers[rs2_addr];

endmodule


// ======================== These part are additional ========================

// Hazard Detection Unit
module hazard_detection_unit (
    input  logic        mem_read_ex,         // Memory read flag in EX stage
    input  logic [4:0]  rd_addr_ex,          // Destination register in EX stage
    input  logic [4:0]  rs1_addr_id,         // Source register 1 in ID stage
    input  logic [4:0]  rs2_addr_id,         // Source register 2 in ID stage
    input  logic        branch_taken,        // Branch taken flag
    input  logic        mem_busy,            // Memory operation in progress
    output logic        stall_if,            // Stall fetch stage
    output logic        stall_id,            // Stall decode stage
    output logic        flush_ex             // Flush execute stage
);

    always_comb begin
        // Default values
        
        // Detect load-use hazard
        // If instruction in EX stage is a load, and its destination register
        // is used by the instruction in ID stage, we need to stall for one cycle
        if (mem_read_ex && 
            ((rd_addr_ex == rs1_addr_id) || (rd_addr_ex == rs2_addr_id)) && 
            (rd_addr_ex != 5'b0)) begin
            stall_if = 1'b1;
            stall_id = 1'b1;
            flush_ex = 1'b1;
        end
        
        // Stall if memory operation is in progress
        else if (mem_busy) begin
            stall_if = 1'b1;
            stall_id = 1'b1;
        end
        
        // Control hazard: branch taken
        else if (branch_taken) begin
            flush_ex = 1'b1;
        end

        else begin
            stall_if = 1'b0;
            stall_id = 1'b0;
            flush_ex = 1'b0;
        end

    end

endmodule

// Register Bypass Unit (to support register file forwarding)
module register_bypass (
    input  logic [4:0]  rs1_addr,            // Source register 1 address
    input  logic [4:0]  rs2_addr,            // Source register 2 address
    input  logic [4:0]  rd_addr_wb,          // Destination register in WB stage
    input  logic        reg_write_wb,        // Register write in WB stage
    input  logic [31:0] write_data_wb,       // Write data in WB stage
    input  logic [31:0] rs1_data_rf,         // Data from register file rs1
    input  logic [31:0] rs2_data_rf,         // Data from register file rs2
    output logic [31:0] rs1_data_bypassed,   // Potentially bypassed rs1 data
    output logic [31:0] rs2_data_bypassed    // Potentially bypassed rs2 data
);

    // Forwarding from WB stage directly to ID stage
    // This handles the case where a register is written and read in consecutive instructions
    always_comb begin
        // Default: use register file output
        
        // Forward from WB stage to rs1 if needed
        if (reg_write_wb && (rd_addr_wb != 5'b0) && (rd_addr_wb == rs1_addr)) begin
            rs1_data_bypassed = write_data_wb;
        end
        else begin
            rs1_data_bypassed = rs1_data_rf;
        end
        
        // Forward from WB stage to rs2 if needed
        if (reg_write_wb && (rd_addr_wb != 5'b0) && (rd_addr_wb == rs2_addr)) begin
            rs2_data_bypassed = write_data_wb;
        end
        else begin
            rs2_data_bypassed = rs2_data_rf;
        end
    end

endmodule