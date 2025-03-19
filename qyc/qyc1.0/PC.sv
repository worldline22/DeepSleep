
// we need to update PC current, however, in this part we don't send out pc_current, so we need to update the pc_current in the IF stage
module program_counter (
    input  logic        clk,            // Clock signal
    input  logic        rst_n,          // Active-low reset
    input  logic        stall,          // Stall signal from hazard detection
    input  logic        branch_taken,   // Branch taken signal from branch unit
    input  logic [31:0] branch_target,  // Target address for branch/jump
    output logic [31:0] pc_current,     // Current PC value
    output logic [31:0] pc_next         // Next PC value (PC+4 or branch target)
);

    // PC register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset PC to beginning of program memory (address 0)
            pc_current <= 32'h0;
        end
        else if (!stall) begin
            // Update PC if not stalled
            pc_current <= pc_next;
        end
        // If stalled, maintain current PC value (implicitly)
    end

    // always_ff @(negedge clk) begin
    //     pc_next = branch_taken ? branch_target : (pc_current + 4);
    // end
    
    // Next PC logic - choose between PC+4 (sequential) or branch target
    assign pc_next = branch_taken ? branch_target : (pc_current + 4);

    // Temporarily, we ignore the relevant operation of branch prediction

endmodule

module instruction_memory (
    input  logic        clk,            // Clock signal
    input  logic        rst_n,          // Active-low reset
    input  logic [31:0] addr,           // Memory address (PC)
    output logic [31:0] instruction     // Fetched instruction
);

    // Memory size parameters (modify as needed)
    localparam MEM_SIZE = 4096;         // Instruction memory size in bytes
    localparam ADDR_WIDTH = $clog2(MEM_SIZE);
    
    // Instruction memory array (byte addressable)
    logic [7:0] mem [0:MEM_SIZE-1];
    
    // Read instruction
    initial begin
        $readmemh("instruction.hex", mem);
    end
    // Combine 4 bytes to form a 32-bit instruction (little-endian byte order)
    assign instruction = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};

endmodule

