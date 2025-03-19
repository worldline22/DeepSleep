module instruction_fetch_stage (
    input  logic        clk,              // Clock signal
    input  logic        rst_n,            // Active-low reset
    
    // Program counter interface
    input  logic        stall,            // Stall signal from hazard detection
    input  logic        branch_taken,     // Branch taken signal, not use now 
    input  logic [31:0] branch_target,    // Branch target address, not use now
    
    // Outputs to decode stage
    output logic [31:0] instruction_id,   // Instruction to decode stage
    output logic [31:0] pc_id             // PC value to decode stage
);

    // PC to instruction memory
    assign imem_addr = pc_current;
    
    // PC module instantiation
    logic [31:0] pc_current;       // Current PC
    logic [31:0] pc_next;
    
    program_counter pc_unit (
        .clk(clk),
        .rst_n(rst_n),
        .stall(stall),
        .branch_taken(branch_taken),
        .branch_target(branch_target),
        .pc_current(pc_current),
        .pc_next(pc_next)
    );

    logic [31:0] imem_data;        // Instruction from memory

    instruction_memory instr_mem (
        .clk(clk),
        .rst_n(rst_n),
        .addr(pc_current),
        .instruction(imem_data)
    );
    
    // Pipeline registers for IF/ID stage
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset pipeline registers
            instruction_id <= 32'h0;  // NOP instruction
            pc_id <= 32'h0;
        end
        else if (!stall) begin
            // Update pipeline registers if not stalled
            instruction_id <= imem_data;
            pc_id <= pc_current;
        end
        // If stalled, maintain previous values (implicitly)
        
        // Handle control hazards (branch taken)
        if (branch_taken) begin
            // Insert NOP (bubble) on branch taken
            // need to write the bubble part
            instruction_id <= 32'h0;  // NOP instruction (addi x0, x0, 0)
        end
    end

endmodule



module instruction_decode_stage (
    input  logic        clk,              // Clock signal
    input  logic        rst_n,            // Active-low reset
    
    // Inputs from fetch stage, all input messages are from the fetch stage
    input  logic [31:0] instruction_id,   // Instruction from fetch stage
    input  logic [31:0] pc_id,            // PC from fetch stage
    
    // Hazard detection interface
    input  logic        stall,            // Stall signal from hazard detection
    
    // Register file interface
    output logic [4:0]  rs1_addr,         // Source register 1 address
    output logic [4:0]  rs2_addr,         // Source register 2 address
    output logic [4:0]  rd_addr,          // Destination register address
    
    // Immediate generation output
    output logic [31:0] imm_data,         // Immediate value
    

    // Control signals generated 
    output logic        reg_write,        // Register write enable
    output logic        mem_to_reg,       // Memory to register
    output logic        mem_write,        // Memory write enable
    output logic        mem_read,         // Memory read enable
    output logic [1:0]  alu_op,           // ALU operation type
    output logic        alu_src,          // ALU source (register/immediate)
    output logic [2:0]  funct3,           // Function code 3
    output logic        funct7_bit30,     // Function code 7, bit 30
    

//------------------------ Additions for branch/jump ------------------------    
    
    // For branch detection
    output logic        is_branch_id,     // Is this a branch instruction?
    output logic        is_jump_id,       // Is this a jump instruction?
    output logic [31:0] pc_id_out         // PC to execute stage (for branch/jump)
);

    // it can generate all the control signals and immediate values required for the pipeline stages.

    // Instruction fields
    logic [6:0] opcode;
    logic [6:0] funct7;
    
    // Extract fields from the instruction
    assign opcode = instruction_id[6:0];
    assign funct3 = instruction_id[14:12];
    assign funct7 = instruction_id[31:25];
    assign funct7_bit30 = instruction_id[30];  // Used to distinguish between ADD/SUB and SRL/SRA
    
    // Register addresses
    assign rs1_addr = instruction_id[19:15];
    assign rs2_addr = instruction_id[24:20];
    assign rd_addr = instruction_id[11:7];
    
    // Pass PC to next stage
    assign pc_id_out = pc_id;
    
    // Immediate generation
    always_comb begin
        // Default: I-type immediate
        // imm_data = {{20{instruction_id[31]}}, instruction_id[31:20]};
        
        case (opcode)
            7'b0000011: begin // Load instructions (I-type)
                imm_data = {{20{instruction_id[31]}}, instruction_id[31:20]};
            end
            
            7'b0010011: begin // I-type ALU operations
                imm_data = {{20{instruction_id[31]}}, instruction_id[31:20]};
            end
            
            7'b0100011: begin // Store instructions (S-type)
                imm_data = {{20{instruction_id[31]}}, instruction_id[31:25], instruction_id[11:7]};
            end
            
            7'b1100011: begin // Branch instructions (B-type)
                imm_data = {{20{instruction_id[31]}}, instruction_id[7], instruction_id[30:25], instruction_id[11:8], 1'b0};
            end
            
            7'b1101111: begin // JAL (J-type)
                imm_data = {{12{instruction_id[31]}}, instruction_id[19:12], instruction_id[20], instruction_id[30:21], 1'b0};
            end
            
            7'b1100111: begin // JALR (I-type)
                imm_data = {{20{instruction_id[31]}}, instruction_id[31:20]};
            end
            
            7'b0110111: begin // LUI (U-type)
                imm_data = {instruction_id[31:12], 12'b0};
            end
            
            7'b0010111: begin // AUIPC (U-type)
                imm_data = {instruction_id[31:12], 12'b0};
            end
            
            default: begin
                imm_data = 32'h0;
            end
        endcase
    end
    
    // Control unit - generates control signals based on opcode
    always_comb begin
        // Default values (NOP or unknown instruction)
        // reg_write = 1'b0;
        // mem_to_reg = 1'b0;
        // mem_write = 1'b0;
        // mem_read = 1'b0;
        // alu_op = 2'b00;
        // alu_src = 1'b0;
        // branch = 1'b0;
        // jump = 1'b0;
        // is_branch_id = 1'b0;
        // is_jump_id = 1'b0;
        
        case (opcode)
            7'b0110011: begin // R-type ALU operations
                reg_write = 1'b1;
                alu_op = 2'b10;
            end
            
            7'b0010011: begin // I-type ALU operations
                reg_write = 1'b1;
                alu_src = 1'b1;   // Use immediate
                alu_op = 2'b10;
            end
            
            7'b0000011: begin // Load instructions
                reg_write = 1'b1;
                mem_to_reg = 1'b1;
                mem_read = 1'b1;
                alu_src = 1'b1;   // Use immediate for address calculation
            end
            
            7'b0100011: begin // Store instructions
                mem_write = 1'b1;
                alu_src = 1'b1;   // Use immediate for address calculation
            end
            
            7'b1100011: begin // Branch instructions
                alu_op = 2'b01;  // Compare
                is_branch_id = 1'b1;
            end
            
            7'b1101111: begin // JAL
                reg_write = 1'b1;
                is_jump_id = 1'b1;
            end
            
            7'b1100111: begin // JALR
                reg_write = 1'b1;
                alu_src = 1'b1;   // Use immediate
                is_jump_id = 1'b1;
            end
            
            7'b0110111: begin // LUI
                reg_write = 1'b1;
                alu_src = 1'b1;   // Use immediate
                alu_op = 2'b11;   // Pass immediate directly
            end
            
            7'b0010111: begin // AUIPC
                reg_write = 1'b1;
                alu_src = 1'b1;   // Use immediate
                alu_op = 2'b11;   // Special operation for AUIPC
            end
            
            default: begin
                // Unknown or unimplemented instruction
                // All control signals are set to default values (do nothing)
            end
        endcase
    end

endmodule
