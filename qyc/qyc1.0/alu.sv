// alu is controlled by alu_control
module alu (
    input  logic [31:0] a,        // First operand
    input  logic [31:0] b,        // Second operand
    input  logic [3:0]  alu_op,   // ALU operation code
    output logic [31:0] result,   // Result of the operation
    output logic        zero      // Zero flag, set when result == 0
);
    /*
    Load指令（lw rd, offset(rs1)）:
    rs1 ───┐              offset ───┐
           ↓                        ↓
           ALU (ADD) ──→ 内存地址 ──→ 内存读取 ──→ rd

    Store指令（sw rs2, offset(rs1)）:
        rs1 ───┐              offset ───┐
            ↓                        ↓
            ALU (ADD) ──→ 内存地址 ──→ 内存写入 ←── rs2
    */

    // ALU operation codes (you may need to adjust based on your control unit implementation)
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_SLL  = 4'b0010; // Shift left logical
    localparam ALU_SLT  = 4'b0011; // Set less than (signed)
    localparam ALU_SLTU = 4'b0100; // Set less than unsigned
    localparam ALU_XOR  = 4'b0101;
    localparam ALU_SRL  = 4'b0110; // Shift right logical
    localparam ALU_SRA  = 4'b0111; // Shift right arithmetic
    localparam ALU_OR   = 4'b1000;
    localparam ALU_AND  = 4'b1001;

    // Temporary result for calculations
    logic [31:0] tmp_result;
    
    // Main ALU operation logic
    always_comb begin
        case (alu_op)
            ALU_ADD:  tmp_result = a + b;
            ALU_SUB:  tmp_result = a - b;
            ALU_SLL:  tmp_result = a << b[4:0];  // Only use bottom 5 bits for shift amount
            ALU_SLT:  tmp_result = $signed(a) < $signed(b) ? 32'b1 : 32'b0;
            ALU_SLTU: tmp_result = a < b ? 32'b1 : 32'b0;
            ALU_XOR:  tmp_result = a ^ b;
            ALU_SRL:  tmp_result = a >> b[4:0];  // Logical right shift
            ALU_SRA:  tmp_result = $signed(a) >>> b[4:0];  // Arithmetic right shift
            ALU_OR:   tmp_result = a | b;
            ALU_AND:  tmp_result = a & b;
            default:  tmp_result = 32'b0;  // Default case
        endcase
    end

    // Set the outputs
    assign result = tmp_result;
    assign zero = (tmp_result == 32'b0);

endmodule

// ALU Control Unit - Decodes the function codes from instruction to determine ALU operation
module alu_control (
    input  logic [1:0] alu_op_in,    // From main control unit (typically based on opcode)
    input  logic [2:0] funct3,       // funct3 field from instruction
    input  logic       funct7_bit30, // Bit 30 from funct7 field (used for SUB/SRA)
    output logic [3:0] alu_op_out    // To ALU
);

    // ALU_OP_IN encoding from control unit (example):
    // 00: load/store (add)
    // 01: branch (subtract for comparison)
    // 10: R-type or I-type ALU operations (depends on funct3/funct7)
    // 11: other operations

    always_comb begin
        case (alu_op_in)
            2'b00: alu_op_out = 4'b0000;  // ADD for load/store, the base address + offset
            
            2'b01: alu_op_out = 4'b0001;  // SUB for branches
            
            2'b10: begin  // R-type and I-type operations
                case (funct3)
                    3'b000: alu_op_out = funct7_bit30 ? 4'b0001 : 4'b0000;  // ADD/SUB
                    3'b001: alu_op_out = 4'b0010;  // SLL
                    3'b010: alu_op_out = 4'b0011;  // SLT
                    3'b011: alu_op_out = 4'b0100;  // SLTU
                    3'b100: alu_op_out = 4'b0101;  // XOR
                    3'b101: alu_op_out = funct7_bit30 ? 4'b0111 : 4'b0110;  // SRL/SRA
                    3'b110: alu_op_out = 4'b1000;  // OR
                    3'b111: alu_op_out = 4'b1001;  // AND
                    default: alu_op_out = 4'b0000;
                endcase
            end
            
            2'b11: alu_op_out = 4'b0000;  // Default to ADD
            
            default: alu_op_out = 4'b0000;
        endcase
    end

endmodule

// Forwarding Unit - Handles data hazards by detecting when forwarding is needed
module forwarding_unit (
    input  logic        rs1_read,             // Whether RS1 is being read
    input  logic        rs2_read,             // Whether RS2 is being read
    input  logic [4:0]  rs1_addr_ex,          // RS1 address in EX stage
    input  logic [4:0]  rs2_addr_ex,          // RS2 address in EX stage
    input  logic [4:0]  rd_addr_mem,          // Destination register address in MEM stage
    input  logic [4:0]  rd_addr_wb,           // Destination register address in WB stage
    input  logic        reg_write_mem,        // Register write signal in MEM stage
    input  logic        reg_write_wb,         // Register write signal in WB stage
    output logic [1:0]  forward_a,            // Forwarding control for operand A
    output logic [1:0]  forward_b             // Forwarding control for operand B
);

    // Forwarding control values:
    // 00: No forwarding, use the register file output
    // 01: Forward from MEM stage
    // 10: Forward from WB stage

    always_comb begin
        // Default: no forwarding
        
        // Check forwarding conditions for operand A (RS1)
        if (rs1_read && rs1_addr_ex != 5'b0) begin  // Only forward if RS1 is being read and not x0
            // Forward from MEM stage (higher priority)
            if (reg_write_mem && rd_addr_mem == rs1_addr_ex)
                forward_a = 2'b01;
            // Forward from WB stage (lower priority)
            else if (reg_write_wb && rd_addr_wb == rs1_addr_ex)
                forward_a = 2'b10;
        end
        
        // Check forwarding conditions for operand B (RS2)
        else if (rs2_read && rs2_addr_ex != 5'b0) begin  // Only forward if RS2 is being read and not x0
            // Forward from MEM stage (higher priority)
            if (reg_write_mem && rd_addr_mem == rs2_addr_ex)
                forward_b = 2'b01;
            // Forward from WB stage (lower priority)
            else if (reg_write_wb && rd_addr_wb == rs2_addr_ex)
                forward_b = 2'b10;
        end

        else begin
            forward_a = 2'b00;
            forward_b = 2'b00;
        end
    end

endmodule

// ALU Input Multiplexer - Selects the correct inputs for the ALU based on forwarding signals
module alu_input_mux (
    input  logic [31:0] reg_data_a,         // Register data from register file
    input  logic [31:0] reg_data_b,         // Register data from register file
    input  logic [31:0] mem_result,         // Result from MEM stage
    input  logic [31:0] wb_result,          // Result from WB stage
    input  logic [1:0]  forward_a,          // Forwarding control for operand A
    input  logic [1:0]  forward_b,          // Forwarding control for operand B
    input  logic        alu_src,            // Control signal to select immediate for operand B
    input  logic [31:0] imm_ex,             // Immediate value in EX stage
    output logic [31:0] alu_input_a,        // Final input A to ALU
    output logic [31:0] alu_input_b         // Final input B to ALU
);

    // Select input A based on forwarding control
    always_comb begin
        case (forward_a)
            2'b00: alu_input_a = reg_data_a;      // Use register value
            2'b01: alu_input_a = mem_result;      // Forward from MEM stage
            2'b10: alu_input_a = wb_result;       // Forward from WB stage
            default: alu_input_a = reg_data_a;    // Default case
        endcase
    end
    
    // Select base for input B based on forwarding control
    logic [31:0] forward_b_result;
    
    always_comb begin
        case (forward_b)
            2'b00: forward_b_result = reg_data_b;   // Use register value
            2'b01: forward_b_result = mem_result;   // Forward from MEM stage
            2'b10: forward_b_result = wb_result;    // Forward from WB stage
            default: forward_b_result = reg_data_b; // Default case
        endcase
    end
    
    // Final selection for input B (register or immediate)
    assign alu_input_b = alu_src ? imm_ex : forward_b_result;

endmodule

module branch_unit (
    input  logic        clk,              // Clock signal
    input  logic        rst_n,            // Active-low reset
    
    // Branch control from decode stage
    input  logic        is_branch_ex,     // Is this a branch instruction?
    input  logic        is_jump_ex,       // Is this a jump instruction?
    input  logic [2:0]  funct3_ex,        // Function code (branch type)
    
    // Operand values for branch condition evaluation
    input  logic [31:0] rs1_data_ex,      // Source register 1 value
    input  logic [31:0] rs2_data_ex,      // Source register 2 value
    
    // PC and immediate for target calculation
    input  logic [31:0] pc_ex,            // Current PC in EX stage
    input  logic [31:0] imm_ex,           // Immediate value (offset)
    
    // ALU result (for JALR instruction)
    input  logic [31:0] alu_result_ex,    // ALU result
    
    // Outputs
    output logic        branch_taken,     // Branch/jump is taken
    output logic [31:0] branch_target     // Target address
);

    // Branch condition evaluation
    logic branch_condition_met;
    
    always_comb begin
        // Default: condition not met
        branch_condition_met = 1'b0;
        
        if (is_branch_ex) begin
            case (funct3_ex)
                3'b000: branch_condition_met = (rs1_data_ex == rs2_data_ex);                  // BEQ
                3'b001: branch_condition_met = (rs1_data_ex != rs2_data_ex);                  // BNE
                3'b100: branch_condition_met = ($signed(rs1_data_ex) < $signed(rs2_data_ex)); // BLT
                3'b101: branch_condition_met = ($signed(rs1_data_ex) >= $signed(rs2_data_ex));// BGE
                3'b110: branch_condition_met = (rs1_data_ex < rs2_data_ex);                   // BLTU
                3'b111: branch_condition_met = (rs1_data_ex >= rs2_data_ex);                  // BGEU
                default: branch_condition_met = 1'b0;
            endcase
        end
    end
    
    // Branch/jump decision
    assign branch_taken = (is_branch_ex && branch_condition_met) || is_jump_ex;
    
    // Branch target calculation
    always_comb begin
        if (funct3_ex == 3'b001 && is_jump_ex) begin
            // JALR: target = rs1 + imm (from ALU)
            branch_target = {alu_result_ex[31:1], 1'b0};  // Clear LSB as per spec
        end
        else begin
            // JAL or conditional branches: target = PC + imm
            branch_target = pc_ex + imm_ex;
        end
    end

endmodule










