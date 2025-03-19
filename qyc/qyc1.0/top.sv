module cpu_top (
    input  logic        clk,
    input  logic        rst_n,

    // output logic [31:0] imem_addr,        // Instruction memory address
    // input  logic [31:0] imem_data,        // Instruction data from memory
    output logic [31:0] dmem_addr,        // Data memory address
    output logic [31:0] dmem_write_data,  // Data to write to memory
    input  logic [31:0] dmem_read_data,   // Data read from memory
    output logic        dmem_write,       // Data memory write enable
    output logic        dmem_read,        // Data memory read enable
    output logic [2:0]  dmem_funct3       // Function code for memory access
);

    // Hazard detection unit
    logic        stall_if;                // Stall signal for IF stage
    logic        stall_id;                // Stall signal for ID stage
    logic        flush_ex;                // Flush execute stage

    // IF stage
    logic [31:0] pc_current;
    logic [31:0] imem_addr;
    logic [31:0] instruction_id;
    logic [31:0] imem_data;
    logic [31:0] pc_id;

    // Branch signals
    logic        branch_taken;            // Branch is taken
    logic [31:0] branch_target;           // Branch target address

    // ID stage
    logic [4:0]  rs1_addr_id;
    logic [4:0]  rs2_addr_id;
    logic [4:0]  rd_addr_id;
    logic [31:0] imm_data_id;
    logic [31:0] rs1_data_id;             // Source register 1 data
    logic [31:0] rs2_data_id;             // Source register 2 data

    logic [1:0]  alu_op_id;
    logic        alu_src_id;    // determine (reg/imm)
    logic [2:0]  funct3_id;
    logic        funct7_bit30_id;

    // ID control signals
    logic        reg_write_id;            // Register write enable
    logic        mem_to_reg_id;           // Memory to register
    logic        mem_write_id;            // Memory write enable
    logic        mem_read_id;             // Memory read enable
    logic        is_branch_id;            // Branch instruction
    logic        is_jump_id;              // Jump instruction

    logic [31:0] pc_ex;                   // PC in execute stage

    // EX stage signals
    logic [4:0]  rs1_addr_ex;             // Source register 1 address
    logic [4:0]  rs2_addr_ex;             // Source register 2 address
    logic [4:0]  rd_addr_ex;              // Destination register address
    logic [31:0] rs1_data_ex;             // Source register 1 data
    logic [31:0] rs2_data_ex;             // Source register 2 data
    logic [31:0] imm_data_ex;             // Immediate value
    
    // Control signals in EX stage
    logic        reg_write_ex;            // Register write enable
    logic        mem_to_reg_ex;           // Memory to register
    logic        mem_write_ex;            // Memory write enable
    logic        mem_read_ex;             // Memory read enable
    logic [1:0]  alu_op_ex;               // ALU operation type
    logic        alu_src_ex;              // ALU source selection
    logic [2:0]  funct3_ex;               // Function code 3
    logic        funct7_bit30_ex;         // Function code 7 bit 30
    logic        is_branch_ex;            // Branch instruction
    logic        is_jump_ex;              // Jump instruction

    logic [1:0]  forward_a;               // Forwarding control for operand A
    logic [1:0]  forward_b;               // Forwarding control for operand B

    logic [31:0] alu_result_ex;           // ALU result
    logic        zero_flag_ex;            // Zero flag from ALU

    logic [31:0] alu_input_a;             // ALU input A (source 1)
    logic [31:0] alu_input_b;             // ALU input B (source 2 or immediate)
    logic [3:0]  alu_control;             // ALU control signal

    // MEM stage signals
    logic [4:0]  rd_addr_mem;             // Destination register address
    logic [31:0] alu_result_mem;          // ALU result
    logic [31:0] write_data_mem;          // Data to write to memory
    logic [31:0] read_data_mem;           // Data read from memory
    
    // Control signals in MEM stage
    logic        reg_write_mem;           // Register write enable
    logic        mem_to_reg_mem;          // Memory to register
    logic        mem_write_mem;           // Memory write enable
    logic        mem_read_mem;            // Memory read enable
    logic [2:0]  funct3_mem;              // Function code 3

    // WB stage signals
    logic [4:0]  rd_addr_wb;              // Destination register address
    logic [31:0] alu_result_wb;           // ALU result
    logic [31:0] read_data_wb;            // Data read from memory
    logic [31:0] write_back_data;         // Data to write back to register file
    
    // Control signals in WB stage
    logic        reg_write_wb;            // Register write enable
    logic        mem_to_reg_wb;           // Memory to register

    // Memory access signals
    logic        mem_busy;                // Memory is busy



    // Instruction Fetch Stage
    instruction_fetch_stage fetch_stage (
        .clk(clk),
        .rst_n(rst_n),
        .stall(stall_if),
        .branch_taken(branch_taken),    // judge whether we need to jump
        .branch_target(branch_target),  // target address of branch
        .instruction_id(instruction_id),
        .pc_id(pc_id)
    );

    // // ID stage
    // logic [4:0]  rs1_addr_id;
    // logic [4:0]  rs2_addr_id;
    // logic [4:0]  rd_addr_id;
    // logic [31:0] imm_data_id;
    // logic [31:0] rs1_data_id;             // Source register 1 data
    // logic [31:0] rs2_data_id;             // Source register 2 data

    // logic [1:0]  alu_op_id;
    // logic        alu_src_id;    // determine (reg/imm)
    // logic [2:0]  funct3_id;
    // logic        funct7_bit30_id;

    // // ID control signals
    // logic        reg_write_id;            // Register write enable
    // logic        mem_to_reg_id;           // Memory to register
    // logic        mem_write_id;            // Memory write enable
    // logic        mem_read_id;             // Memory read enable
    // logic        is_branch_id;            // Branch instruction
    // logic        is_jump_id;              // Jump instruction

    // logic [31:0] pc_ex;                   // PC in execute stage

    instruction_decode_stage decode_stage (
        // input
        .clk(clk),
        .rst_n(rst_n),
        .instruction_id(instruction_id),
        .pc_id(pc_id),
        .stall(stall_id),
        // output
        .rs1_addr(rs1_addr_id),
        .rs2_addr(rs2_addr_id),
        .rd_addr(rd_addr_id),
        .imm_data(imm_data_id),
        .reg_write(reg_write_id),
        // now we can know that whether the instruction if load/store or not
        .mem_to_reg(mem_to_reg_id),
        .mem_write(mem_write_id),
        .mem_read(mem_read_id),
        // judge what type of operation we need to do, and whether we need immediate value
        .alu_op(alu_op_id),
        .alu_src(alu_src_id),
        .funct3(funct3_id),
        .funct7_bit30(funct7_bit30_id),
        // judge whether we need to branch or jump
        .is_branch_id(is_branch_id),
        .is_jump_id(is_jump_id),
        // simply add a line
        .pc_id_out(pc_ex)
    );

    // // EX stage signals
    // logic [4:0]  rs1_addr_ex;             // Source register 1 address
    // logic [4:0]  rs2_addr_ex;             // Source register 2 address
    // logic [4:0]  rd_addr_ex;              // Destination register address
    // logic [31:0] rs1_data_ex;             // Source register 1 data
    // logic [31:0] rs2_data_ex;             // Source register 2 data
    // logic [31:0] imm_data_ex;             // Immediate value
    
    // // Control signals in EX stage
    // logic        reg_write_ex;            // Register write enable
    // logic        mem_to_reg_ex;           // Memory to register
    // logic        mem_write_ex;            // Memory write enable
    // logic        mem_read_ex;             // Memory read enable
    // logic [1:0]  alu_op_ex;               // ALU operation type
    // logic        alu_src_ex;              // ALU source selection
    // logic [2:0]  funct3_ex;               // Function code 3
    // logic        funct7_bit30_ex;         // Function code 7 bit 30
    // logic        is_branch_ex;            // Branch instruction
    // logic        is_jump_ex;              // Jump instruction

    register_file reg_file (
        .clk(clk),
        .rst_n(rst_n),
        .rs1_addr(rs1_addr_id),
        .rs2_addr(rs2_addr_id),
        .rd_addr(rd_addr_wb),   // write back
        .write_data(write_back_data),
        // judge whether we need to write back  
        .reg_write(reg_write_wb),
        // output
        .rs1_data(rs1_data_id),
        .rs2_data(rs2_data_id)
    );

    // pipeline registers for ID/EX stage
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all pipeline registers
            rs1_data_ex <= 32'h0;
            rs2_data_ex <= 32'h0;
            rs1_addr_ex <= 5'h0;
            rs2_addr_ex <= 5'h0;
            rd_addr_ex <= 5'h0;
            imm_data_ex <= 32'h0;
            
            // Control signals
            reg_write_ex <= 1'b0;
            mem_to_reg_ex <= 1'b0;
            mem_write_ex <= 1'b0;
            mem_read_ex <= 1'b0;
            alu_op_ex <= 2'b0;
            alu_src_ex <= 1'b0;
            funct3_ex <= 3'b0;
            funct7_bit30_ex <= 1'b0;
            is_branch_ex <= 1'b0;
            is_jump_ex <= 1'b0;
        end
        else if (!stall_id) begin
            // Update pipeline registers if not stalled
            rs1_data_ex <= rs1_data_id;
            rs2_data_ex <= rs2_data_id;
            rs1_addr_ex <= rs1_addr_id;
            rs2_addr_ex <= rs2_addr_id;
            rd_addr_ex <= rd_addr_id;
            imm_data_ex <= imm_data_id;
            
            // Control signals
            reg_write_ex <= reg_write_id;
            mem_to_reg_ex <= mem_to_reg_id;
            mem_write_ex <= mem_write_id;
            mem_read_ex <= mem_read_id;
            alu_op_ex <= alu_op_id;
            alu_src_ex <= alu_src_id;
            funct3_ex <= funct3_id;
            funct7_bit30_ex <= funct7_bit30_id;
            is_branch_ex <= is_branch_id;
            is_jump_ex <= is_jump_id;
        end
    end

    // Forwarding Unit
    // determine whether we need to forward the data (from ID/EX or directly from MEM/WB)
    // this is the control part of forwarding
    // Forwarding signals

    // logic [1:0]  forward_a;               // Forwarding control for operand A
    // logic [1:0]  forward_b;               // Forwarding control for operand B
    forwarding_unit forwarding (
        .rs1_read(1'b1),  // Always reading rs1 in this implementation
        .rs2_read(1'b1),  // Always reading rs2 in this implementation
        .rs1_addr_ex(rs1_addr_ex),
        .rs2_addr_ex(rs2_addr_ex),
        .rd_addr_mem(rd_addr_mem),
        .rd_addr_wb(rd_addr_wb),
        .reg_write_mem(reg_write_mem),
        .reg_write_wb(reg_write_wb),
        //output
        .forward_a(forward_a),
        .forward_b(forward_b)
    );

    // ALU Input Mux
    // use the signal from forwarding to determine the input of ALU
    // ================ ALU Signals ================
    // logic [31:0] alu_input_a;             // ALU input A (source 1)
    // logic [31:0] alu_input_b;             // ALU input B (source 2 or immediate)
    // logic [3:0]  alu_control;             // ALU control signal
    alu_input_mux alu_mux (
        .reg_data_a(rs1_data_ex),
        .reg_data_b(rs2_data_ex),
        .mem_result(alu_result_mem),
        .wb_result(write_back_data),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .alu_src(alu_src_ex),
        .imm_ex(imm_data_ex),
        // output
        .alu_input_a(alu_input_a),
        .alu_input_b(alu_input_b)
    );

    // ALU Control
    alu_control alu_ctrl (
        .alu_op_in(alu_op_ex),
        .funct3(funct3_ex),
        .funct7_bit30(funct7_bit30_ex),
        .alu_op_out(alu_control)
    );
    
    // ALU
    // logic [31:0] alu_result_ex;           // ALU result
    // logic        zero_flag_ex;            // Zero flag from ALU
    alu alu_unit (
        .a(alu_input_a),
        .b(alu_input_b),
        .alu_op(alu_control),
        // output
        .result(alu_result_ex),
        .zero(zero_flag_ex)
    );
    
    
    branch_unit branch_unit (
        .clk(clk),
        .rst_n(rst_n),
        .is_branch_ex(is_branch_ex),
        .is_jump_ex(is_jump_ex),
        .funct3_ex(funct3_ex),
        .rs1_data_ex(alu_input_a),  // Use forwarded value
        .rs2_data_ex(rs2_data_ex),  // Direct from register stage
        .pc_ex(pc_ex),
        .imm_ex(imm_data_ex),
        .alu_result_ex(alu_result_ex),
        // output. The data come to the fetch part
        .branch_taken(branch_taken),
        .branch_target(branch_target)
    );

    // // MEM stage signals
    // logic [4:0]  rd_addr_mem;             // Destination register address
    // logic [31:0] alu_result_mem;          // ALU result
    // logic [31:0] write_data_mem;          // Data to write to memory
    // logic [31:0] read_data_mem;           // Data read from memory
    
    // // Control signals in MEM stage
    // logic        reg_write_mem;           // Register write enable
    // logic        mem_to_reg_mem;          // Memory to register
    // logic        mem_write_mem;           // Memory write enable
    // logic        mem_read_mem;            // Memory read enable
    // logic [2:0]  funct3_mem;              // Function code 3

    // EX/MEM Pipeline Register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset EX/MEM pipeline register
            rd_addr_mem <= 5'h0;
            alu_result_mem <= 32'h0;
            write_data_mem <= 32'h0;
            reg_write_mem <= 1'b0;
            mem_to_reg_mem <= 1'b0;
            mem_write_mem <= 1'b0;
            mem_read_mem <= 1'b0;
            funct3_mem <= 3'h0;
        end
        else begin
            // Update EX/MEM pipeline register
            rd_addr_mem <= rd_addr_ex;
            alu_result_mem <= alu_result_ex;
            write_data_mem <= rs2_data_ex;  // For store instructions
            reg_write_mem <= reg_write_ex;
            mem_to_reg_mem <= mem_to_reg_ex;
            mem_write_mem <= mem_write_ex;
            mem_read_mem <= mem_read_ex;
            funct3_mem <= funct3_ex;
        end
    end

    // load_store_unit load_store_unit (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .addr(rd_addr_mem),
    //     .store_data(alu_result_ex),
    //     .mem_read(mem_read_mem),
    //     .mem_write(mem_write_mem),
    //     .funct3(funct3_mem),
    //     // output
    //     .alu_result(alu_result_mem),
    //     .dmem_addr(dmem_addr),
    //     .dmem_write_data(dmem_write_data),
    //     .dmem_write(dmem_write),
    //     .dmem_read(dmem_read),
    //     .dmem_funct3(dmem_funct3),
    //     .read_data(dmem_read_data)
    // );

    assign dmem_addr = alu_result_mem;
    assign dmem_write_data = write_data_mem;
    assign dmem_write = mem_write_mem;
    assign dmem_read = mem_read_mem;
    assign dmem_funct3 = funct3_mem;
    assign read_data_mem = dmem_read_data;


    
    // Data Memory Interface
    // output to memory
    // assign dmem_addr = alu_result_mem;
    // assign dmem_write_data = write_data_mem;
    // assign dmem_write = mem_write_mem;
    // assign dmem_read = mem_read_mem;
    // assign dmem_funct3 = funct3_mem;
    // // input from memory
    // assign read_data_mem = dmem_read_data;

    // // WB stage signals
    // logic [4:0]  rd_addr_wb;              // Destination register address
    // logic [31:0] alu_result_wb;           // ALU result
    // logic [31:0] read_data_wb;            // Data read from memory
    // logic [31:0] write_back_data;         // Data to write back to register file
    
    // // Control signals in WB stage
    // logic        reg_write_wb;            // Register write enable
    // logic        mem_to_reg_wb;           // Memory to register

    // // Memory access signals
    // logic        mem_busy;                // Memory is busy

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset MEM/WB pipeline register
            rd_addr_wb <= 5'h0;
            alu_result_wb <= 32'h0;
            read_data_wb <= 32'h0;
            reg_write_wb <= 1'b0;
            mem_to_reg_wb <= 1'b0;
        end
        else begin
            // Update MEM/WB pipeline register
            rd_addr_wb <= rd_addr_mem;
            alu_result_wb <= alu_result_mem;
            read_data_wb <= read_data_mem;
            reg_write_wb <= reg_write_mem;
            mem_to_reg_wb <= mem_to_reg_mem;
        end
    end

    // Write-Back Mux
    assign write_back_data = mem_to_reg_wb ? read_data_wb : alu_result_wb;

    // Hazard Detection Unit
    hazard_detection_unit hazard_unit (
        .mem_read_ex(mem_read_ex),
        .rd_addr_ex(rd_addr_ex),
        .rs1_addr_id(rs1_addr_id),
        .rs2_addr_id(rs2_addr_id),
        .branch_taken(branch_taken),
        .mem_busy(mem_busy),
        .stall_if(stall_if),
        .stall_id(stall_id),
        .flush_ex(flush_ex)
    );
    
    // Memory Hazard Detection Unit

endmodule