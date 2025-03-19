// Memory (MEM) Stage Module
module mem_stage (
    input  logic        clk,              // System clock
    input  logic        rst_n,            // Active-low reset
    
    // Inputs from EX stage
    input  logic [31:0] alu_result_ex,    // ALU result from EX stage
    input  logic [31:0] write_data_ex,    // Data to be written to memory
    input  logic [4:0]  rd_addr_ex,       // Destination register address
    
    // Control signals from EX stage
    input  logic        mem_read_ex,      // Memory read control signal
    input  logic        mem_write_ex,     // Memory write control signal
    input  logic        reg_write_ex,     // Register write control signal
    input  logic        mem_to_reg_ex,    // Memory to register control signal
    
    // Data memory interface
    output logic [31:0] mem_addr,         // Memory address
    output logic [31:0] mem_write_data,   // Data to write to memory
    output logic        mem_read,         // Memory read enable
    output logic        mem_write,        // Memory write enable
    input  logic [31:0] mem_read_data,    // Data read from memory
    
    // Outputs to WB stage
    output logic [31:0] alu_result_mem,   // ALU result to WB stage
    output logic [31:0] mem_data_mem,     // Memory data to WB stage
    output logic [4:0]  rd_addr_mem,      // Destination register address to WB stage
    output logic        reg_write_mem,    // Register write control to WB stage
    output logic        mem_to_reg_mem    // Memory to register control to WB stage
);

    // Connect memory interface signals
    assign mem_addr = alu_result_ex;          // ALU result is the memory address
    assign mem_write_data = write_data_ex;    // Forward write data to memory
    assign mem_read = mem_read_ex;            // Forward memory read control
    assign mem_write = mem_write_ex;          // Forward memory write control
    
    // Pipeline registers for MEM/WB stage
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all pipeline registers
            alu_result_mem <= 32'b0;
            mem_data_mem <= 32'b0;
            rd_addr_mem <= 5'b0;
            reg_write_mem <= 1'b0;
            mem_to_reg_mem <= 1'b0;
        end else begin
            // Update pipeline registers
            alu_result_mem <= alu_result_ex;
            mem_data_mem <= mem_read_data;
            rd_addr_mem <= rd_addr_ex;
            reg_write_mem <= reg_write_ex;
            mem_to_reg_mem <= mem_to_reg_ex;
        end
    end

endmodule

// Write Back (WB) Stage Module
module wb_stage (
    // Inputs from MEM stage
    input  logic [31:0] alu_result_mem,   // ALU result from MEM stage
    input  logic [31:0] mem_data_mem,     // Memory data from MEM stage
    input  logic [4:0]  rd_addr_mem,      // Destination register address
    input  logic        reg_write_mem,    // Register write control signal
    input  logic        mem_to_reg_mem,   // Memory to register control signal
    
    // Outputs to register file and forwarding unit
    output logic [31:0] wb_data,          // Data to write back to register file
    output logic [4:0]  wb_addr,          // Destination register address
    output logic        reg_write_wb      // Register write control signal
);

    // Select write-back data based on mem_to_reg control signal
    assign wb_data = mem_to_reg_mem ? mem_data_mem : alu_result_mem;
    
    // Forward register address and write control
    assign wb_addr = rd_addr_mem;
    assign reg_write_wb = reg_write_mem;

endmodule

// Data Memory Module
module data_memory (
    input  logic        clk,              // System clock
    input  logic        rst_n,            // Active-low reset
    input  logic [31:0] addr,             // Memory address
    input  logic [31:0] write_data,       // Data to write
    input  logic        read_en,          // Read enable
    input  logic        write_en,         // Write enable
    output logic [31:0] read_data         // Data read from memory
);
    
    // Memory array (4KB memory for example)
    logic [31:0] memory [0:1023];  // 1024 words of 32 bits (4KB)
    
    // Initialize memory with zeros on reset
    always_ff @(negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 1024; i++) begin
                memory[i] <= 32'b0;
            end
        end
    end
    
    // Memory read operation (asynchronous read)
    always_comb begin
        if (read_en) begin
            // Check if address is in range
            if (addr[31:2] < 1024) begin
                read_data = memory[addr[11:2]];  // Word-aligned access
            end else begin
                read_data = 32'b0;  // Out of range returns zero
            end
        end else begin
            read_data = 32'b0;
        end
    end
    
    // Memory write operation (synchronous write)
    always_ff @(posedge clk) begin
        if (write_en) begin
            // Check if address is in range
            if (addr[31:2] < 1024) begin
                memory[addr[11:2]] <= write_data;  // Word-aligned access
            end
        end
    end

endmodule

// Load Store Unit - Handles memory access alignment and byte enables
module load_store_unit (
    input  logic [31:0] addr,             // Memory address
    input  logic [31:0] store_data,       // Data to store (from register)
    input  logic [2:0]  funct3,           // Function code from instruction
    input  logic        mem_read,         // Memory read signal
    input  logic        mem_write,        // Memory write signal
    
    // Memory interface
    output logic [31:0] mem_addr,         // Address to memory
    output logic [31:0] mem_write_data,   // Data to write to memory
    output logic [3:0]  mem_byte_enable,  // Byte enable signals
    output logic        mem_read_en,      // Memory read enable
    output logic        mem_write_en,     // Memory write enable
    
    // Load result processing
    input  logic [31:0] mem_read_data,    // Raw data from memory
    output logic [31:0] load_data         // Processed load data (to register)
);

    // Memory access types based on funct3
    localparam LB  = 3'b000;  // Load Byte
    localparam LH  = 3'b001;  // Load Halfword
    localparam LW  = 3'b010;  // Load Word
    localparam LBU = 3'b100;  // Load Byte Unsigned
    localparam LHU = 3'b101;  // Load Halfword Unsigned
    localparam SB  = 3'b000;  // Store Byte
    localparam SH  = 3'b001;  // Store Halfword
    localparam SW  = 3'b010;  // Store Word
    
    // Byte position based on address
    logic [1:0] byte_offset;
    assign byte_offset = addr[1:0];
    
    // Pass through the address and enable signals
    assign mem_addr = {addr[31:2], 2'b00};  // Word-aligned address
    assign mem_read_en = mem_read;
    assign mem_write_en = mem_write;
    
    // Generate byte enable signals for writes
    always_comb begin
        // Default: no bytes enabled
        mem_byte_enable = 4'b0000;
        
        if (mem_write) begin
            case (funct3)
                SB: begin  // Store byte
                    case (byte_offset)
                        2'b00: mem_byte_enable = 4'b0001;
                        2'b01: mem_byte_enable = 4'b0010;
                        2'b10: mem_byte_enable = 4'b0100;
                        2'b11: mem_byte_enable = 4'b1000;
                    endcase
                end
                
                SH: begin  // Store halfword
                    case (byte_offset)
                        2'b00: mem_byte_enable = 4'b0011;
                        2'b10: mem_byte_enable = 4'b1100;
                        default: mem_byte_enable = 4'b0000;  // Misaligned access
                    endcase
                end
                
                SW: begin  // Store word
                    if (byte_offset == 2'b00)
                        mem_byte_enable = 4'b1111;
                    else
                        mem_byte_enable = 4'b0000;  // Misaligned access
                end
                
                default: mem_byte_enable = 4'b0000;
            endcase
        end
    end
    
    // Generate write data with proper byte positioning
    always_comb begin
        // Default value
        mem_write_data = 32'b0;
        
        if (mem_write) begin
            case (funct3)
                SB: begin  // Store byte
                    case (byte_offset)
                        2'b00: mem_write_data = {24'b0, store_data[7:0]};
                        2'b01: mem_write_data = {16'b0, store_data[7:0], 8'b0};
                        2'b10: mem_write_data = {8'b0, store_data[7:0], 16'b0};
                        2'b11: mem_write_data = {store_data[7:0], 24'b0};
                    endcase
                end
                
                SH: begin  // Store halfword
                    case (byte_offset)
                        2'b00: mem_write_data = {16'b0, store_data[15:0]};
                        2'b10: mem_write_data = {store_data[15:0], 16'b0};
                        default: mem_write_data = 32'b0;  // Misaligned access
                    endcase
                end
                
                SW: mem_write_data = store_data;  // Store word (full 32 bits)
                
                default: mem_write_data = 32'b0;
            endcase
        end
    end
    
    // Process load data based on type
    always_comb begin
        // Default value
        load_data = 32'b0;
        
        if (mem_read) begin
            case (funct3)
                LB: begin  // Load byte (sign-extended)
                    case (byte_offset)
                        2'b00: load_data = {{24{mem_read_data[7]}}, mem_read_data[7:0]};
                        2'b01: load_data = {{24{mem_read_data[15]}}, mem_read_data[15:8]};
                        2'b10: load_data = {{24{mem_read_data[23]}}, mem_read_data[23:16]};
                        2'b11: load_data = {{24{mem_read_data[31]}}, mem_read_data[31:24]};
                    endcase
                end
                
                LH: begin  // Load halfword (sign-extended)
                    case (byte_offset)
                        2'b00: load_data = {{16{mem_read_data[15]}}, mem_read_data[15:0]};
                        2'b10: load_data = {{16{mem_read_data[31]}}, mem_read_data[31:16]};
                        default: load_data = 32'b0;  // Misaligned access
                    endcase
                end
                
                LW: load_data = mem_read_data;  // Load word (full 32 bits)
                
                LBU: begin  // Load byte unsigned
                    case (byte_offset)
                        2'b00: load_data = {24'b0, mem_read_data[7:0]};
                        2'b01: load_data = {24'b0, mem_read_data[15:8]};
                        2'b10: load_data = {24'b0, mem_read_data[23:16]};
                        2'b11: load_data = {24'b0, mem_read_data[31:24]};
                    endcase
                end
                
                LHU: begin  // Load halfword unsigned
                    case (byte_offset)
                        2'b00: load_data = {16'b0, mem_read_data[15:0]};
                        2'b10: load_data = {16'b0, mem_read_data[31:16]};
                        default: load_data = 32'b0;  // Misaligned access
                    endcase
                end
                
                default: load_data = 32'b0;
            endcase
        end
    end

endmodule

// Memory Access Hazard Detection Unit
module mem_hazard_unit (
    input  logic [4:0]  rs1_addr_id,      // RS1 address in ID stage
    input  logic [4:0]  rs2_addr_id,      // RS2 address in ID stage
    input  logic [4:0]  rd_addr_ex,       // Destination register in EX stage
    input  logic        mem_read_ex,      // Memory read signal in EX stage
    output logic        stall_pipeline    // Stall pipeline control signal
);

    // Detect load-use hazard (when a load instruction is followed by an instruction that uses the loaded value)
    always_comb begin
        // Stall if the instruction in EX stage is a load and its destination register
        // is used by the instruction in ID stage
        stall_pipeline = mem_read_ex && ((rd_addr_ex == rs1_addr_id) || (rd_addr_ex == rs2_addr_id)) && (rd_addr_ex != 5'b0);
    end

endmodule

// Pipeline Control Unit - Handles stalls and flushes
module pipeline_control (
    input  logic        clk,              // System clock
    input  logic        rst_n,            // Active-low reset
    input  logic        stall_pipeline,   // Stall signal from hazard detection unit
    input  logic        branch_taken,     // Branch taken signal
    output logic        if_id_write,      // IF/ID register write enable
    output logic        pc_write,         // PC write enable
    output logic        clear_id_ex,      // Clear ID/EX pipeline registers
    output logic        clear_ex_mem,     // Clear EX/MEM pipeline registers
    output logic        clear_mem_wb      // Clear MEM/WB pipeline registers
);

    // Stall control
    always_comb begin
        // Default: normal operation
        if_id_write = 1'b1;
        pc_write = 1'b1;
        
        // When a stall is needed
        if (stall_pipeline) begin
            if_id_write = 1'b0;  // Freeze IF/ID register
            pc_write = 1'b0;     // Freeze PC
        end
    end
    
    // Flush control
    always_comb begin
        // Default: no flush
        clear_id_ex = 1'b0;
        clear_ex_mem = 1'b0;
        clear_mem_wb = 1'b0;
        
        // When a branch is taken
        if (branch_taken) begin
            clear_id_ex = 1'b1;  // Flush ID/EX pipeline registers (branch shadow)
        end
    end

endmodule