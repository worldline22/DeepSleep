module data_memory (
    input  logic        clk,              // Clock signal
    input  logic        rst_n,            // Active-low reset
    input  logic [31:0] addr,             // Memory address for load/store
    input  logic [31:0] write_data,       // Data to be written to memory
    input  logic        mem_write,        // Memory write enable
    input  logic        mem_read,         // Memory read enable
    input  logic [2:0]  funct3,           // Function code to determine operation type (LB, LH, LW, etc.)
    output logic [31:0] read_data,        // Data read from memory
    output logic        mem_ready         // Memory operation complete signal
);

    // Memory size parameters (modify as needed)
    localparam MEM_SIZE = 4096;  // Memory size in bytes
    localparam ADDR_WIDTH = $clog2(MEM_SIZE);
    
    // Memory array (byte-addressable)
    logic [7:0] mem [0:MEM_SIZE-1];
    
    // Define memory access type based on funct3
    localparam BYTE  = 3'b000;  // LB/SB
    localparam HALF  = 3'b001;  // LH/SH
    localparam WORD  = 3'b010;  // LW/SW
    localparam BYTE_U = 3'b100; // LBU
    localparam HALF_U = 3'b101; // LHU
    
    // Address alignment signals
    logic [31:0] aligned_addr;
    assign aligned_addr = {addr[31:2], 2'b00};  // Word-aligned address
    
    // Memory access logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset memory when needed, typically initializes to 0
            // Or can be left uninitialized in simulation
            for (int i = 0; i < MEM_SIZE; i = i + 1) begin
                mem[i] <= 8'h00;
            end
            mem_ready <= 1'b0;
        end
        else begin
            
            // Handle memory write operations
            if (mem_write) begin
                case (funct3)
                    BYTE: begin  // SB
                        // Store a byte at the specified address
                        mem[addr] <= write_data[7:0];
                        mem_ready <= 1'b1;
                    end
                    
                    HALF: begin  // SH
                        // Store a half-word (2 bytes) at the specified address
                        if (addr[0] == 1'b0) begin  // Check half-word alignment
                            mem[addr]   <= write_data[7:0];
                            mem[addr+1] <= write_data[15:8];
                            mem_ready <= 1'b1;
                        end
                        else begin
                            // Misaligned access - can handle with exception or padding
                            // For simplicity, we'll just indicate not ready
                            mem_ready <= 1'b0;
                        end
                    end
                    
                    WORD: begin  // SW
                        // Store a word (4 bytes) at the specified address
                        if (addr[1:0] == 2'b00) begin  // Check word alignment
                            mem[addr]   <= write_data[7:0];
                            mem[addr+1] <= write_data[15:8];
                            mem[addr+2] <= write_data[23:16];
                            mem[addr+3] <= write_data[31:24];
                            mem_ready <= 1'b1;
                        end
                        else begin
                            // Misaligned access - can handle with exception or padding
                            // For simplicity, we'll just indicate not ready
                            mem_ready <= 1'b0;
                        end
                    end
                    
                    default: begin
                        // Invalid operation
                        mem_ready <= 1'b0;
                    end
                endcase
            end
        end
    end
    
    // Memory read logic (combinational for immediate response within the cycle)
    always @* begin
        
        if (mem_read) begin
            case (funct3)
                BYTE: begin  // LB (sign-extended byte)
                    read_data = {{24{mem[addr][7]}}, mem[addr]};
                end
                
                BYTE_U: begin  // LBU (zero-extended byte)
                    read_data = {24'b0, mem[addr]};
                end
                
                HALF: begin  // LH (sign-extended half-word)
                    if (addr[0] == 1'b0) begin  // Check half-word alignment
                        read_data = {{16{mem[addr+1][7]}}, mem[addr+1], mem[addr]};
                    end
                end
                
                HALF_U: begin  // LHU (zero-extended half-word)
                    if (addr[0] == 1'b0) begin  // Check half-word alignment
                        read_data = {16'b0, mem[addr+1], mem[addr]};
                    end
                end
                
                WORD: begin  // LW
                    if (addr[1:0] == 2'b00) begin  // Check word alignment
                        read_data = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
                    end
                end
                
                default: begin
                    read_data = 32'h0;  // Invalid operation
                end
            endcase
        end
    end

endmodule

// Load/Store Unit - Handles memory access operations for the pipeline
module load_store_unit (
    input  logic        clk,              // Clock signal
    input  logic        rst_n,            // Active-low reset
    input  logic [31:0] addr,             // Memory address calculated by ALU
    input  logic [31:0] store_data,       // Data to store (from register)
    input  logic        mem_write,        // Memory write control signal
    input  logic        mem_read,         // Memory read control signal
    input  logic [2:0]  funct3,           // Function code for load/store type
    output logic [31:0] load_data,        // Data loaded from memory
    output logic        mem_busy,         // Memory is busy (stall pipeline if needed)
    
    // Interface to the data memory
    // show it out，it is the same as the things up
    output logic [31:0] dmem_addr,        // Address to data memory
    output logic [31:0] dmem_write_data,  // Data to write to memory
    output logic        dmem_write,       // Write enable to data memory
    output logic        dmem_read,        // Read enable to data memory
    output logic [2:0]  dmem_funct3,      // Function code to data memory
    input  logic [31:0] dmem_read_data,   // Data read from memory
    input  logic        dmem_ready        // Memory operation complete signal
);

    // Memory state tracking
    // Memory state tracking
    localparam IDLE = 2'b00;
    localparam MEM_ACCESS = 2'b01;
    localparam COMPLETE = 2'b10;

    reg [1:0] current_state, next_state;
    
    // State machine for memory operations
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end
        else begin
            current_state <= next_state;
        end
    end
      
    // Next state logic
    always_comb begin
        
        case (current_state)
            IDLE: begin
                if (mem_read || mem_write) begin
                    next_state = MEM_ACCESS;
                end
            end
            
            MEM_ACCESS: begin
                if (dmem_ready) begin
                    next_state = COMPLETE;
                end
            end
            
            COMPLETE: begin
                next_state = IDLE;
            end
            
            default: begin
                next_state = IDLE;
            end
        endcase
    end
    
    // Output logic based on current state
    always_comb begin
        case (current_state)
            IDLE: begin
                mem_busy = 1'b0;
            end
            
            MEM_ACCESS: begin
                dmem_addr = addr;
                dmem_write_data = store_data;
                dmem_write = mem_write;
                dmem_read = mem_read;
                dmem_funct3 = funct3;
                mem_busy = 1'b1;  // Stall the pipeline if necessary
            end
            
            COMPLETE: begin
                load_data = dmem_read_data;
                mem_busy = 1'b0;  // Pipeline can proceed
            end
            
            default: begin
                mem_busy = 1'b0;
            end
        endcase
    end

endmodule


// Memory-to-Register Forwarding Unit - Special forwarding for load results
module mem_to_reg_forwarding (
    input  logic        mem_to_reg_mem,   // Memory-to-register control in MEM stage
    input  logic [4:0]  rd_addr_mem,      // Destination register in MEM stage
    // some data don't need to be added in memory
    input  logic [4:0]  rs1_addr_ex,      // Source register 1 in EX stage
    input  logic [4:0]  rs2_addr_ex,      // Source register 2 in EX stage
    input  logic [31:0] load_data_mem,    // Data loaded from memory in MEM stage
    input  logic [31:0] alu_input_a,      // Current ALU input A
    input  logic [31:0] alu_input_b,      // Current ALU input B
    output logic [31:0] forwarded_a,      // Potentially forwarded input A
    output logic [31:0] forwarded_b       // Potentially forwarded input B
);

    always_comb begin
        // Default: no forwarding
        
        // Check if we need to forward loaded data from memory to ALU inputs
        if (mem_to_reg_mem && rd_addr_mem != 5'b0) begin
            // Forward to input A if needed
            if (rd_addr_mem == rs1_addr_ex) begin
                forwarded_a = load_data_mem;
                forwarded_b = alu_input_b;
            end
            
            // Forward to input B if needed
            else if (rd_addr_mem == rs2_addr_ex) begin
                forwarded_b = load_data_mem;
                forwarded_a = alu_input_a;
            end

            else begin
                forwarded_a = alu_input_a;
                forwarded_b = alu_input_b;
            end
        end
        else begin
            forwarded_a = alu_input_a;
            forwarded_b = alu_input_b;
        end
    end

endmodule

