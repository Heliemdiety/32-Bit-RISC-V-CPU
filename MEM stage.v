// Memory Access (MEM) Stage of the RISC-V Pipeline.
// Performs load and store operations to/from data memory and handles load data sign extension.

module mem_stage (
    input wire clk,
    input wire rst_n, // Active low reset

    // Inputs from EX/MEM pipeline register
    input wire [31:0] alu_result_in,      // Address for memory access (calculated by ALU in EX stage)
    input wire [31:0] reg_read_data2_in,  // Data to write (for Store instructions)
    input wire        mem_read_en_in,     // Memory Read Enable
    input wire        mem_write_en_in,    // Memory Write Enable
    input wire [2:0]  funct3_in,          // For load/store type (byte, half, word, signed/unsigned)

    // Data Memory Interface (outputs to external data_memory module)
    output reg [31:0] mem_addr,           // Address to memory
    output reg [31:0] mem_write_data,     // Data to write to memory
    output reg        mem_write_en,       // Memory Write enable
    output reg        mem_read_en,        // Memory Read enable
    output reg [3:0]  mem_byte_enable,    // Byte enable for partial word writes (to data_memory)
    input wire  [31:0] mem_read_data_in,  // Data read from memory (input from data_memory module)

    // Outputs to MEM/WB pipeline register
    output reg [31:0] load_data_out       // Actual data loaded from memory (after potential sign/zero extension)
);

    // This block drives the control signals and data for the data memory.
    // It is synchronous because memory operations typically occur on a clock edge.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // On reset, set all memory control outputs to inactive/safe defaults.
            mem_addr        <= 32'b0;
            mem_write_data  <= 32'b0;
            mem_write_en    <= 1'b0;
            mem_read_en     <= 1'b0;
            mem_byte_enable <= 4'b0;
        end else begin
            // Propagate memory access signals from the EX/MEM register.
            mem_addr       <= alu_result_in;       // ALU result is the memory address
            mem_write_data <= reg_read_data2_in;   // Data to write is from rs2
            mem_write_en   <= mem_write_en_in;     // Enable write if instruction is a store
            mem_read_en    <= mem_read_en_in;      // Enable read if instruction is a load

            // Determine byte enable based on funct3 for store instructions.
            // The memory address's two least significant bits (addr[1:0]) indicate the byte offset within the word.
            // We align byte_enable for the whole word, assuming mem_addr provides the byte address.
            // The data_memory module expects a full 32-bit word address (addr[31:2]) and handles the offset internally.
            // Here, we generate the 4-bit byte_enable mask for the entire word based on the funct3.
            case (funct3_in)
                3'b000: mem_byte_enable = 4'b0001 << alu_result_in[1:0]; // SB (Store Byte) - enable 1 byte at addr[1:0] offset
                3'b001: mem_byte_enable = 4'b0011 << alu_result_in[1:0]; // SH (Store Halfword) - enable 2 bytes at addr[1:0] offset
                3'b010: mem_byte_enable = 4'b1111;                       // SW (Store Word) - enable all 4 bytes
                default: mem_byte_enable = 4'b0; // No valid store operation, or load (byte enable not applicable for loads)
            endcase
        end
    end

    // This combinational block performs sign/zero extension for loaded data.
    // The `load_data_out` is the final data that goes to the MEM/WB register.
    always @(*) begin
        case (funct3_in)
            3'b000: load_data_out = {{24{mem_read_data_in[7]}}, mem_read_data_in[7:0]};   // LB (Load Byte, sign-extended)
            3'b001: load_data_out = {{16{mem_read_data_in[15]}}, mem_read_data_in[15:0]}; // LH (Load Halfword, sign-extended)
            3'b010: load_data_out = mem_read_data_in;                                     // LW (Load Word)
            3'b100: load_data_out = {24'b0, mem_read_data_in[7:0]};                       // LBU (Load Byte Unsigned)
            3'b101: load_data_out = {16'b0, mem_read_data_in[15:0]};                      // LHU (Load Halfword Unsigned)
            default: load_data_out = 32'b0; // Default for non-load operations or invalid funct3
        endcase
    end

endmodule
