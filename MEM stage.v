// Memory Access (MEM) Stage of the RISC-V CPU pipeline.
// Responsible for:
// - Performing actual data memory reads (for Load instructions).
// - Performing actual data memory writes (for Store instructions).
// - Passing loaded data (if any) and ALU result to the MEM/WB pipeline register.

module mem_stage (
    input wire clk,
    input wire rst_n, // Active low reset

    // Inputs from EX/MEM pipeline register
    input wire [31:0] alu_result_in,      // Address for memory access
    input wire [31:0] reg_read_data2_in,  // Data to write (for Store instructions)
    input wire        mem_read_en_in,     // Memory Read Enable
    input wire        mem_write_en_in,    // Memory Write Enable
    input wire [2:0]  funct3_in,          // For load/store type (byte, half, word)

    // Data Memory Interface (to external memory)
    output reg [31:0] mem_addr,         // Address to memory
    output reg [31:0] mem_write_data,   // Data to write to memory
    output reg        mem_write_en,     // Memory Write enable
    output reg        mem_read_en,      // Memory Read enable
    output reg [3:0]  mem_byte_enable,  // Byte enable for partial word writes
    input wire  [31:0] mem_read_data_in, // Data read from memory

    // Outputs to MEM/WB pipeline register
    output reg [31:0] load_data_out     // Actual data loaded from memory (after potential sign extension)
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_addr <= 32'b0;
            mem_write_data <= 32'b0;
            mem_write_en <= 1'b0;
            mem_read_en <= 1'b0;
            mem_byte_enable <= 4'b0;
        end else begin
            // Drive memory control signals based on inputs from EX/MEM
            mem_addr <= alu_result_in;
            mem_write_data <= reg_read_data2_in; // This is the value to be stored
            mem_write_en <= mem_write_en_in;
            mem_read_en <= mem_read_en_in;

            case (funct3_in)
                3'b000: mem_byte_enable = 4'b0001; // SB (store byte) 
                3'b001: mem_byte_enable = 4'b0011; // SH (store half) 
                3'b010: mem_byte_enable = 4'b1111; // SW (store word)
                default: mem_byte_enable = 4'b0; // No valid store operation
            endcase
        end
    end

   
    // This logic performs sign extension for byte/half-word loads.
    // The `load_data_out` is the output that goes to MEM/WB register.
    always @(*) begin // Combinational logic
        case (funct3_in)
            3'b000: load_data_out = {{24{mem_read_data_in[7]}}, mem_read_data_in[7:0]};   // LB (Load Byte, sign-extended)
            3'b001: load_data_out = {{16{mem_read_data_in[15]}}, mem_read_data_in[15:0]}; // LH (Load Halfword, sign-extended)
            3'b010: load_data_out = mem_read_data_in;                                     // LW (Load Word)
            3'b100: load_data_out = {24'b0, mem_read_data_in[7:0]};                        // LBU (Load Byte Unsigned)
            3'b101: load_data_out = {16'b0, mem_read_data_in[15:0]};                       // LHU (Load Halfword Unsigned)
            default: load_data_out = 32'b0; // Should not happen for valid load
        endcase
    end

endmodule

