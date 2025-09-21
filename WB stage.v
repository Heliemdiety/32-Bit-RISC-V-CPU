// Write Back (WB) Stage of the RISC-V Pipeline.
// Selects the final data to be written back to the Register File from either
// the ALU result, loaded data from memory, or PC+4 for JAL/JALR return addresses.

module wb_stage (
    input wire clk,
    input wire rst_n, // Active low reset (though internal logic is combinational)

    // Inputs from MEM/WB pipeline register
    input wire [31:0] alu_result_in,   // ALU result from EX stage (propagated through MEM)
    input wire [31:0] load_data_in,    // Data loaded from memory (after sign/zero extension)
    input wire [31:0] pc_plus_4_in,    // PC+4 value for JAL/JALR return address
    input wire [4:0]  rd_addr_in,      // Destination register address
    input wire        reg_write_en_in, // Register Write Enable signal
    input wire [1:0]  mem_to_reg_in,   // Control signal to select write-back data source

    // Outputs to Register File (these are combinational outputs)
    output wire [31:0] wb_write_data_out,   // Final data to be written to the register file
    output wire [4:0]  wb_write_addr_out,   // Destination register address for the write
    output wire        wb_write_en_out      // Final write enable signal for the register file
);

    // Combinational logic to select the data to write back to the register file.
    // This multiplexer selects one of three possible sources:
    // 00: ALU result (for R-type and I-type arithmetic/logical operations)
    // 01: Loaded data (for Load instructions like LW, LH, LB)
    // 10: PC+4 (for JAL and JALR instructions, saving the return address)
    assign wb_write_data_out = (mem_to_reg_in == 2'b00) ? alu_result_in :     // Select ALU result
                               (mem_to_reg_in == 2'b01) ? load_data_in :      // Select loaded data
                               (mem_to_reg_in == 2'b10) ? pc_plus_4_in :      // Select PC+4
                               32'b0; // Default case for invalid mem_to_reg_in (should not happen with correct control)

    // Pass through the destination register address and the register write enable signal.
    // These signals originated in the ID stage, propagated through EX and MEM stages.
    assign wb_write_addr_out = rd_addr_in;
    assign wb_write_en_out   = reg_write_en_in;

endmodule
