// Responsible for:
// - Writing the result of an operation (ALU result or loaded data)
//   back to the Register File.
// In this simplified design, the write-back logic is mostly handled
// by the direct connection from the MEM/WB register to the Register File's
// write port, controlled by `mem_reg_write_en_out` and `mem_mem_to_reg_out`.
// This module mainly serves as a conceptual stage for the pipeline.

module wb_stage (
    input wire clk,
    input wire rst_n, // Active low reset

    // Inputs from MEM/WB pipeline register
    input wire [31:0] alu_result_in,  // ALU result for R-type/I-type (already selected for write)
    input wire [31:0] load_data_in,   // Data loaded from memory (already selected for write)
    input wire [4:0]  rd_addr_in,     // Destination register address
    input wire        reg_write_en_in, // Register Write Enable
    input wire [1:0]  mem_to_reg_in   // Controls source of write-back data (ALU result, Load data, PC+4)

);

    

endmodule
