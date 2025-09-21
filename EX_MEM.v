// EX/MEM Pipeline Register.
// Stores the results and control signals from the EX stage and passes them to the MEM stage.

module ex_mem_reg (
    input wire clk,
    input wire rst_n, // Active low reset

    // Inputs from EX stage
    input wire [31:0] ex_pc_plus_4_in,      // **NEW**: PC+4 from EX stage (for JAL/JALR return address)
    input wire [31:0] ex_alu_result_in,
    input wire [31:0] ex_reg_read_data2_in, // Data to be stored (for store instructions)
    input wire [4:0]  ex_rd_addr_in,
    input wire        ex_reg_write_en_in,
    input wire [1:0]  ex_mem_to_reg_in,
    input wire        ex_mem_read_en_in,
    input wire        ex_mem_write_en_in,
    input wire [2:0]  ex_funct3_in,         // For load/store type in MEM stage

    // Outputs to MEM stage
    output reg [31:0] mem_pc_plus_4_out,    // **NEW**: PC+4 out to MEM stage
    output reg [31:0] mem_alu_result_out,
    output reg [31:0] mem_reg_read_data2_out,
    output reg [4:0]  mem_rd_addr_out,
    output reg        mem_reg_write_en_out,
    output reg [1:0]  mem_mem_to_reg_out,
    output reg        mem_mem_read_en_out,
    output reg        mem_mem_write_en_out,
    output reg [2:0]  mem_funct3_out
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all outputs to known, safe defaults (NOP equivalent)
            mem_pc_plus_4_out    <= 32'b0;
            mem_alu_result_out   <= 32'b0;
            mem_reg_read_data2_out <= 32'b0;
            mem_rd_addr_out      <= 5'b0;
            mem_reg_write_en_out <= 1'b0;
            mem_mem_to_reg_out   <= 2'b00;
            mem_mem_read_en_out  <= 1'b0;
            mem_mem_write_en_out <= 1'b0;
            mem_funct3_out       <= 3'b0;
        end else begin
            // Normal operation: propagate inputs to outputs on clock edge.
            // (No explicit stall input for this register; bubbles/flushes propagate from previous stages.)
            mem_pc_plus_4_out    <= ex_pc_plus_4_in;
            mem_alu_result_out   <= ex_alu_result_in;
            mem_reg_read_data2_out <= ex_reg_read_data2_in;
            mem_rd_addr_out      <= ex_rd_addr_in;
            mem_reg_write_en_out <= ex_reg_write_en_in;
            mem_mem_to_reg_out   <= ex_mem_to_reg_in;
            mem_mem_read_en_out  <= ex_mem_read_en_in;
            mem_mem_write_en_out <= ex_mem_write_en_in;
            mem_funct3_out       <= ex_funct3_in;
        end
    end

endmodule

