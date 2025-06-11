// MEM/WB Pipeline Register.
// Stores the ALU result (for R/I-type), loaded data (for Load),
// destination register address, and write enable signals from MEM stage
// and passes them to the WB stage.

module mem_wb_reg (
    input wire clk,
    input wire rst_n, // Active low reset

    // Inputs from MEM stage
    input wire [31:0] mem_alu_result_in,
    input wire [31:0] mem_load_data_in,
    input wire [4:0]  mem_rd_addr_in,
    input wire        mem_reg_write_en_in,
    input wire [1:0]  mem_mem_to_reg_in,

    // Outputs to WB stage (directly to Register File write port)
    output reg [31:0] wb_alu_result_out,
    output reg [31:0] wb_load_data_out,
    output reg [4:0]  wb_rd_addr_out,
    output reg        wb_reg_write_en_out,
    output reg [1:0]  wb_mem_to_reg_out
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wb_alu_result_out   <= 32'b0;
            wb_load_data_out    <= 32'b0;
            wb_rd_addr_out      <= 5'b0;
            wb_reg_write_en_out <= 1'b0;
            wb_mem_to_reg_out   <= 2'b00;
        end else begin // No explicit stall for MEM/WB, bubble propagates
            wb_alu_result_out   <= mem_alu_result_in;
            wb_load_data_out    <= mem_load_data_in;
            wb_rd_addr_out      <= mem_rd_addr_in;
            wb_reg_write_en_out <= mem_reg_write_en_in;
            wb_mem_to_reg_out   <= mem_mem_to_reg_in;
        end
    end

endmodule
