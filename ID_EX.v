// ID/EX Pipeline Register.
// Stores the decoded instruction properties from ID stage and passes them to the EX stage.
// Includes logic for stalling (holding values) and flushing (inserting NOPs).

module id_ex_reg (
    input wire clk,
    input wire rst_n, // Active low reset

    // Control signals from Hazard Unit
    input wire stall, // If high, register holds its current value (inputs are not loaded)
    input wire flush, // If high, register is cleared (NOP instruction effectively inserted)

    // Inputs from ID stage
    input wire [31:0] id_pc_plus_4_in,
    input wire [31:0] id_reg_read_data1_in,
    input wire [31:0] id_reg_read_data2_in,
    input wire [31:0] id_immediate_in,
    input wire [4:0]  id_rs1_addr_in,
    input wire [4:0]  id_rs2_addr_in,
    input wire [4:0]  id_rd_addr_in,
    input wire        id_reg_write_en_in,
    input wire [1:0]  id_mem_to_reg_in,
    input wire [3:0]  id_alu_op_in,
    input wire [1:0]  id_alu_src_a_in,
    input wire [1:0]  id_alu_src_b_in,
    input wire        id_mem_read_en_in,
    input wire        id_mem_write_en_in,
    input wire        id_branch_in,
    input wire        id_jump_in,
    input wire        id_jalr_in,
    input wire [2:0]  id_funct3_in,
    input wire [6:0]  id_funct7_in,

    // Custom instruction flags from ID stage
    input wire        id_is_min_u_in,
    input wire        id_is_abs_diff_u_in,

    // Outputs to EX stage (registered versions of inputs)
    output reg [31:0] ex_pc_plus_4_out,
    output reg [31:0] ex_reg_read_data1_out,
    output reg [31:0] ex_reg_read_data2_out,
    output reg [31:0] ex_immediate_out,
    output reg [4:0]  ex_rs1_addr_out,
    output reg [4:0]  ex_rs2_addr_out,
    output reg [4:0]  ex_rd_addr_out,
    output reg        ex_reg_write_en_out,
    output reg [1:0]  ex_mem_to_reg_out,
    output reg [3:0]  ex_alu_op_out,
    output reg [1:0]  ex_alu_src_a_out,
    output reg [1:0]  ex_alu_src_b_out,
    output reg        ex_mem_read_en_out,
    output reg        ex_mem_write_en_out,
    output reg        ex_branch_out,
    output reg        ex_jump_out,
    output reg        ex_jalr_out,
    output reg [2:0]  ex_funct3_out,
    output reg [6:0]  ex_funct7_out,

    // Custom instruction flags output to EX stage
    output reg        ex_is_min_u_out,
    output reg        ex_is_abs_diff_u_out
);

    // NOP instruction (ADDI x0, x0, 0) for flushing the pipeline register.
    // The immediate 0x00000013 (ADDI x0, x0, 0) explicitly represents a NOP.
    localparam NOP_ALU_OP_CODE = 4'b0000; // ALU_ADD (from alu.v)
    localparam NOP_MEM_TO_REG  = 2'b00;   // ALU_Result (write to x0)
    localparam NOP_ALU_SRC_A   = 2'b00;   // PC (or anything)
    localparam NOP_ALU_SRC_B   = 2'b00;   // RegData2 (or anything)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush) begin
            // On reset or flush, clear all outputs to a NOP state.
            // This effectively injects a bubble (NOP) into the EX stage.
            ex_pc_plus_4_out     <= 32'b0;
            ex_reg_read_data1_out <= 32'b0;
            ex_reg_read_data2_out <= 32'b0;
            ex_immediate_out      <= 32'b0;
            ex_rs1_addr_out       <= 5'b0;
            ex_rs2_addr_out       <= 5'b0;
            ex_rd_addr_out        <= 5'b0; // Write to x0, which is ignored
            ex_reg_write_en_out   <= 1'b0; // No register write
            ex_mem_to_reg_out     <= NOP_MEM_TO_REG;
            ex_alu_op_out         <= NOP_ALU_OP_CODE;
            ex_alu_src_a_out      <= NOP_ALU_SRC_A;
            ex_alu_src_b_out      <= NOP_ALU_SRC_B;
            ex_mem_read_en_out    <= 1'b0;
            ex_mem_write_en_out   <= 1'b0;
            ex_branch_out         <= 1'b0;
            ex_jump_out           <= 1'b0;
            ex_jalr_out           <= 1'b0;
            ex_funct3_out         <= 3'b0;
            ex_funct7_out         <= 7'b0;
            ex_is_min_u_out       <= 1'b0;
            ex_is_abs_diff_u_out  <= 1'b0;

            $display("    ID/EX_REG Debug (Time %0t): FLUSH/RESET active. Outputting bubble. Stall=%b, Flush=%b", $time, stall, flush);
        end else if (!stall) begin
            // On clock edge, if not stalled, propagate inputs to outputs.
            ex_pc_plus_4_out     <= id_pc_plus_4_in;
            ex_reg_read_data1_out <= id_reg_read_data1_in;
            ex_reg_read_data2_out <= id_reg_read_data2_in;
            ex_immediate_out      <= id_immediate_in;
            ex_rs1_addr_out       <= id_rs1_addr_in;
            ex_rs2_addr_out       <= id_rs2_addr_in;
            ex_rd_addr_out        <= id_rd_addr_in;
            ex_reg_write_en_out   <= id_reg_write_en_in;
            ex_mem_to_reg_out     <= id_mem_to_reg_in;
            ex_alu_op_out         <= id_alu_op_in;
            ex_alu_src_a_out      <= id_alu_src_a_in;
            ex_alu_src_b_out      <= id_alu_src_b_in;
            ex_mem_read_en_out    <= id_mem_read_en_in;
            ex_mem_write_en_out   <= id_mem_write_en_in;
            ex_branch_out         <= id_branch_in;
            ex_jump_out           <= id_jump_in;
            ex_jalr_out           <= id_jalr_in;
            ex_funct3_out         <= id_funct3_in;
            ex_funct7_out         <= id_funct7_in;
            ex_is_min_u_out       <= id_is_min_u_in;
            ex_is_abs_diff_u_out  <= id_is_abs_diff_u_in;

            $display("    ID/EX_REG Debug (Time %0t): Updated. Input MemWriteEn=%b, Output MemWriteEn=%b. Stall=%b, Flush=%b",
                     $time, id_mem_write_en_in, ex_mem_write_en_out, stall, flush);
        end else begin
            // If stalled, outputs hold their current values.
            $display("    ID/EX_REG Debug (Time %0t): STALLED. No update. Output MemWriteEn=%b. Stall=%b, Flush=%b",
                     $time, ex_mem_write_en_out, stall, flush);
        end
    end

endmodule