// Execute (EX) Stage of the RISC-V Pipeline.
// Performs ALU operations, evaluates branch conditions, and handles forwarding.

module ex_stage (
    input wire clk,
    input wire rst_n, // Active low reset

    // Inputs from ID/EX pipeline register
    input wire [31:0] pc_plus_4_in,
    input wire [31:0] reg_read_data1_in,
    input wire [31:0] reg_read_data2_in,
    input wire [31:0] immediate_in,
    input wire [4:0]  rs1_addr_in, // For hazard unit / forwarding (debugging)
    input wire [4:0]  rs2_addr_in, // For hazard unit / forwarding (debugging)
    input wire [4:0]  rd_addr_in,  // For hazard unit
    input wire        reg_write_en_in, // For hazard unit (load-use)
    input wire        mem_read_en_in,  // For hazard unit (load-use)
    input wire        mem_write_en_in,
    input wire [1:0]  mem_to_reg_in,   // For hazard unit (load-use determination of forward data)
    input wire [3:0]  alu_op_in,       // ALU operation code (now includes custom ops)
    input wire [1:0]  alu_src_a_in,    // 00: PC, 01: RegData1
    input wire [1:0]  alu_src_b_in,    // 00: RegData2, 01: Immediate
    input wire        branch_in,       // Is this a branch instruction?
    input wire        jump_in,         // Is this a JAL instruction?
    input wire        jalr_in,         // Is this a JALR instruction?
    input wire [2:0]  funct3_in,       // For branch type and for load/store size
    input wire [6:0]  funct7_in,       // For R-type special ops (e.g., SUB/SRA)

    // Forwarding control from Hazard Unit
    input wire [1:0]  forward_a,   // 00: No forward (reg_data1_in), 01: EX/MEM.ALU_Result, 10: MEM/WB.Result
    input wire [1:0]  forward_b,   // 00: No forward (reg_data2_in), 01: EX/MEM.ALU_Result, 10: MEM/WB.Result
    // Forwarded data from later stages
    input wire [31:0] forward_ex_alu_result,    // ALU result from EX/MEM stage
    input wire [31:0] forward_mem_load_data,    // Load data from MEM/WB stage (for loads)
    input wire [31:0] forward_mem_alu_result,   // ALU result from MEM/WB stage (for non-loads)
    // **NEW**: Propagate PC+4 for JAL/JALR return address. This is needed for correct write-back.
    input wire [31:0] forward_mem_pc_plus_4,    // PC+4 from MEM/WB stage (for JAL/JALR)

    // Outputs to EX/MEM pipeline register
    output wire [31:0] alu_result_out,
    output wire [31:0] reg_read_data2_out,      // Value to be stored in memory (rs2)
    output wire        branch_taken_out,        // Signal if branch is actually taken (for PC update)
    output wire [31:0] branch_target_out,       // Calculated branch/jump target
    // Pass through for EX/MEM register (signals from ID/EX register)
    output wire [4:0]  rd_addr_out,
    output wire        reg_write_en_out,
    output wire [1:0]  mem_to_reg_out,
    output wire [2:0]  funct3_out,
    output wire [3:0]  alu_op_out,
    output wire        mem_read_en_out,
    output wire        mem_write_en_out,

    // Pass PC+4 to EX/MEM register. This is essential for correct JAL/JALR return address.
    output wire [31:0] pc_plus_4_out
);

    wire [31:0] alu_operand_a;
    wire [31:0] alu_operand_b;

    // Select ALU Operand A (reg_read_data1_in or pc_plus_4_in or forwarded data)
    // Priority: EX/MEM.ALU_Result (forward_a=01) > MEM/WB.Result (forward_a=10) > Raw/Direct (forward_a=00)
    assign alu_operand_a = (forward_a == 2'b01) ? forward_ex_alu_result :
                           (forward_a == 2'b10) ? (mem_to_reg_in == 2'b01 ? forward_mem_load_data : forward_mem_alu_result) :
                           (alu_src_a_in == 2'b00 ? pc_plus_4_in : reg_read_data1_in); // Default: PC or RegData1

    // Select ALU Operand B (reg_read_data2_in or immediate_in or forwarded data)
    // Priority: EX/MEM.ALU_Result (forward_b=01) > MEM/WB.Result (forward_b=10) > Raw/Direct (forward_b=00)
    assign alu_operand_b = (forward_b == 2'b01) ? forward_ex_alu_result :
                           (forward_b == 2'b10) ? (mem_to_reg_in == 2'b01 ? forward_mem_load_data : forward_mem_alu_result) :
                           (alu_src_b_in == 2'b00 ? reg_read_data2_in : immediate_in); // Default: RegData2 or Immediate


    // Logic for forwarding RegData2 (operand for store instructions)
    // This ensures that data being stored is also forwarded if it's currently being
    // written by a previous instruction still in the pipeline.
    assign reg_read_data2_out = (forward_b == 2'b01) ? forward_ex_alu_result :
                                (forward_b == 2'b10) ? (mem_to_reg_in == 2'b01 ? forward_mem_load_data : forward_mem_alu_result) :
                                reg_read_data2_in; // Default: Raw RegData2

    // Internal wires for ALU output and flags
    wire [31:0] alu_result_internal;
    wire        zero_flag;
    wire        less_flag;

    // Instantiate the ALU module
    alu alu_inst (
        .operand_a (alu_operand_a),
        .operand_b (alu_operand_b),
        .alu_op    (alu_op_in),    // ALU operation code (including custom ops)
        .funct3    (funct3_in),    // Passed for potential internal ALU decoding (though not used for custom ops here)
        .funct7    (funct7_in),    // Passed for potential internal ALU decoding
        .result    (alu_result_internal),
        .zero      (zero_flag),
        .less      (less_flag)
    );

    // Output the ALU result to the EX/MEM register
    assign alu_result_out = alu_result_internal;

    wire branch_condition_met;

    // Determine if branch condition is met based on funct3 and ALU flags
    // The ALU calculates (RegA - RegB), and zero_flag/less_flag indicate the comparison result.
    assign branch_condition_met = (funct3_in == 3'b000) ? zero_flag :    // BEQ (Equal) -> (A-B == 0)
                                  (funct3_in == 3'b001) ? ~zero_flag :   // BNE (Not Equal) -> (A-B != 0)
                                  (funct3_in == 3'b100) ? less_flag :    // BLT (Signed Less Than) -> signed(A) < signed(B)
                                  (funct3_in == 3'b101) ? ~less_flag :   // BGE (Signed Greater Than or Equal) -> signed(A) >= signed(B)
                                  (funct3_in == 3'b110) ? less_flag :    // BLTU (Unsigned Less Than) -> unsigned(A) < unsigned(B)
                                  (funct3_in == 3'b111) ? ~less_flag :   // BGEU (Unsigned Greater Than or Equal) -> unsigned(A) >= unsigned(B)
                                  1'b0; // Default for invalid funct3 (should not happen for branch opcode)

    // Calculate branch/jump target address
    // For JALR, the ALU already computed the target (base + immediate).
    // For JAL/Branches, it's PC_plus_4 (current PC + 4) + Immediate.
    assign branch_target_out = (jalr_in) ? alu_result_internal :
                               (jump_in || branch_in) ? (pc_plus_4_in + immediate_in) :
                               32'b0; // Default (should not be used unless jump/branch)

    // Determine if the branch is actually taken (conditional branches only)
    assign branch_taken_out = branch_in && branch_condition_met;

    // Outputs passed directly to EX/MEM register
    assign rd_addr_out        = rd_addr_in;
    assign reg_write_en_out   = reg_write_en_in;
    assign mem_to_reg_out     = mem_to_reg_in;
    assign funct3_out         = funct3_in;
    assign alu_op_out         = alu_op_in; // Pass alu_op for debug or for MEM stage if needed
    assign mem_read_en_out    = mem_read_en_in;
    assign mem_write_en_out   = mem_write_en_in;
    // Pass PC+4 from ID/EX through to EX/MEM. This is crucial for JAL/JALR return address.
    assign pc_plus_4_out      = pc_plus_4_in;

endmodule
