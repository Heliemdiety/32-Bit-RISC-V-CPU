// Instruction Decode (ID) Stage of the RISC-V Pipeline.
// Decodes instructions, reads register file, and generates control signals.

module id_stage (
    input wire clk,
    input wire rst_n, // Active low reset

    input wire [31:0] instruction_in, // Instruction from IF/ID register
    input wire [31:0] pc_curr_if,     // Current PC from IF stage (for AUIPC calculation)

    // Register File Interface (read ports)
    output wire [4:0] reg_file_read_addr1, // Address for RS1 (output to reg_file)
    output wire [4:0] reg_file_read_addr2, // Address for RS2 (output to reg_file)
    input wire [31:0] reg_file_read_data1, // Data read from RS1 (input from reg_file)
    input wire [31:0] reg_file_read_data2, // Data read from RS2 (input from reg_file)

    // Outputs to ID/EX pipeline register (and some for Hazard Unit)
    output wire [31:0] immediate_out,
    output wire [4:0]  rs1_addr_out, // For hazard unit & ID/EX reg
    output wire [4:0]  rs2_addr_out, // For hazard unit & ID/EX reg
    output wire [4:0]  rd_addr_out,  // For hazard unit & ID/EX reg
    output wire        reg_write_en_out,
    output wire [1:0]  mem_to_reg_out,
    output wire [3:0]  alu_op_out,
    output wire [1:0]  alu_src_a_out, // 00: pc, 01: reg_data1
    output wire [1:0]  alu_src_b_out, // 00: reg_data2, 01: immediate
    output wire        mem_read_en_out,
    output wire        mem_write_en_out,
    output wire        branch_out,
    output wire        jump_out,    // For JAL
    output wire        jalr_out,    // For JALR
    output wire [2:0]  funct3_out,
    output wire [6:0]  funct7_out,

    // Outputs for custom instruction detection (to Control Unit and ID/EX register)
    output wire        is_min_u_instr_out,      // Signal for MIN_U instruction
    output wire        is_abs_diff_u_instr_out, // Signal for ABS_DIFF_U instruction

    output wire [6:0]  opcode_debug           // Temporary debug output
);
    // Internal wires for instruction field extraction
    wire [6:0] opcode_local;
    wire [4:0] rd_local;
    wire [2:0] funct3_local;
    wire [4:0] rs1_local;
    wire [4:0] rs2_local;
    wire [6:0] funct7_local;
    wire [31:0] immediate_local; // Consolidated immediate output

    // Local parameters for custom instruction opcodes and functs (must match control_unit.v)
    localparam OPCODE_CUSTOM0    = 7'b0001011; // Standard RISC-V CUSTOM0 opcode

    localparam FUNCT3_MIN_U      = 3'b000;    // Funct3 for MIN_U
    localparam FUNCT7_MIN_U      = 7'b0000000; // Funct7 for MIN_U

    localparam FUNCT3_ABS_DIFF_U = 3'b001;    // Funct3 for ABS_DIFF_U
    localparam FUNCT7_ABS_DIFF_U = 7'b0000000; // Funct7 for ABS_DIFF_U


    
    // 1. Instruction Field Extraction
    
    assign opcode_local = instruction_in[6:0];
    assign rd_local     = instruction_in[11:7];
    assign funct3_local = instruction_in[14:12];
    assign rs1_local    = instruction_in[19:15];
    assign rs2_local    = instruction_in[24:20];
    assign funct7_local = instruction_in[31:25]; // Crucial for R-type and some I-type ops

    // Assign the debug output
    assign opcode_debug = opcode_local;

    
    // 2. Immediate Generation based on Instruction Type
    
    // I-Type Immediate: imm[11:0]
    wire [31:0] imm_i = {{20{instruction_in[31]}}, instruction_in[31:20]};

    // S-Type Immediate: imm[11:5] inst[31:25], imm[4:0] inst[11:7]
    wire [31:0] imm_s = {{20{instruction_in[31]}}, instruction_in[31:25], instruction_in[11:7]};

    // B-Type Immediate: imm[12|10:5|4:1|11] => inst[31|30:25|11:8|7] (shifted left by 1, with 1'b0 LSB)
    wire [31:0] imm_b = {{20{instruction_in[31]}}, instruction_in[7], instruction_in[30:25], instruction_in[11:8], 1'b0};

    // U-Type Immediate: imm[31:12] << 12
    wire [31:0] imm_u = {instruction_in[31:12], 12'b0};

    // J-Type Immediate: imm[20|10:1|11|19:12] => inst[31|30:21|20|19:12] (shifted left by 1, with 1'b0 LSB)
    wire [31:0] imm_j = {{12{instruction_in[31]}}, instruction_in[19:12], instruction_in[20], instruction_in[30:21], 1'b0};

    // Select the correct immediate value based on opcode
    assign immediate_local = (opcode_local == 7'b0010011 || opcode_local == 7'b0000011 || opcode_local == 7'b1100111) ? imm_i : // I-Type (ADDI, LW, JALR)
                             (opcode_local == 7'b0100011) ? imm_s : // S-Type (SW)
                             (opcode_local == 7'b1100011) ? imm_b : // B-Type (BEQ, BNE, etc.)
                             (opcode_local == 7'b0110111 || opcode_local == 7'b0010111) ? imm_u : // U-Type (LUI, AUIPC)
                             (opcode_local == 7'b1101111) ? imm_j : // J-Type (JAL)
                             32'b0; // Default for others

    assign immediate_out = immediate_local;

    
    // 3. Custom Instruction Detection Logic
    // Detect MIN_U instruction: CUSTOM0 opcode + specific funct3 + funct7
    assign is_min_u_instr_out = (opcode_local == OPCODE_CUSTOM0) &&
                                (funct3_local == FUNCT3_MIN_U) &&
                                (funct7_local == FUNCT7_MIN_U);

    // Detect ABS_DIFF_U instruction: CUSTOM0 opcode + specific funct3 + funct7
    assign is_abs_diff_u_instr_out = (opcode_local == OPCODE_CUSTOM0) &&
                                     (funct3_local == FUNCT3_ABS_DIFF_U) &&
                                     (funct7_local == FUNCT7_ABS_DIFF_U);

    
    // 4. Control Unit Instantiation
    // The Control Unit decodes the instruction fields and custom flags to generate appropriate control signals for subsequent pipeline stages.
    control_unit control_unit_inst (
        .opcode          (opcode_local),
        .funct3          (funct3_local),
        .funct7          (funct7_local),
        .is_min_u        (is_min_u_instr_out),      // Pass custom flag to Control Unit
        .is_abs_diff_u   (is_abs_diff_u_instr_out), // Pass custom flag to Control Unit
        .reg_write_en    (reg_write_en_out),
        .mem_to_reg      (mem_to_reg_out),
        .alu_op          (alu_op_out),
        .alu_src_a       (alu_src_a_out),
        .alu_src_b       (alu_src_b_out),
        .mem_read_en     (mem_read_en_out),
        .mem_write_en    (mem_write_en_out),
        .branch          (branch_out),
        .jump            (jump_out),
        .jalr            (jalr_out)
    );

    
    // 5. Register File Read Addresses

    assign reg_file_read_addr1 = rs1_local;
    assign reg_file_read_addr2 = rs2_local;


    // 6. Outputs to ID/EX Pipeline Register
    assign rs1_addr_out = rs1_local;
    assign rs2_addr_out = rs2_local;
    assign rd_addr_out  = rd_local;
    assign funct3_out   = funct3_local;
    assign funct7_out   = funct7_local; 


    // Debug prints for ID stage internal values (can be removed for synthesis)
    always @(*) begin
        $display("    --- ID_STAGE Internal Debug (Time %0t) ---", $time);
        $display("    Instruction_in (from IF/ID Reg): 0x%H", instruction_in);
        $display("    Extracted Fields: opcode=%b, funct3=%b, funct7=%b",
                 opcode_local, funct3_local, funct7_local);
        $display("    Extracted Reg Addrs: rs1=%d, rs2=%d, rd=%d",
                 rs1_local, rs2_local, rd_local);
        $display("    Generated Immediate: 0x%H", immediate_local);
        $display("    Custom Instr Flags: MIN_U=%b, ABS_DIFF_U=%b",
                 is_min_u_instr_out, is_abs_diff_u_instr_out);
        
        
        
        $display("    -------------------------------------------");
    
    end

endmodule
