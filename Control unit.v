// Control Unit for the RISC-V CPU.
// Decodes instructions and generates control signals for all pipeline stages.
// Includes logic for standard RV32I instructions and custom coprocessor instructions.

module control_unit (
    input wire [6:0] opcode,
    input wire [2:0] funct3,
    input wire [6:0] funct7,

    // Inputs for custom instruction flags from ID stage.
    // These flags indicate if the current instruction is a custom one.
    input wire        is_min_u,      // Flag for MIN_U custom instruction
    input wire        is_abs_diff_u, // Flag for ABS_DIFF_U custom instruction

    // Outputs for pipeline control:
    output reg        reg_write_en, // Register Write Enable (to RegFile)
    output reg [1:0]  mem_to_reg,   // Mux select for WB stage (00: ALU_result, 01: Load_data, 10: PC+4)
    output reg [3:0]  alu_op,       // ALU operation code (to ALU)
    output reg [1:0]  alu_src_a,    // Mux select for ALU operand A (00: PC, 01: RegData1)
    output reg [1:0]  alu_src_b,    // Mux select for ALU operand B (00: RegData2, 01: Immediate)
    output reg        mem_read_en,  // Memory Read Enable (to Data Memory)
    output reg        mem_write_en, // Memory Write Enable (to Data Memory)
    output reg        branch,       // Branch instruction detected (for PC update logic)
    output reg        jump,         // JAL instruction detected (for PC update logic)
    output reg        jalr          // JALR instruction detected (for PC update logic)
);

    // ALU Operation Codes (Must match alu.v)
    localparam ALU_ADD          = 4'b0000;
    localparam ALU_SUB          = 4'b0001;
    localparam ALU_SLL          = 4'b0010;
    localparam ALU_SLT          = 4'b0011;
    localparam ALU_SLTU         = 4'b0100;
    localparam ALU_XOR          = 4'b0101;
    localparam ALU_SRL          = 4'b0110;
    localparam ALU_SRA          = 4'b0111;
    localparam ALU_OR           = 4'b1000;
    localparam ALU_AND          = 4'b1001;
    localparam ALU_COPY_B       = 4'b1011;

    // Custom ALU Operation Codes for Graph Algorithms (Must match alu.v)
    localparam ALU_MIN_U        = 4'b1101;
    localparam ALU_ABS_DIFF_U   = 4'b1110;

    // RISC-V Opcode Definitions (common to RV32I)
    localparam OPCODE_R_TYPE    = 7'b0110011; // R-type instructions (ADD, SUB, SLL, etc.)
    localparam OPCODE_I_TYPE    = 7'b0010011; // I-type instructions (ADDI, XORI, etc.)
    localparam OPCODE_LOAD      = 7'b0000011; // Load instructions (LB, LH, LW)
    localparam OPCODE_STORE     = 7'b0100011; // Store instructions (SB, SH, SW)
    localparam OPCODE_BRANCH    = 7'b1100011; // Branch instructions (BEQ, BNE, etc.)
    localparam OPCODE_LUI       = 7'b0110111; // Load Upper Immediate
    localparam OPCODE_AUIPC     = 7'b0010111; // Add Upper Immediate to PC
    localparam OPCODE_JAL       = 7'b1101111; // Jump and Link
    localparam OPCODE_JALR      = 7'b1100111; // Jump and Link Register
    localparam OPCODE_SYSTEM    = 7'b1110011; // System calls (ECALL, EBREAK)

    // Custom Opcode (must match id_stage.v)
    localparam OPCODE_CUSTOM0   = 7'b0001011; // Standard RISC-V CUSTOM0 opcode

    always @(*) begin
        // Default values for all outputs to prevent 'X' propagation and define a safe NOP state.
        reg_write_en   = 1'b0;
        mem_to_reg     = 2'b00; // Default: ALU result writes to Reg
        alu_op         = ALU_ADD; // Default: ADD (can act as a safe NOP for rd=x0)
        alu_src_a      = 2'b00; // Default: PC
        alu_src_b      = 2'b00; // Default: RegData2
        mem_read_en    = 1'b0;
        mem_write_en   = 1'b0;
        branch         = 1'b0;
        jump           = 1'b0;
        jalr           = 1'b0;

        // Debug prints for control unit inputs
        $display("    --- CONTROL_UNIT Internal Debug (Time %0t) ---", $time);
        $display("    Inputs: opcode=%b, funct3=%b, funct7=%b", opcode, funct3, funct7);
        $display("    Custom Instr Flags: MIN_U=%b, ABS_DIFF_U=%b", is_min_u, is_abs_diff_u);

        // Prioritize custom instructions if detected.
        // This ensures that if an instruction matches a custom opcode/funct combination,
        // it is handled as a custom instruction, overriding standard decoding.
        
        // currently commented this out and explicitly handled the custom instructions 
        // if (is_min_u) begin
        //     reg_write_en  = 1'b1;
        //     mem_to_reg    = 2'b00; // ALU result
        //     alu_op        = ALU_MIN_U;
        //     alu_src_a     = 2'b01; // RegData1
        //     alu_src_b     = 2'b00; // RegData2
        // end else if (is_abs_diff_u) begin
        //     reg_write_en  = 1'b1;
        //     mem_to_reg    = 2'b00; // ALU result
        //     alu_op        = ALU_ABS_DIFF_U;
        //     alu_src_a     = 2'b01; // RegData1
        //     alu_src_b     = 2'b00; // RegData2
        // end else begin
            // Standard RISC-V instruction decoding based on opcode
            case (opcode)
                OPCODE_R_TYPE: begin // R-Type (reg-reg arithmetic/logical) - 0110011 (0x33)
                    reg_write_en = 1'b1;
                    mem_to_reg   = 2'b00; // ALU result
                    alu_src_a    = 2'b01; // RegData1
                    alu_src_b    = 2'b00; // RegData2 (RegData2 is used as the second operand)
                    case ({funct7, funct3}) // Decode based on funct7 and funct3
                        10'b0000000000: alu_op = ALU_ADD;   // ADD
                        10'b0100000000: alu_op = ALU_SUB;   // SUB
                        10'b0000000001: alu_op = ALU_SLL;   // SLL
                        10'b0000000010: alu_op = ALU_SLT;   // SLT
                        10'b0000000011: alu_op = ALU_SLTU;  // SLTU
                        10'b0000000100: alu_op = ALU_XOR;   // XOR
                        10'b0000000101: alu_op = ALU_SRL;   // SRL
                        10'b0100000101: alu_op = ALU_SRA;   // SRA
                        10'b0000000110: alu_op = ALU_OR;    // OR
                        10'b0000000111: alu_op = ALU_AND;   // AND
                        default: begin
                            $display("ERROR: Unrecognized R-Type funct3/funct7: %b/%b at time %0t", funct3, funct7, $time);
                            // Control signals remain at default (NOP-like)
                        end
                    endcase
                end

                OPCODE_I_TYPE: begin // I-Type (Immediate arithmetic/logical) - 0010011 (0x13)
                    reg_write_en = 1'b1;
                    mem_to_reg   = 2'b00; // ALU result
                    alu_src_a    = 2'b01; // RegData1
                    alu_src_b    = 2'b01; // Immediate is the second operand
                    case (funct3)
                        3'b000: alu_op = ALU_ADD;   // ADDI
                        3'b001: alu_op = ALU_SLL;   // SLLI (funct7 must be 0)
                        3'b010: alu_op = ALU_SLT;   // SLTI
                        3'b011: alu_op = ALU_SLTU;  // SLTIU
                        3'b100: alu_op = ALU_XOR;   // XORI
                        3'b101: begin // SRLI / SRAI
                            if (funct7 == 7'b0000000) alu_op = ALU_SRL; // SRLI
                            else if (funct7 == 7'b0100000) alu_op = ALU_SRA; // SRAI
                            else begin
                                $display("ERROR: Unrecognized I-Type funct7 for shift: %b at time %0t", funct7, $time);
                                alu_op = ALU_ADD; // Invalid, default to ADD
                            end
                        end
                        3'b110: alu_op = ALU_OR;    // ORI
                        3'b111: alu_op = ALU_AND;   // ANDI
                        default: begin
                            $display("ERROR: Unrecognized I-Type funct3: %b at time %0t", funct3, $time);
                        end
                    endcase
                end

                OPCODE_LOAD: begin // Load instructions - 0000011 (0x03)
                    reg_write_en = 1'b1;
                    mem_to_reg   = 2'b01; // Load data from memory
                    alu_op       = ALU_ADD; // Calculate address: RS1 + Immediate
                    alu_src_a    = 2'b01; // RegData1
                    alu_src_b    = 2'b01; // Immediate
                    mem_read_en  = 1'b1;
                    // funct3 dictates load type (LB, LH, LW, LBU, LHU) - handled in MEM stage
                end

                OPCODE_STORE: begin // S-Type (Store instructions) - 0100011 (0x23)
                    reg_write_en = 1'b0; // Stores do not write to register file
                    mem_to_reg   = 2'b00; // N/A, but safer to default
                    alu_op       = ALU_ADD; // Calculate address: RS1 + Immediate
                    alu_src_a    = 2'b01; // RegData1
                    alu_src_b    = 2'b01; // Immediate
                    mem_write_en = 1'b1;
                    // funct3 dictates store type (SB, SH, SW) - handled in MEM stage
                end

                OPCODE_BRANCH: begin // B-Type (Branch instructions) - 1100011 (0x63)
                    reg_write_en = 1'b0; // Branches don't write to register file
                    mem_to_reg   = 2'b00; // N/A
                    alu_op       = ALU_SUB; // ALU performs subtraction for comparison (rs1 - rs2)
                    alu_src_a    = 2'b01; // RegData1
                    alu_src_b    = 2'b00; // RegData2 (for comparison)
                    branch       = 1'b1; // Signal for PC update logic in EX stage
                    // funct3 dictates branch type (BEQ, BNE, BLT, BGE, BLTU, BGEU) - handled in EX stage
                end

                OPCODE_LUI: begin // LUI (Load Upper Immediate) - 0110111 (0x37)
                    reg_write_en = 1'b1;
                    mem_to_reg   = 2'b00; // ALU result
                    alu_op       = ALU_COPY_B; // Just copy immediate to result
                    alu_src_a    = 2'b00; // Not used as operand_a for ALU_COPY_B
                    alu_src_b    = 2'b01; // Immediate is operand B
                end

                OPCODE_AUIPC: begin // AUIPC (Add Upper Immediate to PC) - 0010111 (0x17)
                    reg_write_en = 1'b1;
                    mem_to_reg   = 2'b00; // ALU result
                    alu_op       = ALU_ADD; // PC + Immediate
                    alu_src_a    = 2'b00; // PC is operand A
                    alu_src_b    = 2'b01; // Immediate is operand B
                end

                OPCODE_JAL: begin // JAL (Jump and Link) - 1101111 (0x6F)
                    reg_write_en = 1'b1;
                    mem_to_reg   = 2'b10; // Write PC+4 to Rd
                    alu_op       = ALU_ADD; // ALU calculates target (PC + Immediate)
                    alu_src_a    = 2'b00; // PC is operand A
                    alu_src_b    = 2'b01; // Immediate is operand B
                    jump         = 1'b1; // Signal for PC update
                end

                OPCODE_JALR: begin // JALR (Jump and Link Register) - 1100111 (0x67)
                    reg_write_en = 1'b1;
                    mem_to_reg   = 2'b10; // Write PC+4 to Rd
                    alu_op       = ALU_ADD; // ALU calculates target (RS1 + Immediate)
                    alu_src_a    = 2'b01; // RegData1 is operand A
                    alu_src_b    = 2'b01; // Immediate is operand B
                    jalr         = 1'b1; // Signal for PC update
                end

                OPCODE_SYSTEM: begin // ECALL / EBREAK (System calls) - 1110011 (0x73)
                    // These instructions typically do not write to registers or access memory
                    // in a basic pipeline. Control signals remain at default (0).
                end

                
                // this part here is explicitly handling the custom opcode for debgging 
                OPCODE_CUSTOM0: begin
                reg_write_en = 1'b1;     // Custom ops write to a register
                mem_to_reg   = 2'b00;    // Result is directly from ALU
                alu_src_a    = 2'b01;    // Operand A is from RegData1 (rs1)
                alu_src_b    = 2'b00;    // Operand B is from RegData2 (rs2)
                mem_read_en  = 1'b0;     // No memory read
                mem_write_en = 1'b0;     // No memory write
                branch       = 1'b0;     // Not a branch
                jump         = 1'b0;     // Not a jump
                jalr         = 1'b0;     // Not a JALR

                // Use funct3 to distinguish between MIN_U and ABS_DIFF_U
                case (funct3)
                    3'b000: alu_op = ALU_MIN_U;      // Corresponds to MIN_U
                    3'b001: alu_op = ALU_ABS_DIFF_U; // Corresponds to ABS_DIFF_U
                    default: begin
                        // Fallback for unrecognized funct3 within custom opcode
                        reg_write_en = 1'b0; // Treat as NOP if funct3 invalid
                        alu_op = 4'b0000; // NOP
                        $error("ERROR: Unrecognized funct3 (%b) for Custom Opcode %b at time %0t", funct3, opcode, $time);
                    end
                endcase
                end // END MARKED CHANGE for OPCODE_CUSTOM0


                default: begin
                    // Invalid opcode: set all controls to 0 (NOP-like behavior) to prevent unintended operations
                    $display("ERROR: Unrecognized Opcode: %b at time %0t", opcode, $time);
                end
            endcase
        //end

        // Debug prints for control unit outputs (can be removed for synthesis)
        $display("    Outputs: RegWriteEn=%b, MemToReg=%b, ALUOp=%b", reg_write_en, mem_to_reg, alu_op);
        $display("    Outputs: ALUSrcA=%b, ALUSrcB=%b, MemReadEn=%b, MemWriteEn=%b", alu_src_a, alu_src_b, mem_read_en, mem_write_en);
        $display("    Outputs: Branch=%b, Jump=%b, JALR=%b", branch, jump, jalr);
        $display("    -----------------------------------");
    end

endmodule
