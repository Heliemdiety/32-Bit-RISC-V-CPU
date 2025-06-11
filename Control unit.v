module control_unit (
    input wire [6:0] opcode,
    input wire [2:0] funct3,
    input wire [6:0] funct7,

    // Outputs for pipeline control
    output reg        reg_write_en, // Register Write Enable
    output reg [1:0]  mem_to_reg,   // Mux select for WB stage (00: ALU_result, 01: Load_data, 10: PC+4)
    output reg [3:0]  alu_op,       // ALU operation code 
    output reg [1:0]  alu_src_a,    // Mux select for ALU operand A (00: PC, 01: RegData1)
    output reg [1:0]  alu_src_b,    // Mux select for ALU operand B (00: RegData2, 01: Immediate)
    output reg        mem_read_en,  // Memory Read Enable
    output reg        mem_write_en, // Memory Write Enable
    output reg        branch,       
    output reg        jump,         
    output reg        jalr         
);

    // ALU Operation Codes
    localparam ALU_ADD         = 4'b0000;
    localparam ALU_SUB         = 4'b0001;
    localparam ALU_SLL         = 4'b0010;
    localparam ALU_SLT         = 4'b0011;
    localparam ALU_SLTU        = 4'b0100;
    localparam ALU_XOR         = 4'b0101;
    localparam ALU_SRL         = 4'b0110;
    localparam ALU_SRA         = 4'b0111;
    localparam ALU_OR          = 4'b1000;
    localparam ALU_AND         = 4'b1001;
    localparam ALU_JALR_TARGET = 4'b1010; // For JALR (base address + immediate)
    localparam ALU_COPY_B      = 4'b1011; // For LUI (just copy immediate)

    always @(*) begin
        // Default values for all outputs to prevent 'X' propagation
        reg_write_en   = 1'b0;
        mem_to_reg     = 2'b00; // Default: ALU result writes to Reg
        alu_op         = ALU_ADD; // Default: ADD 
        alu_src_a      = 2'b00; // Default: PC
        alu_src_b      = 2'b00; // Default: RegData2
        mem_read_en    = 1'b0;
        mem_write_en   = 1'b0;
        branch         = 1'b0;
        jump           = 1'b0;
        jalr           = 1'b0;

        // Debug prints for control unit inputs
        $display("      --- CONTROL_UNIT Internal Debug (Time %0t) ---", $time);
        $display("      Inputs: opcode=%b, funct3=%b, funct7=%b",
                 opcode, funct3, funct7);

        case (opcode)
            // R-Type (reg-reg arithmetic/logical) - 0110011 (0x33)
            7'b0110011: begin
                reg_write_en = 1'b1;
                mem_to_reg   = 2'b00; // ALU result
                alu_src_a    = 2'b01; // RegData1
                alu_src_b    = 2'b00; // RegData2
                case ({funct7, funct3})
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
                        $display("ERROR: Unrecognized R-Type funct3/funct7: %b/%b", funct3, funct7);
                        // Control signals remain at default (e.g., NOP-like)
                    end
                endcase
            end

            // I-Type (Immediate arithmetic/logical) - 0010011 (0x13)
            7'b0010011: begin
                reg_write_en = 1'b1;
                mem_to_reg   = 2'b00; // ALU result
                alu_src_a    = 2'b01; // RegData1
                alu_src_b    = 2'b01; // Immediate
                case (funct3)
                    3'b000: alu_op = ALU_ADD;   // ADDI
                    3'b001: alu_op = ALU_SLL;   // SLLI (funct7 must be 0)
                    3'b010: alu_op = ALU_SLT;   // SLTI
                    3'b011: alu_op = ALU_SLTU;  // SLTIU
                    3'b100: alu_op = ALU_XOR;   // XORI
                    3'b101: begin // SRLI / SRAI
                        if (funct7 == 7'b0000000) alu_op = ALU_SRL; // SRLI
                        else if (funct7 == 7'b0100000) alu_op = ALU_SRA; // SRAI
                        else alu_op = ALU_ADD; // Invalid
                    end
                    3'b110: alu_op = ALU_OR;    // ORI
                    3'b111: alu_op = ALU_AND;   // ANDI
                    default: begin
                        $display("ERROR: Unrecognized I-Type funct3: %b", funct3);
                    end
                endcase
            end

            // Load instructions - 0000011 (0x03)
            7'b0000011: begin
                reg_write_en = 1'b1;
                mem_to_reg   = 2'b01; // Load data from memory
                alu_op       = ALU_ADD; // Calculate address: RS1 + Immediate
                alu_src_a    = 2'b01; // RegData1
                alu_src_b    = 2'b01; // Immediate
                mem_read_en  = 1'b1;
                // funct3 dictates load type (LB, LH, LW, LBU, LHU)
            end

            // S-Type (Store instructions) - 0100011 (0x23)
            7'b0100011: begin
                reg_write_en = 1'b0; 
                mem_to_reg   = 2'b00; 
                alu_op       = ALU_ADD; // Calculate address: RS1 + Immediate
                alu_src_a    = 2'b01; // RegData1
                alu_src_b    = 2'b01; // Immediate
                mem_write_en = 1'b1;
                // funct3 dictates store type (SB, SH, SW)
            end

            // B-Type (Branch instructions) - 1100011 (0x63)
            7'b1100011: begin
                reg_write_en = 1'b0; // Branches don't write to register file
                mem_to_reg   = 2'b00; 
                alu_op       = ALU_SUB; 
                alu_src_a    = 2'b01; // RegData1
                alu_src_b    = 2'b00; // RegData2 (for comparison)
                branch       = 1'b1;
                // funct3 dictates branch type (BEQ, BNE, BLT, BGE, BLTU, BGEU)
            end

            // U-Type (LUI, AUIPC)
            // LUI (Load Upper Immediate) - 0110111 (0x37)
            7'b0110111: begin // LUI
                reg_write_en = 1'b1;
                mem_to_reg   = 2'b00; // ALU result
                alu_op       = ALU_COPY_B; // Just copy immediate
                alu_src_a    = 2'b00; 
                alu_src_b    = 2'b01; // Immediate is operand B
            end
            // AUIPC (Add Upper Immediate to PC) - 0010111 (0x17)
            7'b0010111: begin // AUIPC
                reg_write_en = 1'b1;
                mem_to_reg   = 2'b00; // ALU result
                alu_op       = ALU_ADD; // PC + Immediate
                alu_src_a    = 2'b00; // PC is operand A
                alu_src_b    = 2'b01; // Immediate is operand B
            end

            // J-Type (JAL) - 1101111 (0x6F)
            7'b1101111: begin // JAL
                reg_write_en = 1'b1;
                mem_to_reg   = 2'b10; // Write PC+4 to Rd
                alu_op       = ALU_ADD; // ALU calculates target (PC + Immediate)
                alu_src_a    = 2'b00; // PC is operand A
                alu_src_b    = 2'b01; // Immediate is operand B
                jump         = 1'b1; // Signal for PC update
            end

            // I-Type (JALR) - 1100111 (0x67)
            7'b1100111: begin // JALR
                reg_write_en = 1'b1;
                mem_to_reg   = 2'b10; // Write PC+4 to Rd
                alu_op       = ALU_JALR_TARGET; // ALU calculates target (RS1 + Immediate)
                alu_src_a    = 2'b01; // RegData1 is operand A
                alu_src_b    = 2'b01; // Immediate is operand B
                jalr         = 1'b1; // Signal for PC update
            end

            // ECALL / EBREAK (System calls) - 1110011 (0x73)
            7'b1110011: begin
                // No register write, no memory access for these simplified instructions
                reg_write_en = 1'b0;
                mem_read_en  = 1'b0;
                mem_write_en = 1'b0;
                
            end

            default: begin
                // Invalid opcode: set all controls to 0 to prevent unintended operations
                reg_write_en = 1'b0;
                mem_to_reg   = 2'b00;
                alu_op       = ALU_ADD; // Default to ADD for safety, or NOP (0000)
                alu_src_a    = 2'b00;
                alu_src_b    = 2'b00;
                mem_read_en  = 1'b0;
                mem_write_en = 1'b0;
                branch       = 1'b0;
                jump         = 1'b0;
                jalr         = 1'b0;
                $display("ERROR: Unrecognized Opcode: %b at time %0t", opcode, $time);
            end
        endcase
        // Debug prints for control unit outputs
        $display("      Outputs: RegWriteEn=%b, MemToReg=%b, ALUOp=%b",
                 reg_write_en, mem_to_reg, alu_op);
        $display("      Outputs: ALUSrcA=%b, ALUSrcB=%b, MemReadEn=%b, MemWriteEn=%b",
                 alu_src_a, alu_src_b, mem_read_en, mem_write_en);
        $display("      Outputs: Branch=%b, Jump=%b, JALR=%b",
                 branch, jump, jalr);
        $display("      -----------------------------------");
    end

endmodule

