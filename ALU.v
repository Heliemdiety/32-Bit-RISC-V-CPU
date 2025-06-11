// Arithmetic Logic Unit (ALU) for the RISC-V CPU.
// Performs standard RV32I arithmetic and logical operations.

module alu (
    input wire [31:0] operand_a,
    input wire [31:0] operand_b,
    input wire [3:0]  alu_op,    // Control signal for ALU operation (from Control Unit)
    input wire [2:0]  funct3,    // Used for shifts, SLT/SLTU etc.
    input wire [6:0]  funct7,    // Used for SUB/SRA
    output reg [31:0] result,
    output wire       zero,      // Set if result is zero (for branch conditions)
    output wire       less       // Set if result is less than zero (for SLT, BLT)
);

    // ALU Operation Codes (Example Mappings, adjust as per control_unit.v)
    localparam ALU_ADD   = 4'b0000; // ADD, ADDI
    localparam ALU_SUB   = 4'b0001; // SUB
    localparam ALU_SLL   = 4'b0010; // SLL, SLLI
    localparam ALU_SLT   = 4'b0011; // SLT, SLTI (signed)
    localparam ALU_SLTU  = 4'b0100; // SLTU, SLTIU (unsigned)
    localparam ALU_XOR   = 4'b0101; // XOR, XORI
    localparam ALU_SRL   = 4'b0110; // SRL, SRLI (logical right shift)
    localparam ALU_SRA   = 4'b0111; // SRA, SRAI (arithmetic right shift)
    localparam ALU_OR    = 4'b1000; // OR, ORI
    localparam ALU_AND   = 4'b1001; // AND, ANDI
    localparam ALU_JALR  = 4'b1010; // For JALR (base address + immediate)
    localparam ALU_COPY_B = 4'b1011; // For LUI (just copy immediate)
    localparam ALU_AUIPC_ADD = 4'b1100; // For AUIPC (PC + immediate)

    // Combinational logic for ALU operations
    always @(*) begin
        case (alu_op)
            ALU_ADD: result = operand_a + operand_b;
            ALU_SUB: result = operand_a - operand_b;
            ALU_SLL: result = operand_a << (operand_b & 32'h1F); // Shift amount is 5 bits
            ALU_SLT: result = ($signed(operand_a) < $signed(operand_b)) ? 32'b1 : 32'b0;
            ALU_SLTU: result = (operand_a < operand_b) ? 32'b1 : 32'b0;
            ALU_XOR: result = operand_a ^ operand_b;
            ALU_SRL: result = operand_a >> (operand_b & 32'h1F);
            ALU_SRA: result = $signed(operand_a) >>> (operand_b & 32'h1F); // Arithmetic right shift
            ALU_OR:  result = operand_a | operand_b;
            ALU_AND: result = operand_a & operand_b;
            ALU_JALR: result = operand_a + operand_b; // JALR calculates target (base + imm)
            ALU_COPY_B: result = operand_b;           // For LUI (imm is operand_b)
            ALU_AUIPC_ADD: result = operand_a + operand_b; // For AUIPC (PC + imm)
            default: result = 32'b0; // Default case
        endcase
    end

    // Zero flag output
    assign zero = (result == 32'b0);

    // Less flag output (for signed comparisons).
    // For SLT/SLTI, the ALU directly computes 1 or 0.
    assign less = ($signed(operand_a) < $signed(operand_b));

endmodule
