// Arithmetic Logic Unit (ALU) for the RISC-V CPU.
// Performs standard RV32I arithmetic and logical operations,
// and custom operations for graph algorithms.

module alu (
    input wire [31:0] operand_a,
    input wire [31:0] operand_b,
    input wire [3:0]  alu_op,          // Control signal for ALU operation (from Control Unit)
    
    input wire [2:0]  funct3,          
    input wire [6:0]  funct7,         
    output reg [31:0] result,
    output wire       zero,            
    output wire       less             
);

    localparam ALU_ADD          = 4'b0000; // ADD, ADDI, JALR_TARGET, Load/Store Address Calculation, AUIPC
    localparam ALU_SUB          = 4'b0001; // SUB, Branch comparisons (RegA - RegB)
    localparam ALU_SLL          = 4'b0010; // SLL, SLLI
    localparam ALU_SLT          = 4'b0011; // SLT, SLTI (signed)
    localparam ALU_SLTU         = 4'b0100; // SLTU, SLTIU (unsigned)
    localparam ALU_XOR          = 4'b0101; // XOR, XORI
    localparam ALU_SRL          = 4'b0110; // SRL, SRLI (logical right shift)
    localparam ALU_SRA          = 4'b0111; // SRA, SRAI (arithmetic right shift)
    localparam ALU_OR           = 4'b1000; // OR, ORI
    localparam ALU_AND          = 4'b1001; // AND, ANDI
    localparam ALU_COPY_B       = 4'b1011; // For LUI (just copy immediate)

    // Custom ALU Operation Codes for Graph Algorithms
    localparam ALU_MIN_U        = 4'b1101; // Custom unsigned MIN operation
    localparam ALU_ABS_DIFF_U   = 4'b1110; // Custom unsigned ABSOLUTE DIFFERENCE operation

    
    // This always_comb block ensures the result updates immediately based on inputs.
    always @(*) begin
        case (alu_op)
            ALU_ADD:       result = operand_a + operand_b;
            ALU_SUB:       result = operand_a - operand_b;
            ALU_SLL:       result = operand_a << (operand_b & 32'h1F); // Shift amount is 5 bits (LSB of operand_b)
            ALU_SLT:       result = ($signed(operand_a) < $signed(operand_b)) ? 32'b1 : 32'b0;
            ALU_SLTU:      result = (operand_a < operand_b) ? 32'b1 : 32'b0;
            ALU_XOR:       result = operand_a ^ operand_b;
            ALU_SRL:       result = operand_a >> (operand_b & 32'h1F);
            ALU_SRA:       result = $signed(operand_a) >>> (operand_b & 32'h1F); // Arithmetic right shift (preserves sign bit)
            ALU_OR:        result = operand_a | operand_b;
            ALU_AND:       result = operand_a & operand_b;
            ALU_COPY_B:    result = operand_b; // Used for LUI where immediate is copied directly

            // Custom Instructions for Graph Algorithms
            ALU_MIN_U: begin
                // Unsigned minimum operation: selects the smaller of two unsigned operands
                result = (operand_a < operand_b) ? operand_a : operand_b;
            end
            ALU_ABS_DIFF_U: begin
                // Unsigned absolute difference operation: calculates |operand_a - operand_b|
                // This is crucial for distance calculations in graph algorithms (e.g., Manhattan distance)
                if (operand_a > operand_b) begin
                    result = operand_a - operand_b;
                end else begin
                    result = operand_b - operand_a;
                end
            end
            default: result = 32'b0; // Default case for unsupported/invalid ALU ops
        endcase
    end

    // Zero flag output: Set if the ALU result is zero.
    assign zero = (result == 32'b0);

    // Less flag output: Set if operand_a is less than operand_b (signed comparison).
    // This is used for signed branch conditions (BLT, BGE).
    assign less = ($signed(operand_a) < $signed(operand_b));

endmodule




// // changing ADIFF to a signed funciton 
// // In your port/register declarations, make sure the operands are signed for this operation
// // This is a conceptual change. The physical wires are the same.

// ALU_ABS_DIFF_S: begin // Note: I renamed from _U to _S for Signed
//     logic signed [31:0] diff;
    
//     // Perform a signed subtraction
//     diff = $signed(operand_a) - $signed(operand_b);

//     // Calculate the signed absolute value of the result
//     if (diff < 0) begin
//         result = -diff; // Two's complement negation
//     end else begin
//         result = diff;
//     end
// end