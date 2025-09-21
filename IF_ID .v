// IF/ID Pipeline Register.
// Stores the fetched instruction and PC+4 from IF stage and passes them to the ID stage.
// Includes logic for stalling (holding values) and flushing (inserting NOPs).

module if_id_reg (
    input wire clk,
    input wire rst_n, // Active low reset

    // Control signals from Hazard Unit
    input wire stall, // If high, register holds its current value (inputs are not loaded)
    input wire flush, // If high, register is cleared (NOP instruction effectively inserted)

    // Inputs from IF stage
    input wire [31:0] if_pc_in,
    input wire [31:0] if_pc_plus_4_in,
    input wire [31:0] if_instruction_in,

    // Outputs to ID stage (registered versions of inputs)
    output reg [31:0] id_pc_out,
    output reg [31:0] id_pc_plus_4_out,
    output reg [31:0] id_instruction_out
);

    // NOP instruction: ADDI x0, x0, 0 (opcode 0x13, rd=x0, rs1=x0, imm=0)
    localparam NOP_INSTRUCTION = 32'h00000013;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // On reset, clear outputs to a NOP state.
            id_pc_out          <= 32'b0;
            id_pc_plus_4_out   <= 32'b0;
            id_instruction_out <= NOP_INSTRUCTION; // Insert NOP on reset
        end else if (flush) begin
            // On flush (e.g., control hazard), insert a NOP.
            id_pc_out          <= 32'b0;
            id_pc_plus_4_out   <= 32'b0;
            id_instruction_out <= NOP_INSTRUCTION; // Insert NOP on flush
        end else if (!stall) begin
            // On clock edge, if not stalled, propagate inputs to outputs.
            id_pc_out          <= if_pc_in;
            id_pc_plus_4_out   <= if_pc_plus_4_in;
            id_instruction_out <= if_instruction_in;
        end
        // If `stall` is high and `flush` is low, the register holds its current values.
    end

endmodule