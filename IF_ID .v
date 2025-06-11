// IF/ID Pipeline Register.
// Stores the instruction and PC+4 from the IF stage and passes them to the ID stage.
// Can be stalled or flushed by the Hazard Unit.

module if_id_reg (
    input wire clk,
    input wire rst_n, // Active low reset

    // Control signals from Hazard Unit
    input wire stall, // If high, register holds its current value (PC and Instruction don't update)
    input wire flush, // If high, register is cleared (NOP instruction inserted)

    // Inputs from IF stage
    input wire [31:0] if_pc_in,
    input wire [31:0] if_pc_plus_4_in,
    input wire [31:0] if_instruction_in,

    // Outputs to ID stage
    output reg [31:0] id_pc_out,
    output reg [31:0] id_pc_plus_4_out,
    output reg [31:0] id_instruction_out
);

    // NOP instruction for flushing
    localparam NOP_INSTRUCTION = 32'h00000013;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_pc_out          <= 32'b0;
            id_pc_plus_4_out   <= 32'b0;
            id_instruction_out <= NOP_INSTRUCTION; // On reset, insert NOP
        end else if (flush) begin
            id_pc_out          <= 32'b0;
            id_pc_plus_4_out   <= 32'b0;
            id_instruction_out <= NOP_INSTRUCTION; // On flush, insert NOP
        end else if (!stall) begin 
            id_pc_out          <= if_pc_in;
            id_pc_plus_4_out   <= if_pc_plus_4_in;
            id_instruction_out <= if_instruction_in;
        end
    
    end

endmodule
