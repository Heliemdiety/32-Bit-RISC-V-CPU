// Instruction Fetch (IF) Stage of the RISC-V Pipeline.
// Responsible for generating the next PC and fetching instructions from memory.

module if_stage (
    input wire clk,
    input wire rst_n, // Active low reset

    output wire [31:0] pc_out,          // Current PC value (for debug and IF/ID register)

    // Instruction memory interface
    output wire [31:0] instruction_address,   // Address to instruction memory
    input wire  [31:0] instruction_read_data, // Instruction fetched from instruction memory

    // Outputs to IF/ID pipeline register
    output wire [31:0] instruction_out, // Instruction fetched
    output wire [31:0] pc_plus_4_out,   // PC + 4 (next sequential instruction address)

    // Control signals from hazard unit
    input wire         stall,           // If high, PC holds its current value
    input wire         flush,           // If high, IF/ID register is flushed (implicitly handled by if_id_reg)

    // Branch/Jump feedback from EX stage (for PC update)
    input wire         branch_taken_ex,    // Indicates if a branch in EX stage is taken
    input wire [31:0]  branch_target_ex,   // Target address for taken branch/jump
    input wire         jump_taken_ex       // Indicates if a JAL/JALR in EX stage (or earlier in pipeline, effectively) is taken
);

    // Internal PC register
    reg [31:0] program_counter;

    // Assign outputs for instruction memory and IF/ID register
    assign instruction_address = program_counter;
    assign instruction_out     = instruction_read_data; // Pass fetched instruction to IF/ID reg
    assign pc_plus_4_out       = program_counter + 32'd4; // Calculate next sequential PC
    assign pc_out              = program_counter; // Output current PC for debugging or pipeline state observation

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // On reset, PC is set to the starting address (usually 0x0).
            program_counter <= 32'h00000000;
        end else begin
            // If the IF stage is not stalled, update PC.
            // If `stall` is high, PC holds its value, effectively stalling fetch.
            if (!stall) begin
                // Check if a branch or jump is taken (from EX stage)
                if (branch_taken_ex) begin
                    // For conditional branches, if taken, update PC to branch target.
                    program_counter <= branch_target_ex;
                end else if (jump_taken_ex) begin
                    // For unconditional jumps (JAL/JALR), update PC to jump target.
                    program_counter <= branch_target_ex;
                end else begin
                    // Default: increment PC by 4 for the next sequential instruction.
                    program_counter <= program_counter + 32'd4;
                end
            end
            // If `stall` is high, program_counter retains its value.
            // `flush` control signal affects the IF/ID register, not the PC directly.
        end
    end

endmodule
