// Instruction Fetch (IF) Stage of the RISC-V CPU pipeline.
// Responsible for:
//  Holding the Program Counter (PC).
//  Calculating the next PC value (PC+4, branch target, jump target).
//  Fetching instructions from Instruction Memory.
//  Passing PC+4 and instruction to the IF/ID pipeline register.

module if_stage (
    input wire clk,
    input wire rst_n, // Active low reset

    
    output wire [31:0] pc_out,

    // Instruction memory interface
    output wire [31:0] instruction_address, // Address to instruction memory
    input wire  [31:0] instruction_read_data, // Instruction fetched from instruction memory

    // Outputs to IF/ID pipeline register
    output wire [31:0] instruction_out,
    output wire [31:0] pc_plus_4_out,

    // Control signals from hazard unit
    input wire        stall,        
    input wire        flush,        

    // Branch/Jump feedback from EX stage (for PC update)
    input wire        branch_taken_ex,  // Indicates if a branch in EX stage is taken
    input wire [31:0] branch_target_ex, // Target address for taken branch
    input wire        jump_taken_ex     // Indicates if a JAL/JALR in ID/EX stage is taken
);

    // Internal PC register
    reg [31:0] program_counter;

    // Assign outputs for instruction memory and IF/ID register
    assign instruction_address = program_counter;
    assign instruction_out     = instruction_read_data; // Passed directly from IM
    assign pc_plus_4_out       = program_counter + 32'd4; // Calculate PC+4
    assign pc_out              = program_counter; // Output current PC

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            program_counter <= 32'h00000000;
        end else begin
            if (!stall) begin 
                if (branch_taken_ex) begin
                    program_counter <= branch_target_ex;
                end else if (jump_taken_ex) begin
                    program_counter <= branch_target_ex;
                end else begin
                    program_counter <= program_counter + 32'd4;
                end
            end
        end
    end

endmodule
