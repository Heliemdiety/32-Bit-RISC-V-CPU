// Responsible for:
//  Selecting ALU operands (with forwarding logic).
//  Performing the ALU operation.
//  Calculating branch/jump targets.
// Determining if a branch is taken.
// Passing results and control signals to the EX/MEM pipeline register.

module ex_stage (
    input wire clk,
    input wire rst_n, // Active low reset ( this is important because i was not maintaining it in everymodule)

    // Inputs from ID/EX pipeline register
    input wire [31:0] pc_plus_4_in,
    input wire [31:0] reg_read_data1_in,
    input wire [31:0] reg_read_data2_in,
    input wire [31:0] immediate_in,
    input wire [4:0]  rs1_addr_in, // For hazard unit / forwarding
    input wire [4:0]  rs2_addr_in, // For hazard unit / forwarding
    input wire [4:0]  rd_addr_in,  // For hazard unit
    input wire        reg_write_en_in, // For hazard unit
    input wire        mem_read_en_in,  // For hazard unit (load-use)
    input wire        mem_write_en_in,
    input wire [1:0]  mem_to_reg_in,   // For hazard unit (load-use)
    input wire [3:0]  alu_op_in,
    input wire [1:0]  alu_src_a_in,    // 00: pc_plus_4, 01: reg_read_data1
    input wire [1:0]  alu_src_b_in,    // 00: reg_read_data2, 01: immediate
    input wire        branch_in,      
    input wire        jump_in,        
    input wire        jalr_in,        
    input wire [2:0]  funct3_in,       // For branch type and for load/store size
    input wire [6:0]  funct7_in,       // For R-type special ops (e.g., SUB/SRA)

    // Forwarding control from Hazard Unit
    input wire [1:0]  forward_a,   // 00: No forward (reg_data1_in), 01: EX/MEM.ALU_Result, 10: MEM/WB.Result
    input wire [1:0]  forward_b,   // 00: No forward (reg_data2_in), 01: EX/MEM.ALU_Result, 10: MEM/WB.Result
    // Forwarded data from later stages
    input wire [31:0] forward_ex_alu_result,      // ALU result from EX/MEM stage
    input wire [31:0] forward_mem_load_data, // Load data from MEM/WB stage
    input wire [31:0] forward_mem_alu_result, // ALU result from MEM/WB stage

    // Outputs to EX/MEM pipeline register
    output wire [31:0] alu_result_out,
    output wire [31:0] reg_read_data2_out, // Value to be stored in memory
    output wire        branch_taken_out,   // Signal if branch is actually taken (for PC update)
    output wire [31:0] branch_target_out,   // Calculated branch target
    // Pass through for EX/MEM register (signals from ID/EX register)
    output wire [4:0]  rd_addr_out,
    output wire        reg_write_en_out,
    output wire [1:0]  mem_to_reg_out,
    output wire [2:0]  funct3_out,
    output wire [1:0]  alu_src_a_out,
    output wire [1:0]  alu_src_b_out,
    output wire [3:0]  alu_op_out,
    output wire        mem_read_en_out,
    output wire        mem_write_en_out // Correctly declared as output wire
);

    
    wire [31:0] alu_operand_a;
    wire [31:0] alu_operand_b;

    // ALU Operand A (reg_read_data1_in or pc_plus_4_in or forwarded data)
    assign alu_operand_a = (forward_a == 2'b00) ? (alu_src_a_in == 2'b00 ? pc_plus_4_in : reg_read_data1_in) :
                           (forward_a == 2'b01) ? forward_ex_alu_result :
                           (forward_a == 2'b10) ? (mem_to_reg_in == 2'b01 ? forward_mem_load_data : forward_mem_alu_result) :
                           32'b0; // Should not happen

    // ALU Operand B (reg_read_data2_in or immediate_in or forwarded data)
    assign alu_operand_b = (forward_b == 2'b00) ? (alu_src_b_in == 2'b00 ? reg_read_data2_in : immediate_in) :
                           (forward_b == 2'b01) ? forward_ex_alu_result :
                           (forward_b == 2'b10) ? (mem_to_reg_in == 2'b01 ? forward_mem_load_data : forward_mem_alu_result) :
                           32'b0; // Should not happen


    // Passing reg_read_data2_in through for store instructions, with forwarding.
    // This  goes to the EX/MEM register and then to the MEM stage.
    wire [31:0] data_to_store_forwarded;
    assign data_to_store_forwarded = (forward_b == 2'b00) ? reg_read_data2_in :
                                     (forward_b == 2'b01) ? forward_ex_alu_result :
                                     (forward_b == 2'b10) ? (mem_to_reg_in == 2'b01 ? forward_mem_load_data : forward_mem_alu_result) :
                                     32'b0;
    assign reg_read_data2_out = data_to_store_forwarded;


   
    wire [31:0] alu_result_internal; // Internal wire for ALU output
    wire        zero_flag;          
    wire        less_flag;           // Set if ALU result is less 

    alu alu_inst (
        .operand_a (alu_operand_a),
        .operand_b (alu_operand_b),
        .alu_op    (alu_op_in),
        .funct3    (funct3_in), // Used for shifts, SLT/SLTU etc.
        .funct7    (funct7_in), // Used for SUB/SRA
        .result    (alu_result_internal),
        .zero      (zero_flag),
        .less      (less_flag)
    );

    assign alu_result_out = alu_result_internal; // Output the ALU result


    wire branch_condition_met;

    // Determine if branch condition is met based on funct3
    assign branch_condition_met = (funct3_in == 3'b000) ? zero_flag :   // BEQ (Equal)
                                  (funct3_in == 3'b001) ? ~zero_flag :  // BNE (Not Equal)
                                  (funct3_in == 3'b100) ? less_flag :   // BLT (Signed Less Than)
                                  (funct3_in == 3'b101) ? ~less_flag :  // BGE (Signed Greater Than or Equal)
                                  (funct3_in == 3'b110) ? less_flag :   // BLTU (Unsigned Less Than)
                                  (funct3_in == 3'b111) ? ~less_flag :  // BGEU (Unsigned Greater Than or Equal)
                                  1'b0; // Default

    // Calculate branch/jump target address
    assign branch_target_out = (jalr_in) ? alu_result_internal : // For JALR, ALU already computed target
                               (jump_in || branch_in) ? (pc_plus_4_in + immediate_in) : // For JAL/Branch, PC + Imm
                               32'b0; // Default (should not be used unless jump/branch)

    // Determine if the branch is taken
    assign branch_taken_out = branch_in && branch_condition_met;

    
    // Outputs passed directly to EX/MEM register
    assign rd_addr_out      = rd_addr_in;
    assign reg_write_en_out = reg_write_en_in;
    assign mem_to_reg_out   = mem_to_reg_in;
    assign funct3_out       = funct3_in; // Passed directly from ID/EX
    assign alu_src_a_out    = alu_src_a_in;
    assign alu_src_b_out    = alu_src_b_in;
    assign alu_op_out       = alu_op_in;
    assign mem_read_en_out  = mem_read_en_in;
    assign mem_write_en_out = mem_write_en_in; // This is the line that caused the error many times in my code,, i was not assigning output equal to input. 
    // Removed: pc_plus_4_out as it's directly passed from id_ex_reg to ex_mem_reg in riscv_cpu_top
endmodule
