// Top-level module for the 5-stage pipelined RISC-V CPU.
// Integrates all pipeline stages, memories, hazard unit, and a dedicated WB stage.

module riscv_cpu_top (
    input wire clk,    // Clock signal
    input wire rst_n, // Active low reset

    // Data Memory Interface (to external data_memory module)
    output wire [31:0] mem_addr,
    output wire [31:0] mem_write_data,
    output wire        mem_write_en,
    output wire        mem_read_en,
    input wire  [31:0] mem_read_data,
    output wire [3:0]  mem_byte_enable,

    // Instruction Memory Interface (to instruction_memory module)
    output wire [31:0] imem_addr,
    input wire  [31:0] imem_read_data // Instruction fetched by IF stage
);

    // --- Wires connecting pipeline stages and units ---

    // Wires for custom instruction flags propagating through the pipeline
    wire is_min_u_id_stage_to_id_ex_reg;
    wire is_abs_diff_u_id_stage_to_id_ex_reg;
    wire is_min_u_id_ex_reg_to_ex_stage;
    wire is_abs_diff_u_id_ex_reg_to_ex_stage;

    // IF_STAGE Outputs to IF_ID_REG Inputs
    wire [31:0] if_pc_out;           // Current PC value from IF stage
    wire [31:0] if_pc_plus_4_out;    // PC+4 from IF stage
    wire [31:0] if_instruction_out;  // Fetched instruction from IF stage

    // IF_ID_REG Outputs to ID_STAGE Inputs
    wire [31:0] id_pc_out_from_if_id;
    wire [31:0] id_pc_plus_4_out_from_if_id;
    wire [31:0] id_instruction_out_from_if_id;

    // ID_STAGE Outputs to ID_EX_REG Inputs (and some direct connections to RegFile)
    wire [31:0] id_stage_to_id_ex_reg_immediate;
    wire [4:0]  id_stage_to_id_ex_reg_rs1_addr;
    wire [4:0]  id_stage_to_id_ex_reg_rs2_addr;
    wire [4:0]  id_stage_to_id_ex_reg_rd_addr;
    wire        id_stage_to_id_ex_reg_write_en;
    wire [1:0]  id_stage_to_id_ex_reg_mem_to_reg;
    wire [3:0]  id_stage_to_id_ex_reg_alu_op;
    wire [1:0]  id_stage_to_id_ex_reg_alu_src_a;
    wire [1:0]  id_stage_to_id_ex_reg_alu_src_b;
    wire        id_stage_to_id_ex_reg_mem_read_en;
    wire        id_stage_to_id_ex_reg_mem_write_en;
    wire        id_stage_to_id_ex_reg_branch;
    wire        id_stage_to_id_ex_reg_jump;
    wire        id_stage_to_id_ex_reg_jalr;
    wire [2:0]  id_stage_to_id_ex_reg_funct3;
    wire [6:0]  id_stage_to_id_ex_reg_funct7;

    // ID_EX_REG Outputs to EX_STAGE Inputs
    wire [31:0] id_ex_pc_plus_4_out;
    wire [31:0] id_ex_reg_read_data1_out;
    wire [31:0] id_ex_reg_read_data2_out;
    wire [31:0] id_ex_immediate_out;
    wire [4:0]  id_ex_rs1_addr_out;
    wire [4:0]  id_ex_rs2_addr_out;
    wire [4:0]  id_ex_rd_addr_out;
    wire        id_ex_reg_write_en_out;
    wire [1:0]  id_ex_mem_to_reg_out;
    wire [3:0]  id_ex_alu_op_out;
    wire [1:0]  id_ex_alu_src_a_out;
    wire [1:0]  id_ex_alu_src_b_out;
    wire        id_ex_mem_read_en_out;
    wire        id_ex_mem_write_en_out;
    wire        id_ex_branch_out;
    wire        id_ex_jump_out;
    wire        id_ex_jalr_out;
    wire [2:0]  id_ex_funct3_out;
    wire [6:0]  id_ex_funct7_out;

    // EX_STAGE Outputs to EX_MEM_REG Inputs
    wire [31:0] ex_stage_to_ex_mem_reg_pc_plus_4;
    wire [31:0] ex_stage_to_ex_mem_reg_alu_result;
    wire [31:0] ex_stage_to_ex_mem_reg_reg_read_data2;
    wire        ex_stage_to_ex_mem_reg_branch_taken;
    wire [31:0] ex_stage_to_ex_mem_reg_branch_target;
    wire [4:0]  ex_stage_to_ex_mem_reg_rd_addr;
    wire        ex_stage_to_ex_mem_reg_reg_write_en;
    wire [1:0]  ex_stage_to_ex_mem_reg_mem_to_reg;
    wire [2:0]  ex_stage_to_ex_mem_reg_funct3;
    wire [3:0]  ex_stage_to_ex_mem_reg_alu_op;
    wire        ex_stage_to_ex_mem_reg_mem_read_en;
    wire        ex_stage_to_ex_mem_reg_mem_write_en;

    // EX_MEM_REG Outputs to MEM_STAGE Inputs
    wire [31:0] ex_mem_pc_plus_4_out;
    wire [31:0] ex_mem_alu_result_out;
    wire [31:0] ex_mem_reg_read_data2_out;
    wire [4:0]  ex_mem_rd_addr_out;
    wire        ex_mem_reg_write_en_out;
    wire [1:0]  ex_mem_mem_to_reg_out;
    wire        ex_mem_mem_read_en_out;
    wire        ex_mem_mem_write_en_out;
    wire [2:0]  ex_mem_funct3_out;

    // MEM_STAGE Outputs to MEM_WB_REG Inputs
    wire [31:0] mem_stage_to_mem_wb_reg_load_data; // From MEM_STAGE (after extension)

    // MEM_WB_REG Outputs to WB_STAGE Inputs
    wire [31:0] mem_wb_alu_result_out;
    wire [31:0] mem_wb_load_data_out;
    wire [31:0] mem_wb_pc_plus_4_out;
    wire [4:0]  mem_wb_rd_addr_out;
    wire        mem_wb_reg_write_en_out;
    wire [1:0]  mem_wb_mem_to_reg_out;

    // WB_STAGE Outputs to REG_FILE Write Port
    wire [31:0] wb_stage_write_data;   // Data to be written to register file
    wire [4:0]  wb_stage_write_addr;   // Address to write to in register file
    wire        wb_stage_write_en;      // Enable signal for register file write


    // Register File Read Data (from RegFile to ID stage)
    wire [31:0] reg_file_read_data1;
    wire [31:0] reg_file_read_data2;

    // Hazard Unit Outputs to Pipeline Stage Control Inputs
    wire        stall_if;    // Stall Instruction Fetch stage
    wire        stall_id;    // Stall Instruction Decode stage
    wire        flush_id;    // Flush ID/EX register
    wire [1:0]  forward_a;   // Forwarding control for ALU operand A (rs1)
    wire [1:0]  forward_b;   // Forwarding control for ALU operand B (rs2)

    // Debug wire for opcode from ID stage
    wire [6:0] id_stage_opcode_debug_wire;


    
    // Stage Instantiations
    
    // 1. Instruction Fetch (IF) Stage
    if_stage if_stage_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .pc_out                (if_pc_out),
        .instruction_address   (imem_addr),
        .instruction_read_data (imem_read_data),
        .instruction_out       (if_instruction_out),
        .pc_plus_4_out         (if_pc_plus_4_out),
        .stall                 (stall_if),
        .flush                 (flush_id),
        .branch_taken_ex       (ex_stage_to_ex_mem_reg_branch_taken),
        .branch_target_ex      (ex_stage_to_ex_mem_reg_branch_target),
        .jump_taken_ex         (id_ex_jump_out | id_ex_jalr_out)
    );

    // 2. IF/ID Pipeline Register
    if_id_reg if_id_reg_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .stall                 (stall_if),
        .flush                 (flush_id),
        .if_pc_in              (if_pc_out),
        .if_pc_plus_4_in       (if_pc_plus_4_out),
        .if_instruction_in     (if_instruction_out),
        .id_pc_out             (id_pc_out_from_if_id),
        .id_pc_plus_4_out      (id_pc_plus_4_out_from_if_id),
        .id_instruction_out    (id_instruction_out_from_if_id)
    );

    // 3. Instruction Decode (ID) Stage
    id_stage id_stage_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .instruction_in        (id_instruction_out_from_if_id),
        .pc_curr_if            (id_pc_out_from_if_id),

        .reg_file_read_addr1   (id_stage_to_id_ex_reg_rs1_addr),
        .reg_file_read_addr2   (id_stage_to_id_ex_reg_rs2_addr),
        .reg_file_read_data1   (reg_file_read_data1),
        .reg_file_read_data2   (reg_file_read_data2),

        .immediate_out         (id_stage_to_id_ex_reg_immediate),
        .rs1_addr_out          (id_stage_to_id_ex_reg_rs1_addr),
        .rs2_addr_out          (id_stage_to_id_ex_reg_rs2_addr),
        .rd_addr_out           (id_stage_to_id_ex_reg_rd_addr),
        .reg_write_en_out      (id_stage_to_id_ex_reg_write_en),
        .mem_to_reg_out        (id_stage_to_id_ex_reg_mem_to_reg),
        .alu_op_out            (id_stage_to_id_ex_reg_alu_op),
        .alu_src_a_out         (id_stage_to_id_ex_reg_alu_src_a),
        .alu_src_b_out         (id_stage_to_id_ex_reg_alu_src_b),
        .mem_read_en_out       (id_stage_to_id_ex_reg_mem_read_en),
        .mem_write_en_out      (id_stage_to_id_ex_reg_mem_write_en),
        .branch_out            (id_stage_to_id_ex_reg_branch),
        .jump_out              (id_stage_to_id_ex_reg_jump),
        .jalr_out              (id_stage_to_id_ex_reg_jalr),
        .funct3_out            (id_stage_to_id_ex_reg_funct3),
        .funct7_out            (id_stage_to_id_ex_reg_funct7),

        .is_min_u_instr_out      (is_min_u_id_stage_to_id_ex_reg),
        .is_abs_diff_u_instr_out (is_abs_diff_u_id_stage_to_id_ex_reg),
        .opcode_debug          (id_stage_opcode_debug_wire)
    );

    // 4. ID/EX Pipeline Register
    id_ex_reg id_ex_reg_inst (
        .clk                  (clk),
        .rst_n                (rst_n),
        .stall                (stall_id),
        .flush                (flush_id),

        .id_pc_plus_4_in      (id_pc_plus_4_out_from_if_id),
        .id_reg_read_data1_in (reg_file_read_data1),
        .id_reg_read_data2_in (reg_file_read_data2),
        .id_immediate_in      (id_stage_to_id_ex_reg_immediate),
        .id_rs1_addr_in       (id_stage_to_id_ex_reg_rs1_addr),
        .id_rs2_addr_in       (id_stage_to_id_ex_reg_rs2_addr),
        .id_rd_addr_in        (id_stage_to_id_ex_reg_rd_addr),
        .id_reg_write_en_in   (id_stage_to_id_ex_reg_write_en),
        .id_mem_to_reg_in     (id_stage_to_id_ex_reg_mem_to_reg),
        .id_alu_op_in         (id_stage_to_id_ex_reg_alu_op),
        .id_alu_src_a_in      (id_stage_to_id_ex_reg_alu_src_a),
        .id_alu_src_b_in      (id_stage_to_id_ex_reg_alu_src_b),
        .id_mem_read_en_in    (id_stage_to_id_ex_reg_mem_read_en),
        .id_mem_write_en_in   (id_stage_to_id_ex_reg_mem_write_en),
        .id_branch_in         (id_stage_to_id_ex_reg_branch),
        .id_jump_in           (id_stage_to_id_ex_reg_jump),
        .id_jalr_in           (id_stage_to_id_ex_reg_jalr),
        .id_funct3_in         (id_stage_to_id_ex_reg_funct3),
        .id_funct7_in         (id_stage_to_id_ex_reg_funct7),
        .id_is_min_u_in       (is_min_u_id_stage_to_id_ex_reg),
        .id_is_abs_diff_u_in  (is_abs_diff_u_id_stage_to_id_ex_reg),

        .ex_pc_plus_4_out     (id_ex_pc_plus_4_out),
        .ex_reg_read_data1_out (id_ex_reg_read_data1_out),
        .ex_reg_read_data2_out (id_ex_reg_read_data2_out),
        .ex_immediate_out     (id_ex_immediate_out),
        .ex_rs1_addr_out      (id_ex_rs1_addr_out),
        .ex_rs2_addr_out      (id_ex_rs2_addr_out),
        .ex_rd_addr_out       (id_ex_rd_addr_out),
        .ex_reg_write_en_out  (id_ex_reg_write_en_out),
        .ex_mem_to_reg_out    (id_ex_mem_to_reg_out),
        .ex_alu_op_out        (id_ex_alu_op_out),
        .ex_alu_src_a_out     (id_ex_alu_src_a_out),
        .ex_alu_src_b_out     (id_ex_alu_src_b_out),
        .ex_mem_read_en_out   (id_ex_mem_read_en_out),
        .ex_mem_write_en_out  (id_ex_mem_write_en_out),
        .ex_branch_out        (id_ex_branch_out),
        .ex_jump_out          (id_ex_jump_out),
        .ex_jalr_out          (id_ex_jalr_out),
        .ex_funct3_out        (id_ex_funct3_out),
        .ex_funct7_out        (id_ex_funct7_out),
        .ex_is_min_u_out      (is_min_u_id_ex_reg_to_ex_stage),
        .ex_is_abs_diff_u_out (is_abs_diff_u_id_ex_reg_to_ex_stage)
    );

    // 5. Execute (EX) Stage
    ex_stage ex_stage_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),

        .pc_plus_4_in          (id_ex_pc_plus_4_out),
        .reg_read_data1_in     (id_ex_reg_read_data1_out),
        .reg_read_data2_in     (id_ex_reg_read_data2_out),
        .immediate_in          (id_ex_immediate_out),
        .rs1_addr_in           (id_ex_rs1_addr_out),
        .rs2_addr_in           (id_ex_rs2_addr_out),
        .rd_addr_in            (id_ex_rd_addr_out),
        .reg_write_en_in       (id_ex_reg_write_en_out),
        .mem_read_en_in        (id_ex_mem_read_en_out),
        .mem_write_en_in       (id_ex_mem_write_en_out),
        .mem_to_reg_in         (id_ex_mem_to_reg_out),
        .alu_op_in             (id_ex_alu_op_out),
        .alu_src_a_in          (id_ex_alu_src_a_out),
        .alu_src_b_in          (id_ex_alu_src_b_out),
        .branch_in             (id_ex_branch_out),
        .jump_in               (id_ex_jump_out),
        .jalr_in               (id_ex_jalr_out),
        .funct3_in             (id_ex_funct3_out),
        .funct7_in             (id_ex_funct7_out),

        .forward_a             (forward_a),
        .forward_b             (forward_b),
        .forward_ex_alu_result (ex_mem_alu_result_out),
        .forward_mem_load_data (mem_wb_load_data_out),
        .forward_mem_alu_result (mem_wb_alu_result_out),
        .forward_mem_pc_plus_4  (mem_wb_pc_plus_4_out),

        .alu_result_out        (ex_stage_to_ex_mem_reg_alu_result),
        .reg_read_data2_out    (ex_stage_to_ex_mem_reg_reg_read_data2),
        .branch_taken_out      (ex_stage_to_ex_mem_reg_branch_taken),
        .branch_target_out     (ex_stage_to_ex_mem_reg_branch_target),
        .rd_addr_out           (ex_stage_to_ex_mem_reg_rd_addr),
        .reg_write_en_out      (ex_stage_to_ex_mem_reg_reg_write_en),
        .mem_to_reg_out        (ex_stage_to_ex_mem_reg_mem_to_reg),
        .funct3_out            (ex_stage_to_ex_mem_reg_funct3),
        .alu_op_out            (ex_stage_to_ex_mem_reg_alu_op),
        .mem_read_en_out       (ex_stage_to_ex_mem_reg_mem_read_en),
        .mem_write_en_out      (ex_stage_to_ex_mem_reg_mem_write_en),
        .pc_plus_4_out         (ex_stage_to_ex_mem_reg_pc_plus_4)
    );

    // 6. EX/MEM Pipeline Register
    ex_mem_reg ex_mem_reg_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .ex_pc_plus_4_in       (ex_stage_to_ex_mem_reg_pc_plus_4),
        .ex_alu_result_in      (ex_stage_to_ex_mem_reg_alu_result),
        .ex_reg_read_data2_in  (ex_stage_to_ex_mem_reg_reg_read_data2),
        .ex_rd_addr_in         (ex_stage_to_ex_mem_reg_rd_addr),
        .ex_reg_write_en_in    (ex_stage_to_ex_mem_reg_reg_write_en),
        .ex_mem_to_reg_in      (ex_stage_to_ex_mem_reg_mem_to_reg),
        .ex_mem_read_en_in     (ex_stage_to_ex_mem_reg_mem_read_en),
        .ex_mem_write_en_in    (ex_stage_to_ex_mem_reg_mem_write_en),
        .ex_funct3_in          (ex_stage_to_ex_mem_reg_funct3),

        .mem_pc_plus_4_out     (ex_mem_pc_plus_4_out),
        .mem_alu_result_out    (ex_mem_alu_result_out),
        .mem_reg_read_data2_out (ex_mem_reg_read_data2_out),
        .mem_rd_addr_out       (ex_mem_rd_addr_out),
        .mem_reg_write_en_out  (ex_mem_reg_write_en_out),
        .mem_mem_to_reg_out    (ex_mem_mem_to_reg_out),
        .mem_mem_read_en_out   (ex_mem_mem_read_en_out),
        .mem_mem_write_en_out  (ex_mem_mem_write_en_out),
        .mem_funct3_out        (ex_mem_funct3_out)
    );

    // 7. Memory Access (MEM) Stage
    mem_stage mem_stage_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),

        .alu_result_in         (ex_mem_alu_result_out),
        .reg_read_data2_in     (ex_mem_reg_read_data2_out),
        .mem_read_en_in        (ex_mem_mem_read_en_out),
        .mem_write_en_in       (ex_mem_mem_write_en_out),
        .funct3_in             (ex_mem_funct3_out),

        .mem_addr              (mem_addr),
        .mem_write_data        (mem_write_data),
        .mem_write_en          (mem_write_en),
        .mem_read_en           (mem_read_en),
        .mem_byte_enable       (mem_byte_enable),

        .mem_read_data_in      (mem_read_data),

        .load_data_out         (mem_stage_to_mem_wb_reg_load_data)
    );

    // 8. MEM/WB Pipeline Register
    mem_wb_reg mem_wb_reg_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .mem_alu_result_in     (ex_mem_alu_result_out),
        .mem_load_data_in      (mem_stage_to_mem_wb_reg_load_data),
        .mem_pc_plus_4_in      (ex_mem_pc_plus_4_out),
        .mem_rd_addr_in        (ex_mem_rd_addr_out),
        .mem_reg_write_en_in   (ex_mem_reg_write_en_out),
        .mem_mem_to_reg_in     (ex_mem_mem_to_reg_out),

        .wb_alu_result_out     (mem_wb_alu_result_out),
        .wb_load_data_out      (mem_wb_load_data_out),
        .wb_pc_plus_4_out      (mem_wb_pc_plus_4_out),
        .wb_rd_addr_out        (mem_wb_rd_addr_out),
        .wb_reg_write_en_out   (mem_wb_reg_write_en_out),
        .wb_mem_to_reg_out     (mem_wb_mem_to_reg_out)
    );

    // 9. Write Back (WB) Stage - NEWLY ADDED AS A SEPARATE MODULE
    wb_stage wb_stage_inst (
        .clk              (clk),
        .rst_n            (rst_n),
        // Inputs from MEM/WB register
        .alu_result_in    (mem_wb_alu_result_out),
        .load_data_in     (mem_wb_load_data_out),
        .pc_plus_4_in     (mem_wb_pc_plus_4_out), // Pass PC+4 through to WB stage
        .rd_addr_in       (mem_wb_rd_addr_out),
        .reg_write_en_in  (mem_wb_reg_write_en_out),
        .mem_to_reg_in    (mem_wb_mem_to_reg_out),
        // Outputs to Register File
        .wb_write_data_out (wb_stage_write_data),
        .wb_write_addr_out (wb_stage_write_addr),
        .wb_write_en_out   (wb_stage_write_en)
    );

    // Register File (shared resource for ID read and WB write)
    reg_file reg_file_inst (
        .clk         (clk),
        .rst_n       (rst_n),
        .read_addr1  (id_stage_to_id_ex_reg_rs1_addr),
        .read_addr2  (id_stage_to_id_ex_reg_rs2_addr),
        .read_data1  (reg_file_read_data1),
        .read_data2  (reg_file_read_data2),
        // Connect RegFile write inputs to WB_STAGE outputs
        .write_en    (wb_stage_write_en),
        .write_addr  (wb_stage_write_addr),
        .write_data  (wb_stage_write_data)
    );

    // 10. Hazard Detection Unit
    hazard_unit hazard_unit_inst (
        .id_ex_mem_read_en     (id_ex_mem_read_en_out),
        .id_ex_rd_addr         (id_ex_rd_addr_out),
        .ex_mem_reg_write_en   (ex_mem_reg_write_en_out),
        .ex_mem_rd_addr        (ex_mem_rd_addr_out),
        .mem_wb_reg_write_en   (mem_wb_reg_write_en_out),
        .mem_wb_rd_addr        (mem_wb_rd_addr_out),
        .id_rs1_addr           (id_ex_rs1_addr_out),
        .id_rs2_addr           (id_ex_rs2_addr_out),
        .id_branch_inst        (id_ex_branch_out),
        .id_jump_inst          (id_ex_jump_out),
        .id_jalr_inst          (id_ex_jalr_out),
        .ex_branch_taken       (ex_stage_to_ex_mem_reg_branch_taken),
        .stall_if              (stall_if),
        .stall_id              (stall_id),
        .flush_id              (flush_id),
        .forward_a             (forward_a),
        .forward_b             (forward_b)
    );

endmodule
