// Top-level module for a 5-stage pipelined RISC-V RV32I CPU core.
// This module connects all the pipeline stages, register file, ALU,
// control unit, hazard unit, and memory interfaces.

module riscv_cpu_top (
    input wire clk,
    input wire rst_n, // Active low reset

    // Memory Interface (for external data memory)
    output wire [31:0] mem_addr,
    output wire [31:0] mem_write_data,
    output wire        mem_write_en,
    output wire        mem_read_en,
    input wire  [31:0] mem_read_data,
    output wire  [3:0] mem_byte_enable,

    // Instruction Memory Interface (connected to instruction_memory module)
    output wire [31:0] imem_addr,
    input wire  [31:0] imem_read_data // Instruction fetched by IF stage
);

    

    // IF/ID Pipeline Register Inputs (from IF stage)
    wire [31:0] if_pc_out; // Current PC out for IF/ID reg
    wire [31:0] if_pc_plus_4_out;
    wire [31:0] if_instruction_out;

    // Wires explicitly declared for IF/ID pipeline register outputs to ID stage
    wire [31:0] id_pc_out_from_if_id;
    wire [31:0] id_pc_plus_4_out_from_if_id;
    wire [31:0] id_instruction_out_from_if_id;


    //Wires from ID_STAGE outputs to ID_EX_REG inputs
    // These wires carry the decoded instruction properties from ID_STAGE to the ID/EX pipeline register
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

    // ID/EX Pipeline Register Outputs (from ID_EX_REG to EX_STAGE)
    wire [31:0] id_ex_pc_plus_4_out;
    wire [31:0] id_ex_reg_read_data1_out;
    wire [31:0] id_ex_reg_read_data2_out;
    wire [31:0] id_ex_immediate_out;
    wire [4:0]  id_ex_rs1_addr_out;
    wire [4:0]  id_ex_rs2_addr_out;
    wire [4:0]  id_ex_rd_addr_out;
    wire        id_ex_reg_write_en_out;
    wire [1:0]  id_ex_mem_to_reg_out; // 00: ALU_result, 01: Load_data, 10: PC+4 (for JAL/JALR)
    wire [3:0]  id_ex_alu_op_out;
    wire [1:0]  id_ex_alu_src_a_out; // 00: pc, 01: reg_data1
    wire [1:0]  id_ex_alu_src_b_out; // 00: reg_data2, 01: immediate
    wire        id_ex_mem_read_en_out;
    wire        id_ex_mem_write_en_out;
    wire        id_ex_branch_out;
    wire        id_ex_jump_out;
    wire        id_ex_jalr_out;
    wire [2:0]  id_ex_funct3_out; // Used for load/store type, and branch type
    wire [6:0]  id_ex_funct7_out; // Used for R-type ALU ops

    wire [6:0] id_stage_opcode_debug_wire; // For debugging ID stage internal opcode


    // Wires from EX_STAGE outputs to EX_MEM_REG inputs
    // These wires will carry the EX stage results and controls to the EX/MEM pipeline register.
    wire [31:0] ex_stage_to_ex_mem_reg_alu_result;
    wire [31:0] ex_stage_to_ex_mem_reg_reg_read_data2; // For store instructions
    wire        ex_stage_to_ex_mem_reg_branch_taken; // For PC update
    wire [31:0] ex_stage_to_ex_mem_reg_branch_target; // For PC update
    wire [4:0]  ex_stage_to_ex_mem_reg_rd_addr;
    wire        ex_stage_to_ex_mem_reg_reg_write_en;
    wire [1:0]  ex_stage_to_ex_mem_reg_mem_to_reg;
    wire [2:0]  ex_stage_to_ex_mem_reg_funct3;
    wire [1:0]  ex_stage_to_ex_mem_reg_alu_src_a;
    wire [1:0]  ex_stage_to_ex_mem_reg_alu_src_b;
    wire [3:0]  ex_stage_to_ex_mem_reg_alu_op; // Added for completeness, though often implied from mem_to_reg
    wire        ex_stage_to_ex_mem_reg_mem_read_en;
    wire        ex_stage_to_ex_mem_reg_mem_write_en;


    // EX/MEM Pipeline Register Outputs (from EX_MEM_REG to MEM_STAGE)
    wire [31:0] ex_mem_pc_plus_4_out; // PC+4 value from EX/MEM for JAL/JALR return
    wire [31:0] ex_mem_alu_result_out; // ALU result from EX stage
    wire [31:0] ex_mem_reg_read_data2_out; // Data for store from EX stage
    wire [4:0]  ex_mem_rd_addr_out;
    wire        ex_mem_reg_write_en_out;
    wire [1:0]  ex_mem_mem_to_reg_out;
    wire        ex_mem_mem_read_en_out;
    wire        ex_mem_mem_write_en_out;
    wire [2:0]  ex_mem_funct3_out; // For load/store type, and branch type in MEM stage

    // MEM/WB Pipeline Register Outputs (from MEM_WB_REG to RegFile write)
    wire [31:0] mem_wb_alu_result_out; // ALU result from MEM stage (passed through)
    wire [31:0] mem_wb_load_data_out;  // Loaded data from MEM stage
    wire [4:0]  mem_wb_rd_addr_out;
    wire        mem_wb_reg_write_en_out;
    wire [1:0]  mem_wb_mem_to_reg_out;

    // Register File Read Data (from RegFile to ID stage)
    wire [31:0] reg_file_read_data1;
    wire [31:0] reg_file_read_data2;

    // Hazard Unit Signals
    wire        stall_if;    // Stall IF stage (PC not incremented)
    wire        stall_id;    // Stall ID stage (bubble inserted into ID/EX)
    wire        flush_id;    // Flush ID/EX register (for mispredicted branches/jumps)
    wire [1:0]  forward_a;   // Forwarding control for ALU operand A (rs1)
    wire [1:0]  forward_b;   // Forwarding control for ALU operand B (rs2)

    // Branch/Jump Decision from EX Stage (feedback to IF)
    wire        ex_branch_taken;
    wire [31:0] ex_branch_target;


    // 1. Instruction Fetch (IF) Stage
    if_stage if_stage_inst (
        .clk              (clk),
        .rst_n            (rst_n),
        .pc_out           (if_pc_out), // Current PC out for IF/ID reg
        .instruction_address(imem_addr), // Output to instruction memory (top-level port)
        .instruction_read_data(imem_read_data), // Input from instruction memory (top-level port)
        .instruction_out  (if_instruction_out), // Output instruction for IF/ID reg
        .pc_plus_4_out    (if_pc_plus_4_out), // Output PC+4 for IF/ID reg
        .stall            (stall_if), // Stall signal from hazard unit
        .flush            (flush_id), // Flush signal from hazard unit
        .branch_taken_ex  (ex_branch_taken), // Branch taken decision from EX stage
        .branch_target_ex (ex_branch_target), // Target for taken branch/jump
        .jump_taken_ex    (id_ex_jump_out | id_ex_jalr_out) // Jumps also cause PC change, from ID/EX register 
    );

    // 2. IF/ID Pipeline Register
    if_id_reg if_id_reg_inst (
        .clk              (clk),
        .rst_n            (rst_n),
        .stall            (stall_if),
        .flush            (flush_id),
        .if_pc_in         (if_pc_out),
        .if_pc_plus_4_in  (if_pc_plus_4_out),
        .if_instruction_in(if_instruction_out),
        .id_pc_out        (id_pc_out_from_if_id),
        .id_pc_plus_4_out (id_pc_plus_4_out_from_if_id),
        .id_instruction_out(id_instruction_out_from_if_id)
    );

    // 3. Instruction Decode (ID) Stage
    id_stage id_stage_inst (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .instruction_in        (id_instruction_out_from_if_id), // Instruction from IF/ID
        .pc_curr_if            (id_pc_out_from_if_id),     // Current PC from IF/ID (for AUIPC)

        // Register File Interface (read ports)
        // These are connected to the id_stage_to_id_ex_reg_ wires as they go to ID_EX_REG and also directly used here
        .reg_file_read_addr1   (id_stage_to_id_ex_reg_rs1_addr),
        .reg_file_read_addr2   (id_stage_to_id_ex_reg_rs2_addr),
        .reg_file_read_data1   (reg_file_read_data1), // Data from RegFile
        .reg_file_read_data2   (reg_file_read_data2), // Data from RegFile

        // Outputs to ID/EX pipeline register - NOW CONNECTED TO THE NEW INTERMEDIATE WIRES
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
        .opcode_debug          (id_stage_opcode_debug_wire) // Debug output
    );

    // Register File (shared resource, accessed by ID and WB stages)
    reg_file reg_file_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .read_addr1     (id_stage_to_id_ex_reg_rs1_addr), // RS1 address from ID_STAGE (for RegFile read)
        .read_addr2     (id_stage_to_id_ex_reg_rs2_addr), // RS2 address from ID_STAGE (for RegFile read)
        .read_data1     (reg_file_read_data1), // Data for RS1
        .read_data2     (reg_file_read_data2), // Data for RS2
        .write_en       (mem_wb_reg_write_en_out), // Write enable from MEM/WB
        .write_addr     (mem_wb_rd_addr_out), // Write address from MEM/WB
        .write_data     (
            // Data to write back based on Mem_to_Reg control from MEM/WB
            mem_wb_mem_to_reg_out == 2'b00 ? mem_wb_alu_result_out : // ALU result for R-type/I-type
            mem_wb_mem_to_reg_out == 2'b01 ? mem_wb_load_data_out  : // Loaded data for Load-type
            mem_wb_mem_to_reg_out == 2'b10 ? ex_mem_pc_plus_4_out :  // PC+4 for JAL/JALR (comes from EX_MEM_REG, should be mem_wb_pc_plus_4_out if it were passed along)
            32'b0
        )
    );

    // 4. ID/EX Pipeline Register
    id_ex_reg id_ex_reg_inst (
        .clk                (clk),
        .rst_n              (rst_n),
        .stall              (stall_id), // Stall signal from hazard unit
        .flush              (flush_id), // Flush signal from hazard unit

        // ID Stage Inputs - NOW CONNECTED TO THE NEW INTERMEDIATE WIRES from id_stage
        .id_pc_plus_4_in    (id_pc_plus_4_out_from_if_id), // PC+4 from IF/ID, not from ID stage directly
        .id_reg_read_data1_in (reg_file_read_data1), // Raw data from RegFile (connected here, forwarded in EX)
        .id_reg_read_data2_in (reg_file_read_data2), // Raw data from RegFile (connected here, forwarded in EX)
        .id_immediate_in    (id_stage_to_id_ex_reg_immediate),
        .id_rs1_addr_in     (id_stage_to_id_ex_reg_rs1_addr),
        .id_rs2_addr_in     (id_stage_to_id_ex_reg_rs2_addr),
        .id_rd_addr_in      (id_stage_to_id_ex_reg_rd_addr),
        .id_reg_write_en_in (id_stage_to_id_ex_reg_write_en),
        .id_mem_to_reg_in   (id_stage_to_id_ex_reg_mem_to_reg),
        .id_alu_op_in       (id_stage_to_id_ex_reg_alu_op),
        .id_alu_src_a_in    (id_stage_to_id_ex_reg_alu_src_a),
        .id_alu_src_b_in    (id_stage_to_id_ex_reg_alu_src_b),
        .id_mem_read_en_in  (id_stage_to_id_ex_reg_mem_read_en),
        .id_mem_write_en_in (id_stage_to_id_ex_reg_mem_write_en),
        .id_branch_in       (id_stage_to_id_ex_reg_branch),
        .id_jump_in         (id_stage_to_id_ex_reg_jump),
        .id_jalr_in         (id_stage_to_id_ex_reg_jalr),
        .id_funct3_in       (id_stage_to_id_ex_reg_funct3),
        .id_funct7_in       (id_stage_to_id_ex_reg_funct7),

        // EX Stage Outputs (these are outputs from ID_EX_REG to EX_STAGE)
        .ex_pc_plus_4_out   (id_ex_pc_plus_4_out),
        .ex_reg_read_data1_out (id_ex_reg_read_data1_out),
        .ex_reg_read_data2_out (id_ex_reg_read_data2_out),
        .ex_immediate_out   (id_ex_immediate_out),
        .ex_rs1_addr_out    (id_ex_rs1_addr_out),
        .ex_rs2_addr_out    (id_ex_rs2_addr_out),
        .ex_rd_addr_out     (id_ex_rd_addr_out),
        .ex_reg_write_en_out (id_ex_reg_write_en_out),
        .ex_mem_to_reg_out  (id_ex_mem_to_reg_out),
        .ex_alu_op_out      (id_ex_alu_op_out),
        .ex_alu_src_a_out   (id_ex_alu_src_a_out),
        .ex_alu_src_b_out   (id_ex_alu_src_b_out),
        .ex_mem_read_en_out (id_ex_mem_read_en_out),
        .ex_mem_write_en_out (id_ex_mem_write_en_out),
        .ex_branch_out      (id_ex_branch_out),
        .ex_jump_out        (id_ex_jump_out),
        .ex_jalr_out        (id_ex_jalr_out),
        .ex_funct3_out      (id_ex_funct3_out),
        .ex_funct7_out      (id_ex_funct7_out)
    );

    // 5. Execute (EX) Stage
    ex_stage ex_stage_inst (
        .clk               (clk),
        .rst_n             (rst_n),

        // Inputs from ID/EX register
        .pc_plus_4_in      (id_ex_pc_plus_4_out),
        .reg_read_data1_in (id_ex_reg_read_data1_out),
        .reg_read_data2_in (id_ex_reg_read_data2_out),
        .immediate_in      (id_ex_immediate_out),
        .rs1_addr_in       (id_ex_rs1_addr_out),
        .rs2_addr_in       (id_ex_rs2_addr_out),
        .rd_addr_in        (id_ex_rd_addr_out),
        .reg_write_en_in   (id_ex_reg_write_en_out),
        .mem_read_en_in    (id_ex_mem_read_en_out),         
        .mem_to_reg_in     (id_ex_mem_to_reg_out),
        .alu_op_in         (id_ex_alu_op_out),
        .alu_src_a_in      (id_ex_alu_src_a_out),
        .alu_src_b_in      (id_ex_alu_src_b_out),
        .branch_in         (id_ex_branch_out),
        .jump_in           (id_ex_jump_out),
        .jalr_in           (id_ex_jalr_out),
        .funct3_in         (id_ex_funct3_out),
        .funct7_in         (id_ex_funct7_out),

        // Forwarding control from Hazard Unit
        .forward_a         (forward_a),
        .forward_b         (forward_b),
        // Forwarded data from later stages
        .forward_ex_alu_result (ex_mem_alu_result_out), // From EX/MEM (ALU result)
        .forward_mem_load_data (mem_wb_load_data_out), // From MEM/WB (Load data)
        .forward_mem_alu_result (mem_wb_alu_result_out), // From MEM/WB (ALU result)

        // Outputs to EX/MEM register - NOW CONNECTED TO THE NEW INTERMEDIATE WIRES
        .alu_result_out    (ex_stage_to_ex_mem_reg_alu_result),
        .reg_read_data2_out (ex_stage_to_ex_mem_reg_reg_read_data2),
        .branch_taken_out  (ex_stage_to_ex_mem_reg_branch_taken),
        .branch_target_out (ex_stage_to_ex_mem_reg_branch_target),
        .rd_addr_out       (ex_stage_to_ex_mem_reg_rd_addr),
        .reg_write_en_out  (ex_stage_to_ex_mem_reg_reg_write_en),
        .mem_to_reg_out    (ex_stage_to_ex_mem_reg_mem_to_reg),
        .funct3_out        (ex_stage_to_ex_mem_reg_funct3),
        .alu_src_a_out     (ex_stage_to_ex_mem_reg_alu_src_a),
        .alu_src_b_out     (ex_stage_to_ex_mem_reg_alu_src_b),
        .alu_op_out        (ex_stage_to_ex_mem_reg_alu_op),
        .mem_read_en_out   (ex_stage_to_ex_mem_reg_mem_read_en),
        .mem_write_en_out  (ex_stage_to_ex_mem_reg_mem_write_en),
        
        .mem_write_en_in   (id_ex_mem_write_en_out)
    );

    // 6. EX/MEM Pipeline Register
    ex_mem_reg ex_mem_reg_inst (
        .clk                (clk),
        .rst_n              (rst_n),
        // Inputs from EX stage - ALL CONNECTED TO THE NEW INTERMEDIATE WIRES
        .ex_pc_plus_4_in    (id_ex_pc_plus_4_out), // PC+4 from ID/EX (passed through)
        .ex_alu_result_in   (ex_stage_to_ex_mem_reg_alu_result),
        .ex_reg_read_data2_in (ex_stage_to_ex_mem_reg_reg_read_data2),
        .ex_rd_addr_in      (ex_stage_to_ex_mem_reg_rd_addr),
        .ex_reg_write_en_in (ex_stage_to_ex_mem_reg_reg_write_en),
        .ex_mem_to_reg_in   (ex_stage_to_ex_mem_reg_mem_to_reg),
        .ex_mem_read_en_in  (ex_stage_to_ex_mem_reg_mem_read_en),
        .ex_mem_write_en_in (ex_stage_to_ex_mem_reg_mem_write_en),
        .ex_funct3_in       (ex_stage_to_ex_mem_reg_funct3), // funct3 still comes from EX

        // Outputs to MEM stage (these are outputs from EX_MEM_REG to MEM_STAGE)
        // These are the wires that should be driven only by this register.
        .mem_pc_plus_4_out  (ex_mem_pc_plus_4_out),
        .mem_alu_result_out (ex_mem_alu_result_out),
        .mem_reg_read_data2_out (ex_mem_reg_read_data2_out),
        .mem_rd_addr_out    (ex_mem_rd_addr_out),
        .mem_reg_write_en_out (ex_mem_reg_write_en_out),
        .mem_mem_to_reg_out (ex_mem_mem_to_reg_out),
        .mem_mem_read_en_out (ex_mem_mem_read_en_out),
        .mem_mem_write_en_out (ex_mem_mem_write_en_out),
        .mem_funct3_out     (ex_mem_funct3_out)
    );

    // 7. Memory Access (MEM) Stage
    mem_stage mem_stage_inst (
        .clk                (clk),
        .rst_n              (rst_n),

        // Inputs from EX/MEM register
        .alu_result_in      (ex_mem_alu_result_out), // Address for memory access
        .reg_read_data2_in  (ex_mem_reg_read_data2_out), // Data to write
        .mem_read_en_in     (ex_mem_mem_read_en_out),
        .mem_write_en_in    (ex_mem_mem_write_en_out),
        .funct3_in          (ex_mem_funct3_out),

        // Outputs for Data Memory interface (top-level ports)
        .mem_addr           (mem_addr),
        .mem_write_data     (mem_write_data),
        .mem_write_en       (mem_write_en),
        .mem_read_en        (mem_read_en),
        .mem_byte_enable    (mem_byte_enable),

        // Input from Data Memory (top-level port)
        .mem_read_data_in   (mem_read_data),

        // Outputs to MEM/WB register
        .load_data_out      (mem_wb_load_data_out)
    );

    // 8. MEM/WB Pipeline Register
    mem_wb_reg mem_wb_reg_inst (
        .clk                (clk),
        .rst_n              (rst_n),
        .mem_alu_result_in  (ex_mem_alu_result_out), // ALU result from EX/MEM
        .mem_load_data_in   (mem_wb_load_data_out), // Loaded data from MEM_STAGE
        .mem_rd_addr_in     (ex_mem_rd_addr_out), // RD address from EX/MEM
        .mem_reg_write_en_in (ex_mem_reg_write_en_out),
        .mem_mem_to_reg_in  (ex_mem_mem_to_reg_out),

        // Outputs to WB stage (directly to Register File write port)
        .wb_alu_result_out  (mem_wb_alu_result_out),
        .wb_load_data_out   (mem_wb_load_data_out),
        .wb_rd_addr_out     (mem_wb_rd_addr_out),
        .wb_reg_write_en_out (mem_wb_reg_write_en_out),
        .wb_mem_to_reg_out  (mem_wb_mem_to_reg_out)
    );

    // 9. Write Back (WB) Stage - Integrated into Reg_File write port for simplicity.

    // 10. Hazard Detection Unit
    hazard_unit hazard_unit_inst (
        // Inputs related to pipeline state
        .id_ex_mem_read_en      (id_ex_mem_read_en_out), //
        .id_ex_rd_addr          (id_ex_rd_addr_out),     // Rd of ID/EX
        .ex_mem_reg_write_en    (ex_mem_reg_write_en_out), /
        .ex_mem_rd_addr         (ex_mem_rd_addr_out),      // Rd of EX/MEM
        .mem_wb_reg_write_en    (mem_wb_reg_write_en_out), //MEM/WB writing
        .mem_wb_rd_addr         (mem_wb_rd_addr_out),      // Rd of MEM/WB
        .id_rs1_addr            (id_ex_rs1_addr_out),     // RS1 of current ID (from ID/EX reg)
        .id_rs2_addr            (id_ex_rs2_addr_out),     // RS2 of current ID (from ID/EX reg)
        .id_branch_inst         (id_ex_branch_out),     // Is current ID a branch
        .id_jump_inst           (id_ex_jump_out),       // Is current ID a JAL? (from ID/EX reg)
        .id_jalr_inst           (id_ex_jalr_out),       // Is current ID a JALR? (from ID/EX reg)
        .ex_branch_taken        (ex_stage_to_ex_mem_reg_branch_taken), // Branch taken decision from EX (using new wire)

        // Outputs (Stalls, Flushes, Forwards)
        .stall_if               (stall_if),
        .stall_id               (stall_id),
        .flush_id               (flush_id),
        .forward_a              (forward_a),
        .forward_b              (forward_b)
    );

endmodule
