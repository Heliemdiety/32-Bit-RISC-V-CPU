// Hazard Unit for the RISC-V Pipeline.
// Detects data and control hazards and generates stall, flush, and forwarding signals.

module hazard_unit (
    // Inputs from Pipeline Registers and EX Stage:
    input wire        id_ex_mem_read_en,      // MemRead enable from ID/EX register (for load-use hazard)
    input wire [4:0]  id_ex_rd_addr,          // Destination register of instruction in ID/EX stage

    input wire        ex_mem_reg_write_en,    // RegWrite enable from EX/MEM register (for forwarding)
    input wire [4:0]  ex_mem_rd_addr,         // Destination register of instruction in EX/MEM stage

    input wire        mem_wb_reg_write_en,    // RegWrite enable from MEM/WB register (for forwarding)
    input wire [4:0]  mem_wb_rd_addr,         // Destination register of instruction in MEM/WB stage

    // Current Instruction (ID Stage) source registers
    input wire [4:0]  id_rs1_addr,            // Source register 1 address from ID stage
    input wire [4:0]  id_rs2_addr,            // Source register 2 address from ID stage

    // Branch/Jump Information (from ID stage and EX stage)
    input wire        id_branch_inst,         // Is the instruction in ID stage a branch?
    input wire        id_jump_inst,           // Is the instruction in ID stage a JAL?
    input wire        id_jalr_inst,           // Is the instruction in ID stage a JALR?
    input wire        ex_branch_taken,        // Is a branch in EX stage actually taken? (for control hazard detection)

    // Outputs to Pipeline Stages:
    output reg        stall_if,               // Stall Instruction Fetch stage (PC holds, IF/ID holds)
    output reg        stall_id,               // Stall Instruction Decode stage (bubble into ID/EX)
    output reg        flush_id,               // Flush ID/EX register (insert NOP into EX stage)
    output reg [1:0]  forward_a,              // Forwarding control for ALU operand A (rs1)
    output reg [1:0]  forward_b               // Forwarding control for ALU operand B (rs2)
);

    always @(*) begin
        // Default values: no forwarding, no stalls, no flush.
        forward_a = 2'b00; // No forwarding
        forward_b = 2'b00; // No forwarding
        stall_if  = 1'b0;
        stall_id  = 1'b0;
        flush_id  = 1'b0;

        // --- Data Hazard Detection and Forwarding Logic ---
        // Forwarding priorities: EX/MEM (closest) > MEM/WB (further)

        // Forwarding for RS1 (operand A)
        // EX/MEM to EX forwarding: If the EX/MEM instruction writes to rs1 and rd is not x0
        if (ex_mem_reg_write_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs1_addr)) begin
            forward_a = 2'b01; // Forward from EX/MEM.ALU_Result
        end
        // MEM/WB to EX forwarding: If the MEM/WB instruction writes to rs1 and rd is not x0,
        // AND the EX/MEM instruction is NOT also writing to rs1 (higher priority)
        if (mem_wb_reg_write_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_rs1_addr) &&
            !(ex_mem_reg_write_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs1_addr))) begin
            forward_a = 2'b10; // Forward from MEM/WB.Result
        end

        // Forwarding for RS2 (operand B) - Same logic as RS1
        if (ex_mem_reg_write_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs2_addr)) begin
            forward_b = 2'b01; // Forward from EX/MEM.ALU_Result
        end
        if (mem_wb_reg_write_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_rs2_addr) &&
            !(ex_mem_reg_write_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs2_addr))) begin
            forward_b = 2'b10; // Forward from MEM/WB.Result
        end

        // --- Load-Use Hazard Detection (Stall) ---
        // If the instruction in ID/EX is a load (mem_read_en) and its destination register (id_ex_rd_addr)
        // is a source register for the current instruction in ID stage (id_rs1_addr or id_rs2_addr),
        // a stall is required because the load data is not available for forwarding yet.
        if (id_ex_mem_read_en && (id_ex_rd_addr != 5'b0) &&
            ((id_ex_rd_addr == id_rs1_addr) || (id_ex_rd_addr == id_rs2_addr))) begin
            stall_if = 1'b1; // Stall IF stage (PC holds, IF/ID register holds)
            stall_id = 1'b1; // Stall ID stage (prevents ID/EX register from updating, inserting bubble)
            $display("    HAZARD_UNIT Debug (Time %0t): Load-Use Hazard Detected! Stall IF/ID. ID_EX_RD=%d, ID_RS1=%d, ID_RS2=%d",
                     $time, id_ex_rd_addr, id_rs1_addr, id_rs2_addr);
        end

        // --- Control Hazard Detection (Flush) ---
        // If a branch is taken in the EX stage, or an unconditional jump (JAL/JALR) is in the ID stage,
        // the instruction currently in the ID/EX register is incorrect and must be flushed (turned into a NOP).
        // The IF stage's PC is updated by the branch/jump target.
        if (ex_branch_taken || id_jump_inst || id_jalr_inst) begin
            flush_id = 1'b1; // Flush ID/EX register (turns it into a bubble/NOP for the EX stage)
            $display("    HAZARD_UNIT Debug (Time %0t): Control Hazard Detected! Flush ID/EX. BranchTaken=%b, JAL=%b, JALR=%b",
                     $time, ex_branch_taken, id_jump_inst, id_jalr_inst);
        end

        // General Hazard Unit output debug (can be removed for synthesis)
        $display("    HAZARD_UNIT Debug (Time %0t): stall_if=%b, stall_id=%b, flush_id=%b, forward_a=%b, forward_b=%b",
                 $time, stall_if, stall_id, flush_id, forward_a, forward_b);
    end

endmodule
