// Hazard Detection Unit for the RISC-V CPU pipeline.
// Detects data hazards (load-use) and control hazards (branches/jumps) and generates appropriate stall, flush, and forwarding signals.

module hazard_unit (
    input wire        id_ex_mem_read_en, 
    input wire [4:0]  id_ex_rd_addr,     // Destination register of ID/EX instruction

    input wire        ex_mem_reg_write_en,
    input wire [4:0]  ex_mem_rd_addr,      // Destination register of EX/MEM instruction

    input wire        mem_wb_reg_write_en, 
    input wire [4:0]  mem_wb_rd_addr,      // Destination register of MEM/WB instruction

    // Current Instruction (ID Stage) source registers
    input wire [4:0]  id_rs1_addr,
    input wire [4:0]  id_rs2_addr,

    // Branch/Jump Information
    input wire        id_branch_inst,    
    input wire        id_jump_inst,      
    input wire        id_jalr_inst,      
    input wire        ex_branch_taken,  

    // Outputs to Pipeline Stages
    output reg        stall_if,          // Stall Instruction Fetch stage (PC = PC)
    output reg        stall_id,          // Stall Instruction Decode stage (insert bubble into ID/EX)
    output reg        flush_id,          // Flush ID/EX register (set all to 0/nop)
    output reg [1:0]  forward_a,         // Forwarding control for ALU operand A (rs1)
    output reg [1:0]  forward_b          // Forwarding control for ALU operand B (rs2)
);

    always @(*) begin 
        
        forward_a = 2'b00;
        forward_b = 2'b00;
        if (ex_mem_reg_write_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs1_addr)) begin
            forward_a = 2'b01; // Forward from EX/MEM
        end
        if (mem_wb_reg_write_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_rs1_addr) &&
            !((ex_mem_reg_write_en) && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs1_addr))) begin
            forward_a = 2'b10; // Forward from MEM/WB
        end

        if (ex_mem_reg_write_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs2_addr)) begin
            forward_b = 2'b01; // Forward from EX/MEM
        end
        
        if (mem_wb_reg_write_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_rs2_addr) &&
            !((ex_mem_reg_write_en) && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_rs2_addr))) begin
            forward_b = 2'b10; // Forward from MEM/WB
        end

        stall_if = 1'b0;
        stall_id = 1'b0;
        flush_id = 1'b0;


        if (id_ex_mem_read_en && (id_ex_rd_addr != 5'b0) &&
            ((id_ex_rd_addr == id_rs1_addr) || (id_ex_rd_addr == id_rs2_addr))) begin
            stall_if = 1'b1; // Stall IF stage
            stall_id = 1'b1; // Stall ID stage (insert bubble in ID/EX)
            $display("    HAZARD_UNIT Debug (Time %0t): Load-Use Hazard Detected! Stall IF/ID. ID_EX_RD=%d, ID_RS1=%d, ID_RS2=%d",
                     $time, id_ex_rd_addr, id_rs1_addr, id_rs2_addr);
        end

        if (ex_branch_taken || id_jump_inst || id_jalr_inst) begin
            flush_id = 1'b1; // Flush ID/EX register (turns it into a bubble)
            // Note: IF stage PC is also updated by branch_taken_ex from EX stage.
            $display("    HAZARD_UNIT Debug (Time %0t): Control Hazard Detected! Flush ID/EX. BranchTaken=%b, JAL=%b, JALR=%b",
                     $time, ex_branch_taken, id_jump_inst, id_jalr_inst);
        end

        // General Hazard Unit output debug
        $display("    HAZARD_UNIT Debug (Time %0t): stall_if=%b, stall_id=%b, flush_id=%b, forward_a=%b, forward_b=%b",
                 $time, stall_if, stall_id, flush_id, forward_a, forward_b);
    end

endmodule
