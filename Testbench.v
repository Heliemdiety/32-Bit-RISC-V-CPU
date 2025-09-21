`timescale 1ns / 1ps 

module riscv_cpu_tb;

    integer i; // the most common mistake i have made many times ,, using integer i in for loop like c++ 

   
    parameter CLK_PERIOD = 10; // Clock period in ns (10ns = 100MHz)
    parameter RESET_DURATION = 50; // Reset active duration in ns
    parameter SIM_DURATION = 300; // **EXTENDED**: Total simulation duration in ns (30 cycles)
    
    reg clk;
    reg rst_n; // Active low reset

    // Wires for CPU-Memory Interface (These will be connected to the CPU top module's ports)
    wire [31:0] imem_addr_cpu;
    wire [31:0] imem_read_data_cpu;

    wire [31:0] dmem_addr_cpu;
    wire [31:0] dmem_write_data_cpu;
    wire        dmem_write_en_cpu;
    wire        dmem_read_en_cpu;
    wire [31:0] dmem_read_data_cpu;
    wire [3:0]  dmem_byte_enable_cpu;


    riscv_cpu_top uut_cpu (
        .clk              (clk),
        .rst_n            (rst_n),
        .mem_addr         (dmem_addr_cpu),
        .mem_write_data   (dmem_write_data_cpu),
        .mem_write_en     (dmem_write_en_cpu),
        .mem_read_en      (dmem_read_en_cpu),
        .mem_read_data    (dmem_read_data_cpu),
        .mem_byte_enable  (dmem_byte_enable_cpu),
        .imem_addr        (imem_addr_cpu),
        .imem_read_data   (imem_read_data_cpu)
    );

    instruction_memory uut_imem (
        .clk              (clk),
        .addr             (imem_addr_cpu), // Address from CPU's IF stage
        .instruction_out  (imem_read_data_cpu)
    );

    data_memory uut_dmem (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (dmem_addr_cpu),
        .write_data       (dmem_write_data_cpu),
        .write_en         (dmem_write_en_cpu),
        .read_en          (dmem_read_en_cpu),
        .byte_enable      (dmem_byte_enable_cpu),
        .read_data        (dmem_read_data_cpu)
    );

    
    always #((CLK_PERIOD)/2) clk = ~clk;

    initial begin
        // Initialize signals
        clk = 1'b0;
        rst_n = 1'b0; // Assert reset

        // Dump waveforms to VCD file
        $dumpfile("riscv_cpu.vcd");
        $dumpvars(0, riscv_cpu_tb);

        // Apply reset sequence
        #RESET_DURATION rst_n = 1'b1; // De-assert reset

        // Monitor and display pipeline state
        forever @(posedge clk) begin
            $display("--------------------------------------------------------------------------------");
            $display("TIME: %0t | Cycle: %0d", $time, ($time / CLK_PERIOD));
            $display("--------------------------------------------------------------------------------");

            // IF Stage
            $display("IF Stage: PC = 0x%H, Instruction = 0x%H", uut_cpu.if_stage_inst.program_counter, uut_cpu.imem_read_data);

            // IF/ID Pipeline Register (Output to ID)
            $display("IF/ID Reg: PC_out = 0x%H, PC+4_out = 0x%H, Instruction_out = 0x%H",
                     uut_cpu.if_id_reg_inst.id_pc_out, uut_cpu.if_id_reg_inst.id_pc_plus_4_out, uut_cpu.if_id_reg_inst.id_instruction_out);

            // ID Stage (Decoded - reading from id_stage_to_id_ex_reg_ wires to see current ID outputs)
            $display("ID Stage (Decoded): RS1=%0d (0x%H), RS2=%0d (0x%H), RD=%0d, Imm=0x%H",
                     uut_cpu.id_stage_to_id_ex_reg_rs1_addr, uut_cpu.reg_file_read_data1,
                     uut_cpu.id_stage_to_id_ex_reg_rs2_addr, uut_cpu.reg_file_read_data2,
                     uut_cpu.id_stage_to_id_ex_reg_rd_addr, uut_cpu.id_stage_to_id_ex_reg_immediate);
            $display("  Control Signals (from ID): RegWriteEn=%b, MemToReg=%b, ALUOp=%b, ALUSrcA=%b, ALUSrcB=%b, MemReadEn=%b, MemWriteEn=%b, Branch=%b, Jump=%b, JALR=%b",
                     uut_cpu.id_stage_to_id_ex_reg_write_en, uut_cpu.id_stage_to_id_ex_reg_mem_to_reg,
                     uut_cpu.id_stage_to_id_ex_reg_alu_op, uut_cpu.id_stage_to_id_ex_reg_alu_src_a, uut_cpu.id_stage_to_id_ex_reg_alu_src_b,
                     uut_cpu.id_stage_to_id_ex_reg_mem_read_en, uut_cpu.id_stage_to_id_ex_reg_mem_write_en,
                     uut_cpu.id_stage_to_id_ex_reg_branch, uut_cpu.id_stage_to_id_ex_reg_jump, uut_cpu.id_stage_to_id_ex_reg_jalr);
            $display("  ID Debug Opcode: %b", uut_cpu.id_stage_opcode_debug_wire); // Accessing the debug wire


            // ID/EX Pipeline Register (Output to EX)
            $display("ID/EX Reg (Output): PC+4=0x%H, RegData1=0x%H, RegData2=0x%H, Imm=0x%H",
                     uut_cpu.id_ex_pc_plus_4_out, uut_cpu.id_ex_reg_read_data1_out,
                     uut_cpu.id_ex_reg_read_data2_out, uut_cpu.id_ex_immediate_out);
            $display("  Control Signals (from ID/EX): RegWriteEn=%b, MemToReg=%b, ALUOp=%b, ALUSrcA=%b, ALUSrcB=%b, MemReadEn=%b, MemWriteEn=%b, Branch=%b, Jump=%b, JALR=%b",
                     uut_cpu.id_ex_reg_write_en_out, uut_cpu.id_ex_mem_to_reg_out,
                     uut_cpu.id_ex_alu_op_out, uut_cpu.id_ex_alu_src_a_out, uut_cpu.id_ex_alu_src_b_out,
                     uut_cpu.id_ex_mem_read_en_out, uut_cpu.id_ex_mem_write_en_out,
                     uut_cpu.id_ex_branch_out, uut_cpu.id_ex_jump_out, uut_cpu.id_ex_jalr_out);


            // EX Stage (Reading from ex_stage_to_ex_mem_reg_ wires to see current EX outputs)
            $display("EX Stage (ALU): OperandA=0x%H, OperandB=0x%H, Result=0x%H",
                     uut_cpu.ex_stage_inst.alu_operand_a, uut_cpu.ex_stage_inst.alu_operand_b,
                     uut_cpu.ex_stage_to_ex_mem_reg_alu_result); // Use the wire that will go to EX/MEM reg
            $display("  Branch Taken=%b, Branch Target=0x%H",
                     uut_cpu.ex_stage_to_ex_mem_reg_branch_taken, uut_cpu.ex_stage_to_ex_mem_reg_branch_target);

            // EX/MEM Pipeline Register (Output to MEM)
            $display("EX/MEM Reg (Output): PC+4=0x%H, ALU_Result=0x%H, RegData2=0x%H, RD_addr=%0d",
                     uut_cpu.ex_mem_pc_plus_4_out, uut_cpu.ex_mem_alu_result_out, uut_cpu.ex_mem_reg_read_data2_out,
                     uut_cpu.ex_mem_rd_addr_out);
            $display("  Control Signals (from EX/MEM): MemToReg=%b, MemReadEn=%b, MemWriteEn=%b, RegWriteEn=%b",
                     uut_cpu.ex_mem_mem_to_reg_out, uut_cpu.ex_mem_mem_read_en_out,
                     uut_cpu.ex_mem_mem_write_en_out, uut_cpu.ex_mem_reg_write_en_out);


            // MEM Stage
            $display("MEM Stage (Memory Access): Addr=0x%H, WriteData=0x%H, ReadData=0x%H, WriteEn=%b, ReadEn=%b",
                     uut_cpu.mem_addr, uut_cpu.mem_write_data, uut_cpu.mem_read_data,
                     uut_cpu.mem_write_en, uut_cpu.mem_read_en);
            $display("  Loaded Data=0x%H", uut_cpu.mem_stage_inst.load_data_out);

            // MEM/WB Pipeline Register (Output to WB)
            $display("MEM/WB Reg (Output): ALU_Result=0x%H, Load_Data=0x%H, RD_addr=%0d, RegWriteEn=%b, MemToReg=%b",
                     uut_cpu.mem_wb_reg_inst.wb_alu_result_out, uut_cpu.mem_wb_reg_inst.wb_load_data_out,
                     uut_cpu.mem_wb_reg_inst.wb_rd_addr_out, uut_cpu.mem_wb_reg_inst.wb_reg_write_en_out,
                     uut_cpu.mem_wb_reg_inst.wb_mem_to_reg_out);

            // Register File (WB Stage output writes directly here)
            $display("Register File Write: Enable=%b, Addr=%0d, Data=0x%H",
                     uut_cpu.reg_file_inst.write_en, uut_cpu.reg_file_inst.write_addr, uut_cpu.reg_file_inst.write_data);

            $display("--------------------------------------------------------------------------------\n");
        end
    end

    initial begin
        #SIM_DURATION; // Run simulation for specified duration

        // Print final register file state
        $display("\n================================================================================");
        $display("                          Simulation Finished");
        $display("================================================================================");
        $display("Final Register File State:");
        for (i = 0; i < 32; i = i + 1) begin
            $display("  x%0d (reg[%0d]): 0x%H", i, i, uut_cpu.reg_file_inst.registers[i]);
        end

        // // Print a portion of Data Memory
        // $display("\n--------------------------------------------------------------------------------");
        // $display("First 16 Words of Data Memory:");
        // for (i = 0; i < 16; i = i + 1) begin
        //     $display("  DMEM[0x%H]: 0x%H", i * 4, uut_dmem.dmem[i]);
        // end
        // $display("--------------------------------------------------------------------------------");

        $finish; // End simulation
    end

endmodule
