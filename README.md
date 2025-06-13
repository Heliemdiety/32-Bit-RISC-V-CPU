# 32-Bit-RISC-V-CPU
A 5-stage pipelined CPU implementing the RV32I instruction set architecture in Verilog. This project features a classic pipeline (IF, ID, EX, MEM, WB), a full register file, ALU, control unit, and advanced hazard detection with data forwarding. 

RISC-V RV32I Pipelined CPU Core 
Date: June 11, 2025

1. Project Overview
This project aims to implement a 5-stage pipelined RISC-V RV32I CPU core in Verilog. The CPU is designed to execute standard RV32I instructions, following a classic pipeline structure: Instruction Fetch (IF), Instruction Decode (ID), Execute (EX), Memory Access (MEM), and Write Back (WB). It incorporates a hazard detection unit for data and control hazards, and forwarding paths to minimize stalls.

2. Current Implemented Functionality
The CPU core successfully implements the following:

5-Stage Pipeline: All five stages (IF, ID, EX, MEM, WB) are instantiated and connected.

Pipeline Registers: if_id_reg, id_ex_reg, ex_mem_reg, and mem_wb_reg are in place, correctly latching data and control signals between stages.

PC Management: Program Counter (PC) incrementing and basic branch/jump target calculation.

Instruction Fetch (IF) Stage: Fetches instructions from instruction memory.

Instruction Decode (ID) Stage: Decodes RV32I instruction opcodes, funct3, funct7.
Generates immediate values (I, S, B, U, J types).
Reads operand data from the Register File.

Control Unit: Generates all necessary control signals for subsequent pipeline stages based on the instruction opcode.

Execute (EX) Stage:Performs ALU operations.Calculates branch targets.Forwarding Unit: Implements data forwarding from EX/MEM and MEM/WB stages to resolve data hazards.

Memory Access (MEM) Stage:Handles load and store operations with external data memory interface.

Write Back (WB) Stage: Writes results back to the Register File.

Register File: A 32-entry, 32-bit register file with two read ports and one write port.

Hazard Detection Unit: Detects data hazards (load-use) and control hazards (branches/jumps) and generates stall and flush signals.

Observations on Functionality:Based on simulation logs, R-type (e.g., add, sub), I-type (e.g., addi), and U-type (e.g., auipc) instructions appear to be executing and writing results to the register file correctly, demonstrating the successful flow of data through the pipeline and the effective operation of ALU and forwarding.

3. Identified Critical Problem: MemWriteEn Discrepancy
A persistent and unexplainable issue has been observed regarding the MemWriteEn control signal for Store Word (SW) instructions, specifically when it propagates through the ID/EX pipeline register.

Symptom:
During the cycle where an SW instruction's control signals enter the id_ex_reg:
The ID Stage (and its internal control_unit) correctly outputs MemWriteEn = 1.
The Hazard Detection Unit confirms that stall_id = 0 and flush_id = 0 during this cycle.
However, the id_ex_reg outputs MemWriteEn = 0 to the EX Stage.
Key Debug Evidence:
ID/EX_REG Debug (internal to id_ex_reg.v): Consistently reported Input MemWriteEn=1, Output MemWriteEn=0, Stall=0, Flush=0 for the relevant clock cycle.
RISCV_CPU_TOP Debug (newly added to riscv_cpu_top.v): Confirmed that the stall_id and flush_id wires entering the id_ex_reg instance were indeed 0 at the precise clock edge when the MemWriteEn discrepancy occurred.
Isolated id_ex_reg Testbench: A separate, minimalist testbench for id_ex_reg.v demonstrated that the module correctly passes MemWriteEn=1 to its output when stall=0 and flush=0. This test confirms the id_ex_reg module's Verilog logic is sound and functional in isolation.


4. Conclusion on the Problem Source
Given the following facts:
The id_ex_reg module's Verilog code is correct and works in isolation.
All monitored inputs to the id_ex_reg (including MemWriteEn=1, stall=0, flush=0) are confirmed to be in the correct state at the moment of the clock edge within the full CPU simulation.
Despite confirmed correct inputs, the id_ex_reg still outputs MemWriteEn=0.
This strongly indicates an anomaly or bug within the ModelSim simulator environment itself, rather than a design flaw in the Verilog code. It suggests a caching issue, a corrupted installation, or a rare tool-specific bug that prevents the simulator from correctly evaluating the register's behavior within the larger project context, even after clean compiles and restarts.


6. Key Learnings from this Debugging Process

Methodical Debugging: The importance of systematically tracing signals, identifying contradictions, and using targeted debug prints ($display statements) at different levels of hierarchy.

Isolation Testing: The power of isolating individual modules to verify their correctness in a controlled environment, proving or disproving internal logic errors.

Understanding Pipeline Flow: Deepened understanding of how control signals and data propagate through a pipelined CPU.

Hazard Detection: Gained experience in implementing and debugging hazard detection logic.

Simulator Anomalies: Learning that sometimes, the problem lies outside the code itself, requiring consideration of the development environment.


DEBUGGING JOURNEY :-

1) The pc and instructions were not updating. Due to my previous experience with muon detector, I suspected there could be a problem with the delay, so I set the delay to #10000. Still, I got no significant results.

2)Then I created testbench for each stage to check where my pc is getting stuck, 
IF stage -- passed,
Id stage -- passed 
Now after confirming these two I wondered, if these 2 stages are working independently then why my PC and instr are still showing xxxxx in final output when I use it with my topmodule and final testbench. That meant the problem was not in the individual stages.

3) I printed the stall, clock and other parameters in testbench to check their functioning and found that stall = x. Stall was not getting any value.  So I temporarily removed the hazard unit completely from top module and testbench, force assigned stall = 0, to get some output and  --- woahhlaaa --- I got my PC and instr output from each stage .
But the register files are still 00000000.
 
THAT means my hazard logic is not good and I need to improve it…. 
	
4)  Created a tb for WB stage to check the register issues .. Wb stage is working fine seperately and is generating register values. 
5) Wrote the code again from scratch ,, but still the register file = 0 ….. 


6))Created an entirely new version with new wires. One issue I noted was I was using same wires to drive multiple inputs/outputs. After using separating them, I am able  to get register values ,PC,Instructions etc.
Analysis of the Latest Simulation Log
identified that x1, x2, x3, and x4 are getting their correct values, which means the Instruction Fetch, Decode, Execute, and Register File write-back stages are working correctly for basic arithmetic instructions.

7)) common problems I am facing --
port mismatching ,, eg,-- connecting 2 bit ports with 1 bit port.
Not handling the wire connection whenever I am declaring the new wire 


8)) Since I know register 1 to 5 are having values I am sure that pipeline stages are working properly. I can see that data memory is not holding anything in my simulation. So basically the problem might be in the data memory.
My memwrite_en is also 0 for all the cycles , so no data is being written, to solve this I added debug statements to  check whether the hazard unit is interfering. These debug logs confirmed stall and flush were functioning as expected.

9)) The problem is not due to hazard unit, I can see that at time 11500 my memwrite_en is 1 as input ,, but 0 as output..
I have explicitly assigned the output value equal to Input.
The only way for Input MemWriteEn=1 to result in Output MemWriteEn=0 when Stall=0 and Flush=0 is either modelSim is not working properly here or I am too dumb to understand it further.

10 ))For one last try I ran a testbench for ID-EX stage separately, and it worked fine as expected.

