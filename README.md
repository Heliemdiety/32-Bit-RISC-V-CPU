32-Bit 5-Stage Pipelined RISC-V Processor with Custom ISA for Graph Acceleration
This repository contains the Verilog source code and analysis for a custom 32-bit RISC-V (RV32I) processor. The project features a classic 5-stage pipelined architecture and is enhanced with a custom Instruction Set Architecture (ISA) extension designed specifically to accelerate common graph traversal algorithm kernels.

The architecture, design, and performance analysis of this processor were the subject of a formal research paper submitted to a peer-reviewed IEEE conference.

🚀 Key Features
RV32I Base Instruction Set: Implements the full integer instruction set.

5-Stage Pipelined Architecture: Classic IF, ID, EX, MEM, WB pipeline for efficient instruction execution.

Full Hazard Detection & Forwarding: Includes a comprehensive hazard unit and data forwarding paths to mitigate data hazards and minimize stalls.

Custom ISA Extension: Features two novel instructions (UMIN and ADIFF) designed to accelerate bottlenecks in Dijkstra's, Prim's, and A* search algorithms.

Performance Gains: Achieves significant, projected speedups of up to 5x over optimized software baselines for specific kernels.

Verified Design: The core and custom instructions were validated using self-checking Verilog testbenches.

🏛️ Architecture
The processor is based on the classic 5-stage RISC pipeline. The design includes a full data forwarding unit to handle Read-After-Write (RAW) hazards and a stall unit to resolve load-use hazards, ensuring correct program execution.

✨ Custom ISA Extension
The core contribution of this project is a two-instruction ISA extension designed to accelerate graph algorithms. The instructions were integrated into the processor's Decode and Execute stages.

UMIN (Unsigned Minimum)
Syntax: umin rd, rs1, rs2

Semantics: rd ← min(rs1, rs2) (unsigned)

Purpose: Replaces a multi-instruction, branch-heavy software sequence for finding the minimum of two values. This is a core operation in the relaxation step of Dijkstra's algorithm and the key update step in Prim's algorithm.

ADIFF (Signed Absolute Difference)
Syntax: adiff rd, rs1, rs2

Semantics: rd ← |rs1 - rs2| (signed)

Purpose: Accelerates the calculation of heuristic functions, such as the Manhattan distance (|x1 - x2| + |y1 - y2|), which is a critical bottleneck in A search* and other pathfinding algorithms.

🔬 Performance Results
A rigorous, kernel-level analysis was performed to evaluate the performance of the custom instructions against optimized branching and branchless software baselines. The projected speedups, based on a cycle-accurate model of the pipeline (P=2), are significant.

Kernel	                Baseline Implementation        	Custom ISA Cycles	  Projected Speedup
Prim's Algorithm	    5-Cycle Branchless	                   1	              5.0x
A Heuristic*	        11-Cycle Branching                     3	              3.67x
Dijkstra's Algorithm	6-Cycle Branchless	                   2	              3.0x

Additionally, the custom instructions provide a 3x reduction in static code size for the A* kernel, improving instruction cache efficiency in memory-constrained embedded systems.

📝 Publication
Independent research paper submitted to IEEE INDICON 2025 (Under Review). The architecture, analytical model, and performance results of this project were detailed in a full research paper.

Title: Accelerating Graph Traversal Kernels with a Versatile, Lightweight RISC-V ISA Extension
