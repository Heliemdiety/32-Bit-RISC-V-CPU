// Simple synchronous single-port instruction memory (ROM).
// Contains a pre-loaded test program in RISC-V machine code.

module instruction_memory (
    input wire clk,
    input wire [31:0] addr, // Byte address (word aligned)
    output reg [31:0] instruction_out
);

    reg [31:0] imem [0:255]; // 256 words = 1024 bytes (1KB)

    // Initialization of  instruction memory
    // basic program to demonstrate functionality:
    // 0x0000:  li x1, 10           (ADDI x1, x0, 10) -> 0x00A00093
    // 0x0004:  li x2, 20           (ADDI x2, x0, 20) -> 0x01400113
    // 0x0008:  add x3, x1, x2      (ADD x3, x1, x2)  -> 0x002081B3
    // 0x000C:  addi x4, x3, 5      (ADDI x4, x3, 5)  -> 0x00518213
    // 0x0010:  sw x4, 0(x0)        (SW x4, 0(x0))    -> 0x00402023 (Store x4 at address 0x0)
    // 0x0014:  lw x5, 0(x0)        (LW x5, 0(x0))    -> 0x00002283 (Load from address 0x0 into x5)
    // 0x0018:  beq x5, x3, loop    (BEQ x5, x3, target) -> 0x00328463 (if x5 == x3, branch to loop)
    // 0x001C:  addi x6, x0, 1      (ADDI x6, x0, 1)  -> 0x00100300 (NOP-like, for pipeline fill)
    // 0x0020: loop: addi x7, x0, 2 (ADDI x7, x0, 2)  -> 0x00200380
    // 0x0024:  jal x0, 0x0024      (JAL x0, 0)       -> 0x0000006F (Infinite loop/halt)

    initial begin
        // Program instructions (word-aligned addresses)
        imem[0]  = 32'h00a00093; 
        imem[1]  = 32'h01400113; 
        imem[2]  = 32'h002081b3; 
        imem[3]  = 32'h00518213; 
        imem[4]  = 32'h00402023; 
        imem[5]  = 32'h00002283; 
        imem[6]  = 32'h00328463;                         
        imem[7]  = 32'h00100300; 
        imem[8]  = 32'h00200380; 
        imem[9]  = 32'h0000006f; 
        
    end

    // Read instruction from memory based on word-aligned address
    always @(posedge clk) begin
        instruction_out <= imem[addr[31:2]]; // addr[31:2] gives word address
    end

endmodule
