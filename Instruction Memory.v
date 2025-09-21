// Instruction Memory for the RISC-V CPU.
// Stores the program instructions and provides them to the IF stage.
// Initialized from an external HEX file for flexible program loading.

module instruction_memory (
    input wire clk,                  // Clock signal
    input wire [31:0] addr,          // Byte address (word aligned: addr[31:2] gives word address)
    output reg [31:0] instruction_out // 32-bit instruction read from memory
);

    reg [31:0] imem [0:255]; // 256 words = 1024 bytes (1KB)

    initial begin
        
        $readmemh("publication.hex", imem);
        $display("INFO: instruction_memory: 'inst_mem.hex' loaded successfully at time %0t", $time);
    end

    always @(posedge clk) begin
        instruction_out <= imem[addr[31:2]];
    end

endmodule

