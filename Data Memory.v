// Data Memory for the RISC-V CPU.
// Supports word-aligned reads and writes.
// **IMPROVEMENT**: Added support for byte and half-word writes using byte_enable.

module data_memory (
    input wire clk,
    input wire rst_n, // Active low reset

    input wire [31:0] addr,         // Byte address
    input wire [31:0] write_data,
    input wire        write_en,     // Write enable
    input wire        read_en,      // Read enable
    input wire [3:0]  byte_enable,  // 4-bit byte enable for partial word writes (LSB is byte 0)
    output reg [31:0] read_data
);

    // Memory declaration: 256 words (1KB total).
    // Each element `dmem[i]` holds a 32-bit word.
    reg [31:0] dmem [0:255]; // 256 words = 1024 bytes (1KB)

    // Initialize data memory content.
    // This runs once at the start of simulation.
    integer i;
    reg [31:0] current_word;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            dmem[i] = 32'hDEADBEEF; // Initialize with a distinct value for easier debugging
        end
        $display("    DMEM Initialized: All locations set to 0xDEADBEEF");
    end

    // Write operation: Synchronous write
    // This block executes on the positive edge of the clock.
    always @(posedge clk) begin
        if (!rst_n) begin
            // On reset, no write operation occurs.
        end else if (write_en) begin
            // Calculate the word address (addr[31:2] effectively divides by 4)
            // and the byte offset within the word (addr[1:0]).
            
            current_word = dmem[addr[31:2]]; // Read current word content for partial writes

            // Apply byte enables for partial word writes.
            // This logic allows for SB, SH, and SW operations by masking
            // and inserting the correct bytes from write_data.
            if (byte_enable[0]) current_word[7:0]   = write_data[7:0];   // Byte 0
            if (byte_enable[1]) current_word[15:8]  = write_data[15:8];  // Byte 1
            if (byte_enable[2]) current_word[23:16] = write_data[23:16]; // Byte 2
            if (byte_enable[3]) current_word[31:24] = write_data[31:24]; // Byte 3

            dmem[addr[31:2]] <= current_word; // Write the modified word back

            $display("    DMEM Write: Time=%0t, Addr=0x%H, Data=0x%H, ByteEnable=%b", $time, addr, write_data, byte_enable);
        end
    end

    // Read operation: Synchronous read
    // This block executes on the positive edge of the clock.
    always @(posedge clk) begin
        if (!rst_n) begin
            read_data <= 32'b0; // Output zero on reset
        end else if (read_en) begin
            // Read the full word from memory.
            // The `mem_stage` will handle extracting and sign-extending the correct byte/half-word.
            read_data <= dmem[addr[31:2]];
            $display("    DMEM Read: Time=%0t, Addr=0x%H, Data=0x%H", $time, addr, dmem[addr[31:2]]);
        end else begin
            read_data <= 32'b0; // Default output when not enabled to read
        end
    end

endmodule
