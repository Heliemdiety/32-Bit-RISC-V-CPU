// Supports word-aligned reads and writes.

module data_memory (
    input wire clk,
    input wire rst_n, // Active low reset

    input wire [31:0] addr,           // Byte address
    input wire [31:0] write_data,
    input wire        write_en,        // write enable
    input wire        read_en,        // Read enable
    input wire [3:0]  byte_enable,    // 4-bit byte enable for partial word writes
    output reg [31:0] read_data
);

    // Memory declaration: 256 words (1KB)
    reg [31:0] dmem [0:255]; // 256 words = 1024 bytes (1KB)

    // Initialize data memory
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            dmem[i] = 32'hDEADBEEF; // Distinct value to see if memory is read
        end
        $display("    DMEM Initialized: All locations set to 0xDEADBEEF");
    end

    // Write operation
    always @(posedge clk) begin
        if (!rst_n) begin
            // No action on write_en during reset.
        end else if (write_en) begin.
            if (byte_enable == 4'b1111) begin // Full word write 
                dmem[addr[31:2]] <= write_data;
                $display("    DMEM Write: Time=%0t, Addr=0x%H, Data=0x%H", $time, addr, write_data);
            end
        end
    end

    // Read operation (synchronous read)
    always @(posedge clk) begin
        if (!rst_n) begin
            read_data <= 32'b0; // Output zero on reset
        end else if (read_en) begin
            read_data <= dmem[addr[31:2]];
            $display("    DMEM Read: Time=%0t, Addr=0x%H, Data=0x%H", $time, addr, dmem[addr[31:2]]);
        end else begin
            read_data <= 32'b0; // Default when not enabled to read
        end
    end

endmodule

