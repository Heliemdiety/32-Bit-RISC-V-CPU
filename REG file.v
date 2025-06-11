// Register File for the RISC-V CPU.
// Implements 32 x 32-bit general-purpose registers (x0-x31).
// - x0 is hardwired to zero and cannot be written.
// - Supports two read ports (for rs1 and rs2).
// - Supports one write port (for rd).

module reg_file (
    input wire clk,
    input wire rst_n, // Active low reset

    // Read Ports
    input wire [4:0] read_addr1,  // Address for first read port (rs1)
    input wire [4:0] read_addr2,  // Address for second read port (rs2)
    output wire [31:0] read_data1, // Data from first read port
    output wire [31:0] read_data2, // Data from second read port

    // Write Port
    input wire        write_en,   // Write enable signal
    input wire [4:0]  write_addr,   // Address for write port (rd)
    input wire [31:0] write_data    // Data to write to register
);

    // Declare the array of registers
    reg [31:0] registers [0:31];

    // Initialize registers on reset
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else begin
            // Write operation
            // Only write if write_en is high and write_addr is not x0 (register 0)
            if (write_en && (write_addr != 5'b0)) begin
                registers[write_addr] <= write_data;
            end
        end
    end

    // Read operation
    assign read_data1 = (read_addr1 == 5'b0) ? 32'b0 : registers[read_addr1];
    assign read_data2 = (read_addr2 == 5'b0) ? 32'b0 : registers[read_addr2];

endmodule
