// Register File for the RISC-V CPU.
// Provides two read ports for source operands and one write port for destination operand.
// Register x0 (index 0) always reads as 0 and cannot be written.
// Now receives write inputs directly from a dedicated WB stage.

module reg_file (
    input wire clk,             // Clock signal
    input wire rst_n,           // Active low reset

    // Read Ports (inputs from ID stage)
    input wire [4:0] read_addr1,  // Address for first read port (rs1)
    input wire [4:0] read_addr2,  // Address for second read port (rs2)
    output wire [31:0] read_data1, // Data from first read port
    output wire [31:0] read_data2, // Data from second read port

    // Write Port (inputs from WB stage)
    input wire        write_en,   // Write enable signal from WB stage
    input wire [4:0]  write_addr,   // Address for write port from WB stage
    input wire [31:0] write_data    // Data to write to register from WB stage
);

    // Declare the array of 32 registers, each 32-bits wide.
    reg [31:0] registers [0:31];
    integer i;

    // Synchronous write operation and asynchronous reset.
    // This block updates the register content on the positive edge of the clock.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // On an active low reset, clear all registers to zero.
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
            $display("    REG_FILE Initialized: All registers set to 0x0 at time %0t", $time);
        end else begin
            // Write operation: Only occurs if 'write_en' is high
            // AND the destination register is not x0 (register 0).
            // Register x0 is hardwired to zero and cannot be modified.
            if (write_en && (write_addr != 5'b0)) begin
                registers[write_addr] <= write_data;
                $display("    REG_FILE Write: Time=%0t, Addr=%d (R%d), Data=0x%H", $time, write_addr, write_addr, write_data);
            end
        end
    end

    // Combinational read operation.
    // Reading from register x0 (address 5'b0) always returns 0.
    assign read_data1 = (read_addr1 == 5'b0) ? 32'b0 : registers[read_addr1];
    assign read_data2 = (read_addr2 == 5'b0) ? 32'b0 : registers[read_addr2];

endmodule

