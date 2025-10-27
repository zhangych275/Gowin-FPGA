// Simple asynchronous (dual-clock) FIFO
// Data width 12, depth 4096 (addr width 12).
// Reset is active-high (port rst). This matches analog_signal_acquisition usage:
// .rst(!rst_n)  -> rst will be 1 when system reset is asserted.
//
// NOTE: This is a generic, simulation-friendly asynchronous FIFO with
// two-stage synchronization for pointers. For production use verify timing,
// metastability requirements, and replace with vendor-provided hardened FIFO
// if needed.
module fifo_12x4096 (
    input               wr_clk,    // write clock domain
    input               rd_clk,    // read clock domain
    input               rst,       // active-high reset for FIFO internal logic
    input  [11:0]       din,       // data in
    input               wr_en,     // write enable (in wr_clk domain)
    input               rd_en,     // read enable (in rd_clk domain)
    output reg [11:0]   dout,      // data out (in rd_clk domain)
    output              full,      // full flag (in wr_clk domain)
    output              empty      // empty flag (in rd_clk domain)
);

localparam ADDR_WIDTH = 12;
localparam DEPTH = 1 << ADDR_WIDTH;

// memory
reg [11:0] mem [0:DEPTH-1];

// write pointers (binary and gray)
reg [ADDR_WIDTH-1:0] wr_ptr_bin;
reg [ADDR_WIDTH-1:0] rd_ptr_bin_wr_sync; // synchronized read ptr into write clock domain
reg [ADDR_WIDTH-1:0] wr_ptr_bin_next;
reg [ADDR_WIDTH-1:0] wr_ptr_gray;
reg [ADDR_WIDTH-1:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;
reg [ADDR_WIDTH-1:0] rd_ptr_bin_from_gray;

// read pointers (binary and gray)
reg [ADDR_WIDTH-1:0] rd_ptr_bin;
reg [ADDR_WIDTH-1:0] wr_ptr_bin_rd_sync; // synchronized write ptr into read clock domain
reg [ADDR_WIDTH-1:0] rd_ptr_gray;
reg [ADDR_WIDTH-1:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;
reg [ADDR_WIDTH-1:0] wr_ptr_bin_from_gray;

// Gray code conversion functions
function [ADDR_WIDTH-1:0] bin2gray;
    input [ADDR_WIDTH-1:0] bin;
    integer i;
    begin
        bin2gray = (bin >> 1) ^ bin;
    end
endfunction

function [ADDR_WIDTH-1:0] gray2bin;
    input [ADDR_WIDTH-1:0] gray;
    integer j;
    reg [ADDR_WIDTH-1:0] b;
    begin
        b = gray;
        for (j = 1; j < ADDR_WIDTH; j = j + 1) begin
            b = b ^ (gray >> j);
        end
        gray2bin = b;
    end
endfunction

// write domain: synchronize rd_ptr_gray into write clock domain (2-stage)
always @(posedge wr_clk or posedge rst) begin
    if (rst) begin
        rd_ptr_gray_sync1 <= {ADDR_WIDTH{1'b0}};
        rd_ptr_gray_sync2 <= {ADDR_WIDTH{1'b0}};
    end else begin
        rd_ptr_gray_sync1 <= rd_ptr_gray;
        rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
    end
end

// convert synchronized gray to binary for full calculation
always @(*) begin
    rd_ptr_bin_from_gray = gray2bin(rd_ptr_gray_sync2);
end

// write pointer update and memory write
always @(posedge wr_clk or posedge rst) begin
    if (rst) begin
        wr_ptr_bin <= {ADDR_WIDTH{1'b0}};
        wr_ptr_gray <= {ADDR_WIDTH{1'b0}};
    end else begin
        if (wr_en) begin
            // Only write if not full. The user should gate wr_en by full check.
            mem[wr_ptr_bin] <= din;
            wr_ptr_bin <= wr_ptr_bin + 1'b1;
            wr_ptr_gray <= bin2gray(wr_ptr_bin + 1'b1);
        end
    end
end

// full detection in write clock domain:
// FIFO full when next write pointer equals read pointer synchronized with MSB inverted
// Equivalent typical check with gray pointers but here we convert to binary and compare:
assign full = ( (wr_ptr_bin + (wr_en ? 1'b1 : 1'b0)) == rd_ptr_bin_from_gray );

// read domain: synchronize wr_ptr_gray into read clock domain (2-stage)
always @(posedge rd_clk or posedge rst) begin
    if (rst) begin
        wr_ptr_gray_sync1 <= {ADDR_WIDTH{1'b0}};
        wr_ptr_gray_sync2 <= {ADDR_WIDTH{1'b0}};
    end else begin
        wr_ptr_gray_sync1 <= wr_ptr_gray;
        wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
    end
end

// convert synchronized gray to binary for empty calculation
always @(*) begin
    wr_ptr_bin_from_gray = gray2bin(wr_ptr_gray_sync2);
end

// read pointer update and memory read
always @(posedge rd_clk or posedge rst) begin
    if (rst) begin
        rd_ptr_bin <= {ADDR_WIDTH{1'b0}};
        rd_ptr_gray <= {ADDR_WIDTH{1'b0}};
        dout <= 12'd0;
    end else begin
        if (rd_en && (rd_ptr_bin != wr_ptr_bin_from_gray)) begin
            dout <= mem[rd_ptr_bin];
            rd_ptr_bin <= rd_ptr_bin + 1'b1;
            rd_ptr_gray <= bin2gray(rd_ptr_bin + 1'b1);
        end
    end
end

// empty flag: empty when read pointer equals synchronized write pointer
assign empty = (rd_ptr_bin == wr_ptr_bin_from_gray);

// keep external visible copies of gray pointers for cross sync
// wr_ptr_gray is updated in write always block already
// rd_ptr_gray is updated in read always block already

endmodule