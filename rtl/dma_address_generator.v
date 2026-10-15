`timescale 1ns/1ps
// =============================================================================
// dma_address_generator.v  –  Sequential Address Generator
//
// Generates word-aligned addresses for DMA source or destination.
// Increments by BEAT_BYTES (= DATA_WIDTH/8) after every accepted beat.
// Checks that both the base address and the transfer length are naturally
// aligned to the beat size (e.g., 4-byte alignment for a 32-bit bus).
//
// Parameters:
//   ADDR_WIDTH : address bus width (default 32)
//   DATA_WIDTH : data bus width   (default 32)
// =============================================================================
module dma_address_generator #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Load new transfer descriptor (level, sampled on rising edge)
    input  wire [ADDR_WIDTH-1:0] base_addr,
    input  wire [31:0]           xfer_len,     // transfer size in bytes
    input  wire                  load,         // assert for one cycle to latch

    // Progress – assert beat_done for one cycle per accepted AXI beat
    input  wire                  beat_done,

    // Outputs
    output reg  [ADDR_WIDTH-1:0] current_addr,
    output reg  [31:0]           bytes_remain,
    output wire                  xfer_done,    // all bytes delivered
    output wire                  align_error   // misaligned addr or length
);

    // Beat size (bytes per AXI data transfer)
    localparam BEAT_BYTES     = DATA_WIDTH / 8;
    localparam ALIGN_BITS     = $clog2(BEAT_BYTES);

    // Alignment error: address or length not a multiple of BEAT_BYTES
    assign align_error = (|base_addr[ALIGN_BITS-1:0]) || (|xfer_len[ALIGN_BITS-1:0]);

    // Transfer complete when no bytes remain
    assign xfer_done = (bytes_remain == 32'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_addr <= {ADDR_WIDTH{1'b0}};
            bytes_remain <= 32'd0;
        end else if (load) begin
            current_addr <= base_addr;
            bytes_remain <= xfer_len;
        end else if (beat_done && !xfer_done) begin
            current_addr <= current_addr + BEAT_BYTES[ADDR_WIDTH-1:0];
            bytes_remain <= bytes_remain - BEAT_BYTES[31:0];
        end
    end

endmodule
