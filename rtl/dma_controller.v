`timescale 1ns/1ps
// =============================================================================
// dma_controller.v  –  Main DMA Transfer FSM
//
// Orchestrates one DMA channel transfer at a time (selected by the arbiter).
// For each channel it:
//   1. Loads source/destination addresses and byte count.
//   2. Validates the configuration (non-zero length, alignment).
//   3. Splits the total transfer into legal AXI4 bursts, honouring:
//        • Max burst length from BURST_CFG register (ARLEN value ≤ 255).
//        • AXI4 4 KB page boundary: a burst must not cross a 4 KB boundary.
//   4. Issues rd_start and wr_start concurrently to the burst engine.
//   5. Waits for rd_done and wr_done.
//   6. Repeats until all bytes are transferred.
//   7. Updates STATUS, signals the interrupt controller.
//   8. Releases the arbiter (xfer_done pulse).
//
// Error handling:
//   rd_error or wr_error from the burst engine → ERROR state →
//   error code recorded → error interrupt raised.
//
// Parameters match the top-level; all buses are flat (ch × W bits).
// =============================================================================
module dma_controller #(
    parameter NUM_CHANNELS = 2,
    parameter ADDR_WIDTH   = 32,
    parameter DATA_WIDTH   = 32,
    parameter FIFO_DEPTH   = 16
) (
    input  wire clk,
    input  wire rst_n,

    // -----------------------------------------------------------------------
    // From AXI4-Lite slave (config, one entry per channel – flat buses)
    // -----------------------------------------------------------------------
    input  wire [NUM_CHANNELS-1:0]        ch_enable,
    input  wire [NUM_CHANNELS-1:0]        ch_start,
    input  wire [NUM_CHANNELS-1:0]        ch_soft_reset,
    input  wire [32*NUM_CHANNELS-1:0]     ch_src_addr,
    input  wire [32*NUM_CHANNELS-1:0]     ch_dst_addr,
    input  wire [32*NUM_CHANNELS-1:0]     ch_xfer_len,
    input  wire [8*NUM_CHANNELS-1:0]      ch_burst_len,   // max ARLEN value

    // -----------------------------------------------------------------------
    // Status outputs to AXI4-Lite slave
    // -----------------------------------------------------------------------
    output reg  [NUM_CHANNELS-1:0]        ch_status_idle,
    output reg  [NUM_CHANNELS-1:0]        ch_status_busy,
    output reg  [NUM_CHANNELS-1:0]        ch_status_done,
    output reg  [NUM_CHANNELS-1:0]        ch_status_error,
    output reg  [8*NUM_CHANNELS-1:0]      ch_error_code,

    // -----------------------------------------------------------------------
    // Arbiter interface
    // -----------------------------------------------------------------------
    output wire [NUM_CHANNELS-1:0]        arb_request,
    input  wire [NUM_CHANNELS-1:0]        arb_grant,
    input  wire [$clog2(NUM_CHANNELS)-1:0] arb_grant_idx,
    input  wire                           arb_grant_valid,
    output reg                            arb_xfer_done,

    // -----------------------------------------------------------------------
    // Interrupt controller interface (one-cycle pulses)
    // -----------------------------------------------------------------------
    output reg  [NUM_CHANNELS-1:0]        irq_done,
    output reg  [NUM_CHANNELS-1:0]        irq_error,

    // -----------------------------------------------------------------------
    // Burst engine interface
    // -----------------------------------------------------------------------
    output reg  [ADDR_WIDTH-1:0]          be_rd_addr,
    output reg  [7:0]                     be_rd_len,
    output reg                            be_rd_start,
    input  wire                           be_rd_idle,

    output reg  [ADDR_WIDTH-1:0]          be_wr_addr,
    output reg  [7:0]                     be_wr_len,
    output reg                            be_wr_start,
    input  wire                           be_wr_idle,

    input  wire                           be_rd_done,
    input  wire                           be_wr_done,
    input  wire                           be_rd_error,
    input  wire                           be_wr_error
);

    // -----------------------------------------------------------------------
    // Local parameters
    // -----------------------------------------------------------------------
    localparam BEAT_BYTES    = DATA_WIDTH / 8;
    localparam BEAT_LOG2     = $clog2(BEAT_BYTES);     // e.g. 2 for 32-bit
    localparam CH_W          = $clog2(NUM_CHANNELS);

    // FSM state encoding
    localparam S_IDLE       = 4'd0;
    localparam S_ARB        = 4'd1;   // waiting for arbiter grant
    localparam S_LOAD       = 4'd2;   // latch channel config
    localparam S_VALIDATE   = 4'd3;   // alignment / length check
    localparam S_COMPUTE    = 4'd4;   // compute next burst parameters
    localparam S_BURST_INIT = 4'd5;   // issue rd_start + wr_start
    localparam S_BURST_WAIT = 4'd6;   // wait for both rd_done and wr_done
    localparam S_UPDATE     = 4'd7;   // advance addresses, decrement length
    localparam S_DONE       = 4'd8;   // finalize, pulse irq_done
    localparam S_ERROR      = 4'd9;   // record error, pulse irq_error

    // Error codes
    localparam ERR_ALIGN    = 8'h01;  // misaligned address or length
    localparam ERR_ZERO_LEN = 8'h02;  // zero transfer length
    localparam ERR_RD_RESP  = 8'h04;  // AXI read error response
    localparam ERR_WR_RESP  = 8'h08;  // AXI write error response

    // -----------------------------------------------------------------------
    // Per-channel pending request tracking
    // Request = enabled AND started, not yet active
    // -----------------------------------------------------------------------
    reg [NUM_CHANNELS-1:0] ch_pending;

    // Arbiter request: pending and not already done/error
    assign arb_request = ch_pending & ch_enable &
                         ~ch_status_busy & ~ch_status_done & ~ch_status_error;

    // -----------------------------------------------------------------------
    // Main FSM registers
    // -----------------------------------------------------------------------
    reg  [3:0]            state;
    reg  [CH_W-1:0]       active_ch;

    // Latched transfer parameters for the active channel
    reg  [ADDR_WIDTH-1:0] src_addr_cur;   // current source address
    reg  [ADDR_WIDTH-1:0] dst_addr_cur;   // current destination address
    reg  [31:0]           bytes_remain;
    reg  [7:0]            max_burst_len;  // from ch_burst_len register

    // Current burst parameters (computed in S_COMPUTE)
    reg  [7:0]            burst_arlen;    // ARLEN for this burst
    reg  [31:0]           burst_bytes;    // bytes in this burst

    // Completion flags for concurrent read + write
    reg                   rd_done_flag;
    reg                   wr_done_flag;

    // Error accumulation
    reg  [7:0]            err_code;

    // -----------------------------------------------------------------------
    // Burst length computation (combinational helpers)
    // -----------------------------------------------------------------------
    // beats_avail = floor(bytes_remain / BEAT_BYTES)
    wire [31:0] beats_avail = bytes_remain >> BEAT_LOG2;

    // beats until next 4 KB boundary (source side)
    wire [11:0] src_page_off  = src_addr_cur[11:0];
    wire [31:0] src_4kb_bytes = 32'd4096 - {20'd0, src_page_off};
    wire [31:0] src_4kb_beats = src_4kb_bytes >> BEAT_LOG2;

    // beats until next 4 KB boundary (destination side)
    wire [11:0] dst_page_off  = dst_addr_cur[11:0];
    wire [31:0] dst_4kb_bytes = 32'd4096 - {20'd0, dst_page_off};
    wire [31:0] dst_4kb_beats = dst_4kb_bytes >> BEAT_LOG2;

    // max_beats respects burst length register
    wire [31:0] max_beats = {24'd0, max_burst_len} + 32'd1;

    // Minimum of the four constraints (encoded as 32-bit comparisons)
    wire [31:0] min_avail_max  = (beats_avail  < max_beats)     ? beats_avail  : max_beats;
    wire [31:0] min_src_4kb   = (min_avail_max < src_4kb_beats) ? min_avail_max: src_4kb_beats;
    wire [31:0] burst_beats_w = (min_src_4kb   < dst_4kb_beats) ? min_src_4kb  : dst_4kb_beats;

    // -----------------------------------------------------------------------
    // Per-channel start capture
    // -----------------------------------------------------------------------
    integer c;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch_pending <= {NUM_CHANNELS{1'b0}};
        end else begin
            for (c = 0; c < NUM_CHANNELS; c = c + 1) begin
                if (ch_soft_reset[c]) begin
                    ch_pending[c] <= 1'b0;
                end else if (ch_start[c] && ch_enable[c]) begin
                    ch_pending[c] <= 1'b1;
                end else if (arb_grant[c] && (state == S_ARB)) begin
                    ch_pending[c] <= 1'b0;  // grant accepted, clear pending
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // Status register reset per-channel on soft-reset
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin : STATUS_RESET
        integer sc;
        if (!rst_n) begin
            ch_status_idle  <= {NUM_CHANNELS{1'b1}}; // all idle on reset
            ch_status_busy  <= {NUM_CHANNELS{1'b0}};
            ch_status_done  <= {NUM_CHANNELS{1'b0}};
            ch_status_error <= {NUM_CHANNELS{1'b0}};
            for (sc = 0; sc < NUM_CHANNELS; sc = sc + 1)
                ch_error_code[8*sc +: 8] <= 8'd0;
        end else begin
            for (sc = 0; sc < NUM_CHANNELS; sc = sc + 1) begin
                if (ch_soft_reset[sc]) begin
                    ch_status_idle [sc] <= 1'b1;
                    ch_status_busy [sc] <= 1'b0;
                    ch_status_done [sc] <= 1'b0;
                    ch_status_error[sc] <= 1'b0;
                    ch_error_code[8*sc +: 8] <= 8'd0;
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // Main FSM
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            active_ch     <= {CH_W{1'b0}};
            src_addr_cur  <= {ADDR_WIDTH{1'b0}};
            dst_addr_cur  <= {ADDR_WIDTH{1'b0}};
            bytes_remain  <= 32'd0;
            max_burst_len <= 8'h0F;
            burst_arlen   <= 8'd0;
            burst_bytes   <= 32'd0;
            rd_done_flag  <= 1'b0;
            wr_done_flag  <= 1'b0;
            err_code      <= 8'd0;
            arb_xfer_done <= 1'b0;
            be_rd_start   <= 1'b0;
            be_wr_start   <= 1'b0;
            be_rd_addr    <= {ADDR_WIDTH{1'b0}};
            be_wr_addr    <= {ADDR_WIDTH{1'b0}};
            be_rd_len     <= 8'd0;
            be_wr_len     <= 8'd0;
            irq_done      <= {NUM_CHANNELS{1'b0}};
            irq_error     <= {NUM_CHANNELS{1'b0}};
        end else begin
            // Default: clear one-cycle signals
            arb_xfer_done <= 1'b0;
            be_rd_start   <= 1'b0;
            be_wr_start   <= 1'b0;
            irq_done      <= {NUM_CHANNELS{1'b0}};
            irq_error     <= {NUM_CHANNELS{1'b0}};

            // Collect burst engine done flags (may arrive on different cycles)
            if (be_rd_done) rd_done_flag <= 1'b1;
            if (be_wr_done) wr_done_flag <= 1'b1;
            if (be_rd_error || be_wr_error) begin
                err_code <= be_rd_error ? ERR_RD_RESP : ERR_WR_RESP;
            end

            case (state)
                // -------------------------------------------------------
                S_IDLE: begin
                    if (arb_grant_valid) begin
                        state <= S_ARB;
                    end
                end

                // -------------------------------------------------------
                S_ARB: begin
                    // Wait one cycle for arbiter to settle, then load
                    if (arb_grant_valid) begin
                        active_ch <= arb_grant_idx;
                        state     <= S_LOAD;
                    end
                end

                // -------------------------------------------------------
                S_LOAD: begin
                    // Latch active channel's configuration
                    src_addr_cur  <= ch_src_addr [32*arb_grant_idx +: 32];
                    dst_addr_cur  <= ch_dst_addr [32*arb_grant_idx +: 32];
                    bytes_remain  <= ch_xfer_len [32*arb_grant_idx +: 32];
                    max_burst_len <= ch_burst_len [ 8*arb_grant_idx +:  8];
                    err_code      <= 8'd0;
                    rd_done_flag  <= 1'b0;
                    wr_done_flag  <= 1'b0;

                    // Mark channel busy
                    ch_status_idle [arb_grant_idx] <= 1'b0;
                    ch_status_busy [arb_grant_idx] <= 1'b1;
                    ch_status_done [arb_grant_idx] <= 1'b0;
                    ch_status_error[arb_grant_idx] <= 1'b0;

                    state <= S_VALIDATE;
                end

                // -------------------------------------------------------
                S_VALIDATE: begin
                    // Check: length > 0 and alignment
                    // Alignment: src, dst, len all must be BEAT_BYTES aligned
                    if (ch_xfer_len[32*active_ch +: 32] == 32'd0) begin
                        err_code <= ERR_ZERO_LEN;
                        state    <= S_ERROR;
                    end else if (|src_addr_cur[BEAT_LOG2-1:0] ||
                                 |dst_addr_cur[BEAT_LOG2-1:0] ||
                                 |bytes_remain[BEAT_LOG2-1:0]) begin
                        err_code <= ERR_ALIGN;
                        state    <= S_ERROR;
                    end else begin
                        state <= S_COMPUTE;
                    end
                end

                // -------------------------------------------------------
                S_COMPUTE: begin
                    // Compute burst parameters using combinational wires
                    // (burst_beats_w, src_4kb_beats, dst_4kb_beats, etc.)
                    burst_arlen <= burst_beats_w[7:0] - 8'd1;
                    burst_bytes <= burst_beats_w << BEAT_LOG2;
                    state       <= S_BURST_INIT;
                end

                // -------------------------------------------------------
                S_BURST_INIT: begin
                    // Issue read and write burst commands simultaneously
                    be_rd_addr  <= src_addr_cur;
                    be_rd_len   <= burst_arlen;
                    be_rd_start <= 1'b1;

                    be_wr_addr  <= dst_addr_cur;
                    be_wr_len   <= burst_arlen;
                    be_wr_start <= 1'b1;

                    rd_done_flag <= 1'b0;
                    wr_done_flag <= 1'b0;

                    state <= S_BURST_WAIT;
                end

                // -------------------------------------------------------
                S_BURST_WAIT: begin
                    // Accumulate done flags (may arrive in any order)
                    if (be_rd_done) rd_done_flag <= 1'b1;
                    if (be_wr_done) wr_done_flag <= 1'b1;

                    // Error check
                    if (be_rd_error || be_wr_error) begin
                        err_code <= be_rd_error ? ERR_RD_RESP : ERR_WR_RESP;
                        state    <= S_ERROR;
                    end else if ((rd_done_flag || be_rd_done) &&
                                 (wr_done_flag || be_wr_done)) begin
                        state <= S_UPDATE;
                    end
                end

                // -------------------------------------------------------
                S_UPDATE: begin
                    src_addr_cur <= src_addr_cur + burst_bytes;
                    dst_addr_cur <= dst_addr_cur + burst_bytes;
                    bytes_remain <= bytes_remain - burst_bytes;

                    if (bytes_remain <= burst_bytes) begin
                        // All bytes covered
                        state <= S_DONE;
                    end else begin
                        state <= S_COMPUTE;
                    end
                end

                // -------------------------------------------------------
                S_DONE: begin
                    ch_status_busy [active_ch] <= 1'b0;
                    ch_status_done [active_ch] <= 1'b1;
                    ch_status_idle [active_ch] <= 1'b1;
                    irq_done[active_ch]         <= 1'b1;
                    arb_xfer_done              <= 1'b1;
                    state                      <= S_IDLE;
                end

                // -------------------------------------------------------
                S_ERROR: begin
                    ch_status_busy [active_ch]      <= 1'b0;
                    ch_status_error[active_ch]      <= 1'b1;
                    ch_status_idle [active_ch]      <= 1'b1;
                    ch_error_code[8*active_ch +: 8] <= err_code;
                    irq_error[active_ch]            <= 1'b1;
                    arb_xfer_done                  <= 1'b1;
                    state                          <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // ---------------------------------------------------------------
            // Per-channel soft reset overrides FSM for that channel
            // If the active channel gets reset while busy, abort cleanly
            // ---------------------------------------------------------------
            if (ch_soft_reset[active_ch] && (state != S_IDLE)) begin
                state                         <= S_IDLE;
                ch_status_busy[active_ch]     <= 1'b0;
                ch_status_idle[active_ch]     <= 1'b1;
                arb_xfer_done                 <= 1'b1;  // release arbiter
            end
        end
    end

endmodule
