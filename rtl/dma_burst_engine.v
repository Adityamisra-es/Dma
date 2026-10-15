`timescale 1ns/1ps
// =============================================================================
// dma_burst_engine.v  –  AXI4 Burst Read/Write Engine
//
// Contains two independent state machines:
//   • Read FSM  : AR channel → R channel → FIFO write
//   • Write FSM : AW channel → W channel (from FIFO) → B channel
//
// The read and write FSMs run concurrently; the FIFO provides elastic
// buffering between them.  The controller issues rd_start / wr_start pulses
// with pre-computed burst addresses and lengths.
//
// 4 KB boundary enforcement is done by the CALLER (dma_controller) which
// must supply a burst_len that does not cross a 4 KB page.
//
// AXI4 burst type is always INCR (2'b01).
// Beat size is always log2(DATA_WIDTH/8).
// LAST is asserted on the final beat of each burst.
//
// Backpressure:
//   • Read: RREADY deasserted when FIFO is full → read stalls.
//   • Write: WVALID deasserted when FIFO is empty → write stalls.
// =============================================================================
module dma_burst_engine #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 4
) (
    input  wire clk,
    input  wire rst_n,

    // -----------------------------------------------------------------------
    // Read burst command (one-cycle pulse on rd_start)
    // -----------------------------------------------------------------------
    input  wire [ADDR_WIDTH-1:0]   rd_addr,
    input  wire [7:0]              rd_len,    // ARLEN (beats − 1)
    input  wire                    rd_start,
    output wire                    rd_idle,   // high when read FSM is idle

    // -----------------------------------------------------------------------
    // Write burst command (one-cycle pulse on wr_start)
    // -----------------------------------------------------------------------
    input  wire [ADDR_WIDTH-1:0]   wr_addr,
    input  wire [7:0]              wr_len,    // AWLEN (beats − 1)
    input  wire                    wr_start,
    output wire                    wr_idle,   // high when write FSM is idle

    // -----------------------------------------------------------------------
    // FIFO interface  (burst engine is producer for RD, consumer for WR)
    // -----------------------------------------------------------------------
    output wire [DATA_WIDTH-1:0]   fifo_wr_data,
    output wire                    fifo_wr_en,
    input  wire                    fifo_full,

    input  wire [DATA_WIDTH-1:0]   fifo_rd_data,
    output wire                    fifo_rd_en,
    input  wire                    fifo_empty,

    // -----------------------------------------------------------------------
    // AXI4 Master – Read Address Channel
    // -----------------------------------------------------------------------
    output reg  [ID_WIDTH-1:0]     m_axi_arid,
    output reg  [ADDR_WIDTH-1:0]   m_axi_araddr,
    output reg  [7:0]              m_axi_arlen,
    output reg  [2:0]              m_axi_arsize,
    output reg  [1:0]              m_axi_arburst,
    output reg                     m_axi_arvalid,
    input  wire                    m_axi_arready,

    // -----------------------------------------------------------------------
    // AXI4 Master – Read Data Channel
    // -----------------------------------------------------------------------
    input  wire [ID_WIDTH-1:0]     m_axi_rid,
    input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output reg                     m_axi_rready,

    // -----------------------------------------------------------------------
    // AXI4 Master – Write Address Channel
    // -----------------------------------------------------------------------
    output reg  [ID_WIDTH-1:0]     m_axi_awid,
    output reg  [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output reg  [7:0]              m_axi_awlen,
    output reg  [2:0]              m_axi_awsize,
    output reg  [1:0]              m_axi_awburst,
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,

    // -----------------------------------------------------------------------
    // AXI4 Master – Write Data Channel
    // -----------------------------------------------------------------------
    output reg  [DATA_WIDTH-1:0]   m_axi_wdata,
    output reg  [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg                     m_axi_wlast,
    output reg                     m_axi_wvalid,
    input  wire                    m_axi_wready,

    // -----------------------------------------------------------------------
    // AXI4 Master – Write Response Channel
    // -----------------------------------------------------------------------
    input  wire [ID_WIDTH-1:0]     m_axi_bid,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output reg                     m_axi_bready,

    // -----------------------------------------------------------------------
    // Status  (one-cycle pulses)
    // -----------------------------------------------------------------------
    output reg                     rd_done,
    output reg                     wr_done,
    output reg                     rd_error,   // RRESP != OKAY
    output reg                     wr_error    // BRESP != OKAY
);

    localparam AXI_BURST_INCR = 2'b01;
    localparam AXI_RESP_OKAY  = 2'b00;
    localparam AXI_SIZE       = $clog2(DATA_WIDTH/8); // e.g., 2 for 32-bit

    // -----------------------------------------------------------------------
    // Read FSM
    // -----------------------------------------------------------------------
    localparam RD_IDLE  = 2'd0;
    localparam RD_ADDR  = 2'd1;
    localparam RD_DATA  = 2'd2;
    localparam RD_DONE  = 2'd3;

    reg [1:0]  rd_state;
    reg [7:0]  rd_beats_left;   // beats still expected from slave

    assign rd_idle = (rd_state == RD_IDLE);

    // FIFO write side driven by read engine
    assign fifo_wr_data = m_axi_rdata;
    assign fifo_wr_en   = m_axi_rvalid && m_axi_rready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state       <= RD_IDLE;
            rd_beats_left  <= 8'd0;
            m_axi_arid     <= {ID_WIDTH{1'b0}};
            m_axi_araddr   <= {ADDR_WIDTH{1'b0}};
            m_axi_arlen    <= 8'd0;
            m_axi_arsize   <= 3'd0;
            m_axi_arburst  <= 2'b01;
            m_axi_arvalid  <= 1'b0;
            m_axi_rready   <= 1'b0;
            rd_done        <= 1'b0;
            rd_error       <= 1'b0;
        end else begin
            rd_done  <= 1'b0;
            rd_error <= 1'b0;

            case (rd_state)
                RD_IDLE: begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    if (rd_start) begin
                        m_axi_araddr  <= rd_addr;
                        m_axi_arlen   <= rd_len;
                        m_axi_arsize  <= AXI_SIZE[2:0];
                        m_axi_arburst <= AXI_BURST_INCR;
                        m_axi_arid    <= {ID_WIDTH{1'b0}};
                        m_axi_arvalid <= 1'b1;
                        rd_beats_left <= rd_len;
                        rd_state      <= RD_ADDR;
                    end
                end

                RD_ADDR: begin
                    if (m_axi_arready && m_axi_arvalid) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1; // ready to accept data (may be throttled)
                        rd_state      <= RD_DATA;
                    end
                end

                RD_DATA: begin
                    // Backpressure: if FIFO full, deassert RREADY
                    m_axi_rready <= !fifo_full;

                    if (m_axi_rvalid && m_axi_rready) begin
                        if (m_axi_rresp != AXI_RESP_OKAY)
                            rd_error <= 1'b1;

                        if (m_axi_rlast) begin
                            m_axi_rready <= 1'b0;
                            rd_state     <= RD_DONE;
                        end else begin
                            rd_beats_left <= rd_beats_left - 8'd1;
                        end
                    end
                end

                RD_DONE: begin
                    rd_done  <= 1'b1;
                    rd_state <= RD_IDLE;
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Write FSM
    // -----------------------------------------------------------------------
    localparam WR_IDLE  = 3'd0;
    localparam WR_ADDR  = 3'd1;
    localparam WR_DATA  = 3'd2;
    localparam WR_RESP  = 3'd3;
    localparam WR_DONE  = 3'd4;

    reg [2:0]  wr_state;
    reg [7:0]  wr_beats_left;   // write beats remaining in this burst
    reg [7:0]  wr_len_reg;

    assign wr_idle = (wr_state == WR_IDLE);

    // FIFO read side driven by write engine
    // Pop from FIFO when WVALID and WREADY both asserted (beat transferred)
    assign fifo_rd_en = m_axi_wvalid && m_axi_wready && !fifo_empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state      <= WR_IDLE;
            wr_beats_left <= 8'd0;
            wr_len_reg    <= 8'd0;
            m_axi_awid    <= {ID_WIDTH{1'b0}};
            m_axi_awaddr  <= {ADDR_WIDTH{1'b0}};
            m_axi_awlen   <= 8'd0;
            m_axi_awsize  <= 3'd0;
            m_axi_awburst <= 2'b01;
            m_axi_awvalid <= 1'b0;
            m_axi_wdata   <= {DATA_WIDTH{1'b0}};
            m_axi_wstrb   <= {(DATA_WIDTH/8){1'b0}};
            m_axi_wlast   <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;
            wr_done       <= 1'b0;
            wr_error      <= 1'b0;
        end else begin
            wr_done  <= 1'b0;
            wr_error <= 1'b0;

            case (wr_state)
                WR_IDLE: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;
                    if (wr_start) begin
                        m_axi_awaddr  <= wr_addr;
                        m_axi_awlen   <= wr_len;
                        m_axi_awsize  <= AXI_SIZE[2:0];
                        m_axi_awburst <= AXI_BURST_INCR;
                        m_axi_awid    <= {ID_WIDTH{1'b0}};
                        m_axi_awvalid <= 1'b1;
                        wr_beats_left <= wr_len;
                        wr_len_reg    <= wr_len;
                        wr_state      <= WR_ADDR;
                    end
                end

                WR_ADDR: begin
                    if (m_axi_awready && m_axi_awvalid) begin
                        m_axi_awvalid <= 1'b0;
                        wr_state      <= WR_DATA;
                        // Pre-load first word if FIFO has data
                        if (!fifo_empty) begin
                            m_axi_wdata  <= fifo_rd_data;
                            m_axi_wstrb  <= {(DATA_WIDTH/8){1'b1}};
                            m_axi_wlast  <= (wr_beats_left == 8'd0);
                            m_axi_wvalid <= 1'b1;
                        end
                    end
                end

                WR_DATA: begin
                    // If FIFO empty, deassert WVALID (stall)
                    if (fifo_empty) begin
                        m_axi_wvalid <= 1'b0;
                    end else begin
                        m_axi_wdata  <= fifo_rd_data;
                        m_axi_wstrb  <= {(DATA_WIDTH/8){1'b1}};
                        m_axi_wlast  <= (wr_beats_left == 8'd0);
                        m_axi_wvalid <= 1'b1;
                    end

                    if (m_axi_wvalid && m_axi_wready && !fifo_empty) begin
                        if (wr_beats_left == 8'd0) begin
                            // Last beat transferred
                            m_axi_wvalid <= 1'b0;
                            m_axi_bready <= 1'b1;
                            wr_state     <= WR_RESP;
                        end else begin
                            wr_beats_left <= wr_beats_left - 8'd1;
                        end
                    end
                end

                WR_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        if (m_axi_bresp != AXI_RESP_OKAY)
                            wr_error <= 1'b1;
                        m_axi_bready <= 1'b0;
                        wr_state     <= WR_DONE;
                    end
                end

                WR_DONE: begin
                    wr_done  <= 1'b1;
                    wr_state <= WR_IDLE;
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

endmodule
