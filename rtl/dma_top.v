`timescale 1ns/1ps
// =============================================================================
// dma_top.v  –  DMA Controller Top-Level Integration
//
// Integrates:
//   • dma_axi_lite_slave   – CPU configuration interface
//   • dma_arbiter          – Round-robin channel selection
//   • dma_controller       – Main transfer FSM
//   • dma_burst_engine     – AXI4 read/write burst handler
//   • dma_fifo             – Elastic data buffer between read and write
//   • dma_interrupt_controller – IRQ management
//
// External interfaces:
//   • s_axil_*  : AXI4-Lite slave for CPU register access
//   • m_axi_*   : AXI4 master for memory/peripheral data transfers
//   • irq       : Interrupt output (level, active-high)
//   • clk/rst_n : Global clock and active-low synchronous reset
//
// Parameters:
//   NUM_CHANNELS : 1 – 4 (default 2)
//   ADDR_WIDTH   : address bus width (default 32)
//   DATA_WIDTH   : data bus width    (default 32)
//   FIFO_DEPTH   : FIFO entries, power-of-2 (default 16)
// =============================================================================
module dma_top #(
    parameter NUM_CHANNELS = 2,
    parameter ADDR_WIDTH   = 32,
    parameter DATA_WIDTH   = 32,
    parameter FIFO_DEPTH   = 16,
    parameter ID_WIDTH     = 4
) (
    input  wire clk,
    input  wire rst_n,

    // -----------------------------------------------------------------------
    // AXI4-Lite Slave  (CPU configuration port)
    // -----------------------------------------------------------------------
    input  wire [ADDR_WIDTH-1:0]       s_axil_awaddr,
    input  wire                        s_axil_awvalid,
    output wire                        s_axil_awready,

    input  wire [DATA_WIDTH-1:0]       s_axil_wdata,
    input  wire [DATA_WIDTH/8-1:0]     s_axil_wstrb,
    input  wire                        s_axil_wvalid,
    output wire                        s_axil_wready,

    output wire [1:0]                  s_axil_bresp,
    output wire                        s_axil_bvalid,
    input  wire                        s_axil_bready,

    input  wire [ADDR_WIDTH-1:0]       s_axil_araddr,
    input  wire                        s_axil_arvalid,
    output wire                        s_axil_arready,

    output wire [DATA_WIDTH-1:0]       s_axil_rdata,
    output wire [1:0]                  s_axil_rresp,
    output wire                        s_axil_rvalid,
    input  wire                        s_axil_rready,

    // -----------------------------------------------------------------------
    // AXI4 Master  (data transfer port)
    // -----------------------------------------------------------------------
    // Read address channel
    output wire [ID_WIDTH-1:0]         m_axi_arid,
    output wire [ADDR_WIDTH-1:0]       m_axi_araddr,
    output wire [7:0]                  m_axi_arlen,
    output wire [2:0]                  m_axi_arsize,
    output wire [1:0]                  m_axi_arburst,
    output wire                        m_axi_arvalid,
    input  wire                        m_axi_arready,

    // Read data channel
    input  wire [ID_WIDTH-1:0]         m_axi_rid,
    input  wire [DATA_WIDTH-1:0]       m_axi_rdata,
    input  wire [1:0]                  m_axi_rresp,
    input  wire                        m_axi_rlast,
    input  wire                        m_axi_rvalid,
    output wire                        m_axi_rready,

    // Write address channel
    output wire [ID_WIDTH-1:0]         m_axi_awid,
    output wire [ADDR_WIDTH-1:0]       m_axi_awaddr,
    output wire [7:0]                  m_axi_awlen,
    output wire [2:0]                  m_axi_awsize,
    output wire [1:0]                  m_axi_awburst,
    output wire                        m_axi_awvalid,
    input  wire                        m_axi_awready,

    // Write data channel
    output wire [DATA_WIDTH-1:0]       m_axi_wdata,
    output wire [DATA_WIDTH/8-1:0]     m_axi_wstrb,
    output wire                        m_axi_wlast,
    output wire                        m_axi_wvalid,
    input  wire                        m_axi_wready,

    // Write response channel
    input  wire [ID_WIDTH-1:0]         m_axi_bid,
    input  wire [1:0]                  m_axi_bresp,
    input  wire                        m_axi_bvalid,
    output wire                        m_axi_bready,

    // -----------------------------------------------------------------------
    // Interrupt output
    // -----------------------------------------------------------------------
    output wire irq
);

    // -----------------------------------------------------------------------
    // Internal wires  –  AXI4-Lite slave → controller
    // -----------------------------------------------------------------------
    wire [NUM_CHANNELS-1:0]        ch_enable;
    wire [NUM_CHANNELS-1:0]        ch_start;
    wire [NUM_CHANNELS-1:0]        ch_soft_reset;
    wire [NUM_CHANNELS-1:0]        ch_irq_en;
    wire [32*NUM_CHANNELS-1:0]     ch_src_addr;
    wire [32*NUM_CHANNELS-1:0]     ch_dst_addr;
    wire [32*NUM_CHANNELS-1:0]     ch_xfer_len;
    wire [8*NUM_CHANNELS-1:0]      ch_burst_len;
    wire [NUM_CHANNELS-1:0]        ch_int_mask_done;
    wire [NUM_CHANNELS-1:0]        ch_int_mask_error;
    wire [NUM_CHANNELS-1:0]        ch_int_clr_done;
    wire [NUM_CHANNELS-1:0]        ch_int_clr_error;

    // Controller → slave (status)
    wire [NUM_CHANNELS-1:0]        ch_status_idle;
    wire [NUM_CHANNELS-1:0]        ch_status_busy;
    wire [NUM_CHANNELS-1:0]        ch_status_done;
    wire [NUM_CHANNELS-1:0]        ch_status_error;
    wire [8*NUM_CHANNELS-1:0]      ch_error_code;

    // Interrupt controller → slave (int status)
    wire [NUM_CHANNELS-1:0]        ch_int_status_done;
    wire [NUM_CHANNELS-1:0]        ch_int_status_error;

    // -----------------------------------------------------------------------
    // Internal wires  –  controller ↔ arbiter
    // -----------------------------------------------------------------------
    wire [NUM_CHANNELS-1:0]        arb_request;
    wire [NUM_CHANNELS-1:0]        arb_grant;
    wire [$clog2(NUM_CHANNELS)-1:0] arb_grant_idx;
    wire                           arb_grant_valid;
    wire                           arb_xfer_done;

    // -----------------------------------------------------------------------
    // Internal wires  –  controller ↔ burst engine
    // -----------------------------------------------------------------------
    wire [ADDR_WIDTH-1:0]          be_rd_addr;
    wire [7:0]                     be_rd_len;
    wire                           be_rd_start;
    wire                           be_rd_idle;
    wire [ADDR_WIDTH-1:0]          be_wr_addr;
    wire [7:0]                     be_wr_len;
    wire                           be_wr_start;
    wire                           be_wr_idle;
    wire                           be_rd_done;
    wire                           be_wr_done;
    wire                           be_rd_error;
    wire                           be_wr_error;

    // -----------------------------------------------------------------------
    // Internal wires  –  FIFO
    // -----------------------------------------------------------------------
    wire [DATA_WIDTH-1:0]          fifo_wr_data;
    wire                           fifo_wr_en;
    wire                           fifo_full;
    wire [DATA_WIDTH-1:0]          fifo_rd_data;
    wire                           fifo_rd_en;
    wire                           fifo_empty;

    // -----------------------------------------------------------------------
    // Internal wires  –  controller → interrupt controller
    // -----------------------------------------------------------------------
    wire [NUM_CHANNELS-1:0]        irq_done_pulse;
    wire [NUM_CHANNELS-1:0]        irq_error_pulse;

    // -----------------------------------------------------------------------
    // Module instantiations
    // -----------------------------------------------------------------------

    dma_axi_lite_slave #(
        .NUM_CHANNELS (NUM_CHANNELS),
        .ADDR_WIDTH   (ADDR_WIDTH),
        .DATA_WIDTH   (DATA_WIDTH)
    ) u_axil_slave (
        .clk               (clk),
        .rst_n             (rst_n),
        // AXI4-Lite
        .s_axil_awaddr     (s_axil_awaddr),
        .s_axil_awvalid    (s_axil_awvalid),
        .s_axil_awready    (s_axil_awready),
        .s_axil_wdata      (s_axil_wdata),
        .s_axil_wstrb      (s_axil_wstrb),
        .s_axil_wvalid     (s_axil_wvalid),
        .s_axil_wready     (s_axil_wready),
        .s_axil_bresp      (s_axil_bresp),
        .s_axil_bvalid     (s_axil_bvalid),
        .s_axil_bready     (s_axil_bready),
        .s_axil_araddr     (s_axil_araddr),
        .s_axil_arvalid    (s_axil_arvalid),
        .s_axil_arready    (s_axil_arready),
        .s_axil_rdata      (s_axil_rdata),
        .s_axil_rresp      (s_axil_rresp),
        .s_axil_rvalid     (s_axil_rvalid),
        .s_axil_rready     (s_axil_rready),
        // Register outputs
        .ch_enable         (ch_enable),
        .ch_start          (ch_start),
        .ch_soft_reset     (ch_soft_reset),
        .ch_irq_en         (ch_irq_en),
        .ch_src_addr       (ch_src_addr),
        .ch_dst_addr       (ch_dst_addr),
        .ch_xfer_len       (ch_xfer_len),
        .ch_burst_len      (ch_burst_len),
        .ch_int_mask_done  (ch_int_mask_done),
        .ch_int_mask_error (ch_int_mask_error),
        .ch_int_clr_done   (ch_int_clr_done),
        .ch_int_clr_error  (ch_int_clr_error),
        // Status inputs
        .ch_status_idle    (ch_status_idle),
        .ch_status_busy    (ch_status_busy),
        .ch_status_done    (ch_status_done),
        .ch_status_error   (ch_status_error),
        .ch_error_code     (ch_error_code),
        .ch_int_status_done  (ch_int_status_done),
        .ch_int_status_error (ch_int_status_error)
    );

    dma_arbiter #(
        .NUM_CHANNELS (NUM_CHANNELS)
    ) u_arbiter (
        .clk         (clk),
        .rst_n       (rst_n),
        .request     (arb_request),
        .xfer_done   (arb_xfer_done),
        .grant       (arb_grant),
        .grant_idx   (arb_grant_idx),
        .grant_valid (arb_grant_valid)
    );

    dma_controller #(
        .NUM_CHANNELS (NUM_CHANNELS),
        .ADDR_WIDTH   (ADDR_WIDTH),
        .DATA_WIDTH   (DATA_WIDTH),
        .FIFO_DEPTH   (FIFO_DEPTH)
    ) u_controller (
        .clk            (clk),
        .rst_n          (rst_n),
        // Config
        .ch_enable      (ch_enable),
        .ch_start       (ch_start),
        .ch_soft_reset  (ch_soft_reset),
        .ch_src_addr    (ch_src_addr),
        .ch_dst_addr    (ch_dst_addr),
        .ch_xfer_len    (ch_xfer_len),
        .ch_burst_len   (ch_burst_len),
        // Status
        .ch_status_idle  (ch_status_idle),
        .ch_status_busy  (ch_status_busy),
        .ch_status_done  (ch_status_done),
        .ch_status_error (ch_status_error),
        .ch_error_code   (ch_error_code),
        // Arbiter
        .arb_request    (arb_request),
        .arb_grant      (arb_grant),
        .arb_grant_idx  (arb_grant_idx),
        .arb_grant_valid(arb_grant_valid),
        .arb_xfer_done  (arb_xfer_done),
        // Interrupts
        .irq_done       (irq_done_pulse),
        .irq_error      (irq_error_pulse),
        // Burst engine
        .be_rd_addr     (be_rd_addr),
        .be_rd_len      (be_rd_len),
        .be_rd_start    (be_rd_start),
        .be_rd_idle     (be_rd_idle),
        .be_wr_addr     (be_wr_addr),
        .be_wr_len      (be_wr_len),
        .be_wr_start    (be_wr_start),
        .be_wr_idle     (be_wr_idle),
        .be_rd_done     (be_rd_done),
        .be_wr_done     (be_wr_done),
        .be_rd_error    (be_rd_error),
        .be_wr_error    (be_wr_error)
    );

    dma_burst_engine #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .ID_WIDTH   (ID_WIDTH)
    ) u_burst_engine (
        .clk            (clk),
        .rst_n          (rst_n),
        // Read command
        .rd_addr        (be_rd_addr),
        .rd_len         (be_rd_len),
        .rd_start       (be_rd_start),
        .rd_idle        (be_rd_idle),
        // Write command
        .wr_addr        (be_wr_addr),
        .wr_len         (be_wr_len),
        .wr_start       (be_wr_start),
        .wr_idle        (be_wr_idle),
        // FIFO
        .fifo_wr_data   (fifo_wr_data),
        .fifo_wr_en     (fifo_wr_en),
        .fifo_full      (fifo_full),
        .fifo_rd_data   (fifo_rd_data),
        .fifo_rd_en     (fifo_rd_en),
        .fifo_empty     (fifo_empty),
        // AXI4 Master
        .m_axi_arid     (m_axi_arid),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_rid      (m_axi_rid),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready),
        .m_axi_awid     (m_axi_awid),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_awsize   (m_axi_awsize),
        .m_axi_awburst  (m_axi_awburst),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_bid      (m_axi_bid),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        // Status
        .rd_done        (be_rd_done),
        .wr_done        (be_wr_done),
        .rd_error       (be_rd_error),
        .wr_error       (be_wr_error)
    );

    dma_fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .DEPTH      (FIFO_DEPTH)
    ) u_fifo (
        .clk       (clk),
        .rst_n     (rst_n),
        .wr_en     (fifo_wr_en),
        .wr_data   (fifo_wr_data),
        .full      (fifo_full),
        .overflow  (),             // monitored by burst engine internally
        .rd_en     (fifo_rd_en),
        .rd_data   (fifo_rd_data),
        .empty     (fifo_empty),
        .underflow (),
        .count     ()
    );

    dma_interrupt_controller #(
        .NUM_CHANNELS (NUM_CHANNELS)
    ) u_irq (
        .clk               (clk),
        .rst_n             (rst_n),
        .ch_done           (irq_done_pulse),
        .ch_error          (irq_error_pulse),
        .int_mask_done     (ch_int_mask_done),
        .int_mask_error    (ch_int_mask_error),
        .int_clr_done      (ch_int_clr_done),
        .int_clr_error     (ch_int_clr_error),
        .int_status_done   (ch_int_status_done),
        .int_status_error  (ch_int_status_error),
        .irq               (irq)
    );

endmodule
