`timescale 1ns/1ps
// =============================================================================
// tb_dma_single_transfer.sv  –  Single-Beat Memory-to-Memory Transfer Test
//
// Tests a minimal 4-byte (1 beat) DMA transfer:
//   1. Initialises source memory word.
//   2. Programs DMA via AXI4-Lite.
//   3. Issues start.
//   4. Waits for IRQ / STATUS[done].
//   5. Reads destination memory word.
//   6. Compares against source data  →  PASS / FAIL.
// =============================================================================
module tb_dma_single_transfer;

    localparam NUM_CH   = 2;
    localparam DW       = 32;
    localparam AW       = 32;
    localparam ID_W     = 4;
    localparam FIFO_D   = 16;
    localparam CLK_HALF = 5;

    // -----------------------------------------------------------------------
    // DUT + memory model signals
    // -----------------------------------------------------------------------
    logic clk = 0, rst_n;

    // AXI4-Lite (to DMA config)
    logic [AW-1:0]   axil_awaddr; logic axil_awvalid, axil_awready;
    logic [DW-1:0]   axil_wdata;  logic [DW/8-1:0] axil_wstrb;
    logic            axil_wvalid, axil_wready;
    logic [1:0]      axil_bresp;  logic axil_bvalid, axil_bready;
    logic [AW-1:0]   axil_araddr; logic axil_arvalid, axil_arready;
    logic [DW-1:0]   axil_rdata;  logic [1:0] axil_rresp;
    logic            axil_rvalid, axil_rready;

    // AXI4 Master (from DMA to memory)
    logic [ID_W-1:0] m_arid;  logic [AW-1:0] m_araddr; logic [7:0] m_arlen;
    logic [2:0]      m_arsize; logic [1:0] m_arburst; logic m_arvalid, m_arready;
    logic [ID_W-1:0] m_rid;   logic [DW-1:0] m_rdata; logic [1:0] m_rresp;
    logic            m_rlast, m_rvalid, m_rready;
    logic [ID_W-1:0] m_awid;  logic [AW-1:0] m_awaddr; logic [7:0] m_awlen;
    logic [2:0]      m_awsize; logic [1:0] m_awburst; logic m_awvalid, m_awready;
    logic [DW-1:0]   m_wdata; logic [DW/8-1:0] m_wstrb; logic m_wlast;
    logic            m_wvalid, m_wready;
    logic [ID_W-1:0] m_bid;   logic [1:0] m_bresp; logic m_bvalid, m_bready;
    logic            irq;

    // -----------------------------------------------------------------------
    // DUT
    // -----------------------------------------------------------------------
    dma_top #(.NUM_CHANNELS(NUM_CH),.ADDR_WIDTH(AW),.DATA_WIDTH(DW),
              .FIFO_DEPTH(FIFO_D),.ID_WIDTH(ID_W)) dut (
        .clk(clk),.rst_n(rst_n),
        .s_axil_awaddr(axil_awaddr),.s_axil_awvalid(axil_awvalid),.s_axil_awready(axil_awready),
        .s_axil_wdata(axil_wdata),.s_axil_wstrb(axil_wstrb),.s_axil_wvalid(axil_wvalid),.s_axil_wready(axil_wready),
        .s_axil_bresp(axil_bresp),.s_axil_bvalid(axil_bvalid),.s_axil_bready(axil_bready),
        .s_axil_araddr(axil_araddr),.s_axil_arvalid(axil_arvalid),.s_axil_arready(axil_arready),
        .s_axil_rdata(axil_rdata),.s_axil_rresp(axil_rresp),.s_axil_rvalid(axil_rvalid),.s_axil_rready(axil_rready),
        .m_axi_arid(m_arid),.m_axi_araddr(m_araddr),.m_axi_arlen(m_arlen),
        .m_axi_arsize(m_arsize),.m_axi_arburst(m_arburst),.m_axi_arvalid(m_arvalid),.m_axi_arready(m_arready),
        .m_axi_rid(m_rid),.m_axi_rdata(m_rdata),.m_axi_rresp(m_rresp),
        .m_axi_rlast(m_rlast),.m_axi_rvalid(m_rvalid),.m_axi_rready(m_rready),
        .m_axi_awid(m_awid),.m_axi_awaddr(m_awaddr),.m_axi_awlen(m_awlen),
        .m_axi_awsize(m_awsize),.m_axi_awburst(m_awburst),.m_axi_awvalid(m_awvalid),.m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata),.m_axi_wstrb(m_wstrb),.m_axi_wlast(m_wlast),
        .m_axi_wvalid(m_wvalid),.m_axi_wready(m_wready),
        .m_axi_bid(m_bid),.m_axi_bresp(m_bresp),.m_axi_bvalid(m_bvalid),.m_axi_bready(m_bready),
        .irq(irq)
    );

    // -----------------------------------------------------------------------
    // Memory model
    // -----------------------------------------------------------------------
    axi_memory_model #(.DATA_WIDTH(DW),.ADDR_WIDTH(AW),.ID_WIDTH(ID_W),
                       .MEM_DEPTH(4096),.RD_LATENCY(2),.WR_LATENCY(1)) mem (
        .clk(clk),.rst_n(rst_n),
        .s_axi_arid(m_arid),.s_axi_araddr(m_araddr),.s_axi_arlen(m_arlen),
        .s_axi_arsize(m_arsize),.s_axi_arburst(m_arburst),
        .s_axi_arvalid(m_arvalid),.s_axi_arready(m_arready),
        .s_axi_rid(m_rid),.s_axi_rdata(m_rdata),.s_axi_rresp(m_rresp),
        .s_axi_rlast(m_rlast),.s_axi_rvalid(m_rvalid),.s_axi_rready(m_rready),
        .s_axi_awid(m_awid),.s_axi_awaddr(m_awaddr),.s_axi_awlen(m_awlen),
        .s_axi_awsize(m_awsize),.s_axi_awburst(m_awburst),
        .s_axi_awvalid(m_awvalid),.s_axi_awready(m_awready),
        .s_axi_wdata(m_wdata),.s_axi_wstrb(m_wstrb),.s_axi_wlast(m_wlast),
        .s_axi_wvalid(m_wvalid),.s_axi_wready(m_wready),
        .s_axi_bid(m_bid),.s_axi_bresp(m_bresp),
        .s_axi_bvalid(m_bvalid),.s_axi_bready(m_bready)
    );

    always #CLK_HALF clk = ~clk;

    // -----------------------------------------------------------------------
    // AXI4-Lite tasks
    // -----------------------------------------------------------------------
    task axil_write(input [AW-1:0] addr, input [DW-1:0] data);
        @(posedge clk); #1;
        axil_awaddr  = addr;  axil_awvalid = 1;
        axil_wdata   = data;  axil_wstrb   = 4'hF; axil_wvalid = 1;
        axil_bready  = 1;
        fork
            begin @(posedge clk iff axil_awready); axil_awvalid = 0; end
            begin @(posedge clk iff axil_wready);  axil_wvalid  = 0; end
        join
        @(posedge clk iff axil_bvalid);
        axil_bready = 0;
        @(posedge clk);
    endtask

    task axil_read(input [AW-1:0] addr, output [DW-1:0] data);
        @(posedge clk); #1;
        axil_araddr  = addr; axil_arvalid = 1;
        axil_rready  = 1;
        @(posedge clk iff axil_arready); axil_arvalid = 0;
        @(posedge clk iff axil_rvalid);  data = axil_rdata; axil_rready = 0;
        @(posedge clk);
    endtask

    // -----------------------------------------------------------------------
    // Test
    // -----------------------------------------------------------------------
    localparam SRC_ADDR = 32'h0000_0100;  // word index 64
    localparam DST_ADDR = 32'h0000_0200;  // word index 128
    localparam SRC_DATA = 32'hCAFE_BABE;

    localparam CH0 = 32'h0;
    logic [DW-1:0] rd;
    integer fail_count = 0;

    initial begin
        $dumpfile("tb_dma_single_transfer.vcd");
        $dumpvars(0, tb_dma_single_transfer);

        // Init AXI4-Lite bus
        {axil_awvalid,axil_wvalid,axil_bready,axil_arvalid,axil_rready} = '0;
        axil_wstrb = 4'hF;

        // Reset
        rst_n = 0; repeat(4) @(posedge clk); rst_n = 1; repeat(2) @(posedge clk);

        $display("\n=== TEST: Single-Beat M2M Transfer ===");

        // Seed source memory
        mem.mem[SRC_ADDR >> 2] = SRC_DATA;

        // Program CH0
        axil_write(CH0 + 32'h08, SRC_ADDR);     // SRC_ADDR
        axil_write(CH0 + 32'h0C, DST_ADDR);     // DST_ADDR
        axil_write(CH0 + 32'h10, 32'h4);        // XFER_LEN = 4 bytes (1 beat)
        axil_write(CH0 + 32'h14, 32'h0);        // BURST_CFG: max 1 beat (ARLEN=0)
        axil_write(CH0 + 32'h00, 32'h3);        // CTRL: enable=1, start=1

        // Wait for done interrupt or STATUS[done]
        fork
            begin : WAIT_DONE
                repeat(2000) @(posedge clk);
                $display("FAIL: Timeout waiting for transfer done");
                fail_count++;
                disable WAIT_IRQ;
            end
            begin : WAIT_IRQ
                @(posedge irq);
                disable WAIT_DONE;
                $display("  IRQ received at %0t", $time);
            end
        join

        repeat(2) @(posedge clk);

        // Read STATUS
        axil_read(CH0 + 32'h04, rd);
        if (!rd[2]) begin
            $display("  FAIL: STATUS[done] not set (0x%08X)", rd);
            fail_count++;
        end else
            $display("  PASS: STATUS[done]=1");

        // Read destination
        begin : CHECK_DATA
            logic [DW-1:0] dst_data;
            dst_data = mem.mem[DST_ADDR >> 2];
            if (dst_data !== SRC_DATA) begin
                $display("  FAIL: DST data 0x%08X != SRC data 0x%08X", dst_data, SRC_DATA);
                fail_count++;
            end else
                $display("  PASS: DST data = 0x%08X", dst_data);
        end

        $display("\n=== SINGLE TRANSFER TEST COMPLETE ===");
        if (fail_count == 0)
            $display("TEST PASS");
        else
            $display("TEST FAIL: %0d error(s)", fail_count);

        #100 $finish;
    end

    initial begin #500000; $display("TEST FAIL: Timeout"); $finish; end

endmodule
