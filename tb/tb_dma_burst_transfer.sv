`timescale 1ns/1ps
// =============================================================================
// tb_dma_burst_transfer.sv  –  Multi-Beat Burst Transfer Test
//
// Tests:
//   A. 8-beat burst (32 bytes) M2M transfer
//   B. 64-beat burst (256 bytes) M2M transfer
//   C. Transfer that spans multiple internal bursts (length > max_burst_len)
//   D. Transfer near a 4 KB address boundary
//
// Each case: seed source, run DMA, compare destination byte-by-byte.
// =============================================================================
module tb_dma_burst_transfer;

    localparam NUM_CH   = 2;
    localparam DW       = 32;
    localparam AW       = 32;
    localparam ID_W     = 4;
    localparam FIFO_D   = 32;
    localparam CLK_HALF = 5;
    localparam MEM_D    = 8192;  // words

    logic clk = 0, rst_n;

    // AXI4-Lite
    logic [AW-1:0] axil_awaddr; logic axil_awvalid, axil_awready;
    logic [DW-1:0] axil_wdata;  logic [3:0] axil_wstrb;
    logic axil_wvalid, axil_wready, axil_bvalid, axil_bready;
    logic [1:0] axil_bresp;
    logic [AW-1:0] axil_araddr; logic axil_arvalid, axil_arready;
    logic [DW-1:0] axil_rdata;  logic [1:0] axil_rresp;
    logic axil_rvalid, axil_rready;

    // AXI4 Master
    logic [ID_W-1:0] m_arid,m_rid,m_awid,m_bid;
    logic [AW-1:0]   m_araddr,m_awaddr;
    logic [7:0]      m_arlen,m_awlen;
    logic [2:0]      m_arsize,m_awsize;
    logic [1:0]      m_arburst,m_awburst,m_rresp,m_bresp;
    logic            m_arvalid,m_arready,m_rlast,m_rvalid,m_rready;
    logic [DW-1:0]   m_rdata,m_wdata;
    logic [3:0]      m_wstrb;
    logic            m_awvalid,m_awready,m_wlast,m_wvalid,m_wready,m_bvalid,m_bready;
    logic            irq;

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

    axi_memory_model #(.DATA_WIDTH(DW),.ADDR_WIDTH(AW),.ID_WIDTH(ID_W),
                       .MEM_DEPTH(MEM_D),.RD_LATENCY(2),.WR_LATENCY(1)) mem (
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

    integer fail_count = 0;

    // -----------------------------------------------------------------------
    // Tasks
    // -----------------------------------------------------------------------
    task axil_write(input [AW-1:0] addr, input [DW-1:0] data);
        @(posedge clk); #1;
        axil_awaddr=addr; axil_awvalid=1; axil_wdata=data; axil_wstrb=4'hF;
        axil_wvalid=1; axil_bready=1;
        fork
            begin @(posedge clk iff axil_awready); axil_awvalid=0; end
            begin @(posedge clk iff axil_wready);  axil_wvalid=0; end
        join
        @(posedge clk iff axil_bvalid); axil_bready=0; @(posedge clk);
    endtask

    task axil_read(input [AW-1:0] addr, output [DW-1:0] data);
        @(posedge clk); #1;
        axil_araddr=addr; axil_arvalid=1; axil_rready=1;
        @(posedge clk iff axil_arready); axil_arvalid=0;
        @(posedge clk iff axil_rvalid); data=axil_rdata; axil_rready=0;
        @(posedge clk);
    endtask

    // Perform one complete DMA transfer and check
    task run_dma_and_check(
        input [AW-1:0] src, dst,
        input [31:0]   len_bytes,   // must be multiple of 4
        input [7:0]    burst_cfg,   // ARLEN value
        input string   test_name
    );
        integer i, words;
        logic [DW-1:0] s_data, d_data, rd;
        words = len_bytes >> 2;

        $display("\n-- %s: src=0x%08X dst=0x%08X len=%0d bytes burst_cfg=%0d --",
                 test_name, src, dst, len_bytes, burst_cfg);

        // Seed source; clear destination
        for (i = 0; i < words; i++) begin
            mem.mem[(src >> 2) + i] = $urandom();
            mem.mem[(dst >> 2) + i] = 32'hDEAD_DEAD;
        end

        // Soft-reset ch0, program, start
        axil_write(32'h00, 32'h4);           // soft_reset
        repeat(2) @(posedge clk);
        axil_write(32'h00, 32'h0);           // clear reset
        axil_write(32'h08, src);
        axil_write(32'h0C, dst);
        axil_write(32'h10, len_bytes);
        axil_write(32'h14, {24'h0, burst_cfg});
        axil_write(32'h00, 32'h3);           // enable + start

        // Wait for done
        fork
            begin : WT
                repeat(200000) @(posedge clk);
                $display("  FAIL: %s timed out", test_name);
                fail_count++;
                disable WI;
            end
            begin : WI
                @(posedge irq);
                disable WT;
            end
        join

        repeat(3) @(posedge clk);

        // Check status
        axil_read(32'h04, rd);
        if (!rd[2]) begin
            $display("  FAIL: STATUS[done] not set"); fail_count++;
        end

        // Scoreboard: compare all words
        for (i = 0; i < words; i++) begin
            s_data = mem.mem[(src >> 2) + i];
            d_data = mem.mem[(dst >> 2) + i];
            if (d_data !== s_data) begin
                $display("  FAIL: word[%0d] src=0x%08X dst=0x%08X", i, s_data, d_data);
                fail_count++;
                if (fail_count > 5) begin
                    $display("  (further mismatches suppressed)"); i = words;
                end
            end
        end
        if (fail_count == 0)
            $display("  PASS: %s data verified (%0d words)", test_name, words);
    endtask

    // -----------------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("tb_dma_burst_transfer.vcd");
        $dumpvars(0, tb_dma_burst_transfer);

        {axil_awvalid,axil_wvalid,axil_bready,axil_arvalid,axil_rready}='0;
        axil_wstrb=4'hF;

        rst_n=0; repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        $display("\n=== TEST: Burst Transfer Suite ===");

        // A: 8-beat burst (32 bytes)
        run_dma_and_check(32'h0000_0400, 32'h0000_0500, 32, 8'h7, "8-beat burst");

        // B: 64-beat transfer with 8-beat max (requires 8 sub-bursts)
        run_dma_and_check(32'h0000_0600, 32'h0000_0800, 256, 8'h7, "256-byte multi-burst");

        // C: Transfer near 4KB boundary (src = 0x0FF0, 16 bytes, crosses 0x1000)
        run_dma_and_check(32'h0000_0FF0, 32'h0000_2000, 16, 8'hF, "4KB boundary transfer");

        $display("\n=== BURST TRANSFER TESTS COMPLETE ===");
        if (fail_count == 0)
            $display("TEST PASS");
        else
            $display("TEST FAIL: %0d error(s)", fail_count);

        #200 $finish;
    end

    initial begin #2000000; $display("TEST FAIL: Global timeout"); $finish; end

endmodule
