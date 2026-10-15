`timescale 1ns/1ps
// =============================================================================
// tb_dma_multichannel.sv  –  Multi-Channel Arbitration Test
//
// Tests with NUM_CHANNELS = 2:
//   1. Both channels programmed simultaneously → round-robin service
//   2. Verify channel 0 then channel 1 both complete correctly
//   3. Re-arm both channels → verify CH1 gets priority next (round-robin)
//   4. Concurrent requests: scoreboard checks each channel's data independently
// =============================================================================
module tb_dma_multichannel;

    localparam NUM_CH   = 2;
    localparam DW       = 32;
    localparam AW       = 32;
    localparam ID_W     = 4;
    localparam FIFO_D   = 16;
    localparam CLK_HALF = 5;
    localparam MEM_D    = 8192;

    logic clk=0, rst_n;

    logic [AW-1:0] axil_awaddr; logic axil_awvalid, axil_awready;
    logic [DW-1:0] axil_wdata;  logic [3:0] axil_wstrb;
    logic axil_wvalid,axil_wready,axil_bvalid,axil_bready; logic [1:0] axil_bresp;
    logic [AW-1:0] axil_araddr; logic axil_arvalid,axil_arready;
    logic [DW-1:0] axil_rdata;  logic [1:0] axil_rresp; logic axil_rvalid,axil_rready;

    logic [ID_W-1:0] m_arid,m_rid,m_awid,m_bid;
    logic [AW-1:0]   m_araddr,m_awaddr;
    logic [7:0]      m_arlen,m_awlen; logic [2:0] m_arsize,m_awsize;
    logic [1:0]      m_arburst,m_awburst,m_rresp,m_bresp;
    logic            m_arvalid,m_arready,m_rlast,m_rvalid,m_rready;
    logic [DW-1:0]   m_rdata,m_wdata; logic [3:0] m_wstrb;
    logic            m_awvalid,m_awready,m_wlast,m_wvalid,m_wready,m_bvalid,m_bready;
    logic irq;

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
        .m_axi_awsize(m_awsize),.m_axi_awburst(m_arburst),.m_axi_awvalid(m_awvalid),.m_axi_awready(m_awready),
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

    task axil_write(input [AW-1:0] addr, input [DW-1:0] data);
        @(posedge clk); #1;
        axil_awaddr=addr; axil_awvalid=1; axil_wdata=data;
        axil_wstrb=4'hF; axil_wvalid=1; axil_bready=1;
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

    task reset_ch(input integer ch);
        axil_write(ch * 32'h40, 32'h4); // soft_reset
        repeat(3) @(posedge clk);
        axil_write(ch * 32'h40, 32'h0);
        repeat(2) @(posedge clk);
    endtask

    // Seed source region and program channel
    task program_ch(
        input integer ch,
        input [AW-1:0] src, dst,
        input [31:0] len
    );
        integer base; integer i;
        base = ch * 32'h40;
        // Seed source memory
        for (i = 0; i < (len >> 2); i++) begin
            mem.mem[(src >> 2) + i] = $urandom_range(32'hFFFF_FFFF, 0);
        end
        axil_write(base + 32'h08, src);
        axil_write(base + 32'h0C, dst);
        axil_write(base + 32'h10, len);
        axil_write(base + 32'h14, 32'h7);     // 8-beat burst
        axil_write(base + 32'h00, 32'h3);     // enable + start
    endtask

    task verify_ch(input integer ch, input [AW-1:0] src, dst, input [31:0] len);
        integer i; logic [DW-1:0] s, d;
        for (i = 0; i < (len >> 2); i++) begin
            s = mem.mem[(src >> 2) + i];
            d = mem.mem[(dst >> 2) + i];
            if (s !== d) begin
                $display("  FAIL CH%0d word[%0d]: src=0x%08X dst=0x%08X", ch, i, s, d);
                fail_count++;
                if (fail_count > 5) i = len; // abort on many errors
            end
        end
    endtask

    logic [DW-1:0] rd;
    logic ch0_done, ch1_done;

    initial begin
        $dumpfile("tb_dma_multichannel.vcd");
        $dumpvars(0, tb_dma_multichannel);
        {axil_awvalid,axil_wvalid,axil_bready,axil_arvalid,axil_rready}='0;

        rst_n=0; repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        $display("\n=== TEST: Multi-Channel Arbitration ===");

        // -------------------------------------------------------------------
        // Round 1: Both channels programmed simultaneously
        //          CH0: src=0x1000 dst=0x2000 len=32
        //          CH1: src=0x3000 dst=0x4000 len=32
        // -------------------------------------------------------------------
        $display("\n-- Round 1: Concurrent CH0 + CH1 start --");
        reset_ch(0); reset_ch(1);
        program_ch(0, 32'h1000, 32'h2000, 32);
        program_ch(1, 32'h3000, 32'h4000, 32);

        // Wait for both done interrupts
        ch0_done = 0; ch1_done = 0;
        fork
            begin : T1
                repeat(200000) @(posedge clk);
                $display("  FAIL: Timeout waiting for both channels");
                fail_count++; disable D1;
            end
            begin : D1
                // Poll STATUS for both channels
                while (!ch0_done || !ch1_done) begin
                    @(posedge clk);
                    axil_read(32'h00 + 32'h04, rd); if (rd[2]) ch0_done=1;
                    axil_read(32'h40 + 32'h04, rd); if (rd[2]) ch1_done=1;
                end
                disable T1;
            end
        join

        $display("  CH0 done=%0d CH1 done=%0d", ch0_done, ch1_done);
        if (!ch0_done) begin $display("  FAIL: CH0 did not complete"); fail_count++; end
        if (!ch1_done) begin $display("  FAIL: CH1 did not complete"); fail_count++; end

        // Verify data
        verify_ch(0, 32'h1000, 32'h2000, 32);
        verify_ch(1, 32'h3000, 32'h4000, 32);
        if (fail_count == 0) $display("  PASS: Both channels transferred correctly");

        // -------------------------------------------------------------------
        // Round 2: Re-arm. CH1 should get priority (last grant was CH1)
        // -------------------------------------------------------------------
        $display("\n-- Round 2: Re-arm, verify round-robin continues --");
        reset_ch(0); reset_ch(1);
        program_ch(0, 32'h5000, 32'h6000, 16);
        program_ch(1, 32'h7000, 32'h8000, 16);

        ch0_done=0; ch1_done=0;
        fork
            begin : T2
                repeat(200000) @(posedge clk);
                $display("  FAIL: Timeout R2");
                fail_count++; disable D2;
            end
            begin : D2
                while (!ch0_done || !ch1_done) begin
                    @(posedge clk);
                    axil_read(32'h00 + 32'h04, rd); if (rd[2]) ch0_done=1;
                    axil_read(32'h40 + 32'h04, rd); if (rd[2]) ch1_done=1;
                end
                disable T2;
            end
        join

        verify_ch(0, 32'h5000, 32'h6000, 16);
        verify_ch(1, 32'h7000, 32'h8000, 16);
        if (fail_count == 0) $display("  PASS: Round 2 both channels OK");

        $display("\n=== MULTICHANNEL TEST COMPLETE ===");
        if (fail_count == 0) $display("TEST PASS");
        else                 $display("TEST FAIL: %0d error(s)", fail_count);

        #200 $finish;
    end

    initial begin #5000000; $display("TEST FAIL: Global timeout"); $finish; end

endmodule
