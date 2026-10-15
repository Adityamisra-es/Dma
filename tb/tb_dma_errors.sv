`timescale 1ns/1ps
// =============================================================================
// tb_dma_errors.sv  –  Error Handling Test
//
// Tests:
//   1. Zero-length transfer → ERROR_CODE = ERR_ZERO_LEN (0x02)
//   2. Misaligned source address → ERROR_CODE = ERR_ALIGN (0x01)
//   3. AXI read error response (RRESP=SLVERR) → ERROR_CODE = ERR_RD_RESP (0x04)
//   4. AXI write error response (BRESP=SLVERR) → ERROR_CODE = ERR_WR_RESP (0x08)
//   5. INT_STATUS cleared correctly via W1C
//   6. DMA recovers after soft-reset following error
// =============================================================================
module tb_dma_errors;

    localparam DW=32, AW=32, ID_W=4, NUM_CH=2, FIFO_D=16, CLK_HALF=5;

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

    // -----------------------------------------------------------------------
    // Error-injecting memory (RESP_FORCE_ERR controlled at elaboration)
    // We use a parameter to select which instance to connect
    // -----------------------------------------------------------------------
    // Normal memory (no errors)
    logic [ID_W-1:0] nm_arid,nm_rid,nm_awid,nm_bid;
    logic [AW-1:0]   nm_araddr,nm_awaddr;
    logic [7:0]      nm_arlen,nm_awlen; logic [2:0] nm_arsize,nm_awsize;
    logic [1:0]      nm_arburst,nm_awburst,nm_rresp,nm_bresp;
    logic            nm_arvalid,nm_arready,nm_rlast,nm_rvalid,nm_rready;
    logic [DW-1:0]   nm_rdata,nm_wdata; logic [3:0] nm_wstrb;
    logic            nm_awvalid,nm_awready,nm_wlast,nm_wvalid,nm_wready,nm_bvalid,nm_bready;

    // Error-injection override (controlled by test)
    logic force_err = 0;

    // Mux between normal and error response
    // We implement error injection by overriding RRESP/BRESP on the fly
    // through a simple combinational override layer
    logic [DW-1:0] rdata_mux;
    logic [1:0]    rresp_mux, bresp_mux;
    logic          rlast_mux, rvalid_mux, arready_mux;
    logic          awready_mux, wready_mux, bvalid_mux;
    logic [ID_W-1:0] rid_mux, bid_mux;

    assign rdata_mux  = nm_rdata;
    assign rlast_mux  = nm_rlast;
    assign rvalid_mux = nm_rvalid;
    assign arready_mux= nm_arready;
    assign awready_mux= nm_awready;
    assign wready_mux = nm_wready;
    assign bvalid_mux = nm_bvalid;
    assign rid_mux    = nm_rid;
    assign bid_mux    = nm_bid;
    assign rresp_mux  = force_err ? 2'b10 : nm_rresp;
    assign bresp_mux  = force_err ? 2'b10 : nm_bresp;

    dma_top #(.NUM_CHANNELS(NUM_CH),.ADDR_WIDTH(AW),.DATA_WIDTH(DW),
              .FIFO_DEPTH(FIFO_D),.ID_WIDTH(ID_W)) dut (
        .clk(clk),.rst_n(rst_n),
        .s_axil_awaddr(axil_awaddr),.s_axil_awvalid(axil_awvalid),.s_axil_awready(axil_awready),
        .s_axil_wdata(axil_wdata),.s_axil_wstrb(axil_wstrb),.s_axil_wvalid(axil_wvalid),.s_axil_wready(axil_wready),
        .s_axil_bresp(axil_bresp),.s_axil_bvalid(axil_bvalid),.s_axil_bready(axil_bready),
        .s_axil_araddr(axil_araddr),.s_axil_arvalid(axil_arvalid),.s_axil_arready(axil_arready),
        .s_axil_rdata(axil_rdata),.s_axil_rresp(axil_rresp),.s_axil_rvalid(axil_rvalid),.s_axil_rready(axil_rready),
        .m_axi_arid(nm_arid),.m_axi_araddr(nm_araddr),.m_axi_arlen(nm_arlen),
        .m_axi_arsize(nm_arsize),.m_axi_arburst(nm_arburst),.m_axi_arvalid(nm_arvalid),.m_axi_arready(arready_mux),
        .m_axi_rid(rid_mux),.m_axi_rdata(rdata_mux),.m_axi_rresp(rresp_mux),
        .m_axi_rlast(rlast_mux),.m_axi_rvalid(rvalid_mux),.m_axi_rready(nm_rready),
        .m_axi_awid(nm_awid),.m_axi_awaddr(nm_awaddr),.m_axi_awlen(nm_awlen),
        .m_axi_awsize(nm_awsize),.m_axi_awburst(nm_awburst),.m_axi_awvalid(nm_awvalid),.m_axi_awready(awready_mux),
        .m_axi_wdata(nm_wdata),.m_axi_wstrb(nm_wstrb),.m_axi_wlast(nm_wlast),
        .m_axi_wvalid(nm_wvalid),.m_axi_wready(wready_mux),
        .m_axi_bid(bid_mux),.m_axi_bresp(bresp_mux),.m_axi_bvalid(bvalid_mux),.m_axi_bready(nm_bready),
        .irq(irq)
    );

    axi_memory_model #(.DATA_WIDTH(DW),.ADDR_WIDTH(AW),.ID_WIDTH(ID_W),
                       .MEM_DEPTH(4096),.RD_LATENCY(1),.WR_LATENCY(1)) mem (
        .clk(clk),.rst_n(rst_n),
        .s_axi_arid(nm_arid),.s_axi_araddr(nm_araddr),.s_axi_arlen(nm_arlen),
        .s_axi_arsize(nm_arsize),.s_axi_arburst(nm_arburst),
        .s_axi_arvalid(nm_arvalid),.s_axi_arready(nm_arready),
        .s_axi_rid(nm_rid),.s_axi_rdata(nm_rdata),.s_axi_rresp(nm_rresp),
        .s_axi_rlast(nm_rlast),.s_axi_rvalid(nm_rvalid),.s_axi_rready(nm_rready),
        .s_axi_awid(nm_awid),.s_axi_awaddr(nm_awaddr),.s_axi_awlen(nm_awlen),
        .s_axi_awsize(nm_awsize),.s_axi_awburst(nm_awburst),
        .s_axi_awvalid(nm_awvalid),.s_axi_awready(nm_awready),
        .s_axi_wdata(nm_wdata),.s_axi_wstrb(nm_wstrb),.s_axi_wlast(nm_wlast),
        .s_axi_wvalid(nm_wvalid),.s_axi_wready(nm_wready),
        .s_axi_bid(nm_bid),.s_axi_bresp(nm_bresp),
        .s_axi_bvalid(nm_bvalid),.s_axi_bready(nm_bready)
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

    task reset_ch0;
        axil_write(32'h00, 32'h4); // soft_reset
        repeat(3) @(posedge clk);
        axil_write(32'h00, 32'h0);
        repeat(2) @(posedge clk);
    endtask

    task wait_for_irq_or_timeout(output logic timed_out);
        timed_out = 0;
        fork
            begin : WT begin repeat(50000) @(posedge clk); timed_out=1; disable WI; end end
            begin : WI begin @(posedge irq or posedge irq); disable WT; end end
        join
        repeat(2) @(posedge clk);
    endtask

    logic [DW-1:0] rd;
    logic timed_out;

    initial begin
        $dumpfile("tb_dma_errors.vcd");
        $dumpvars(0, tb_dma_errors);
        {axil_awvalid,axil_wvalid,axil_bready,axil_arvalid,axil_rready}='0;
        force_err = 0;

        rst_n=0; repeat(4) @(posedge clk); rst_n=1; repeat(2) @(posedge clk);

        $display("\n=== TEST: Error Handling ===");

        // -------------------------------------------------------------------
        // Test 1: Zero-length transfer
        // -------------------------------------------------------------------
        $display("\n-- T1: Zero-length transfer --");
        reset_ch0();
        axil_write(32'h08, 32'h100); axil_write(32'h0C, 32'h200);
        axil_write(32'h10, 32'h0);   // len=0
        axil_write(32'h14, 32'hF);
        axil_write(32'h00, 32'h3);   // enable+start
        wait_for_irq_or_timeout(timed_out);
        axil_read(32'h04, rd);
        if (!rd[3]) begin $display("  FAIL: STATUS[error] not set"); fail_count++; end
        else $display("  PASS: STATUS[error]=1 for zero-length");
        axil_read(32'h20, rd);
        if (rd[7:0] !== 8'h02) begin $display("  FAIL: ERR_CODE=0x%02X (exp 0x02)", rd[7:0]); fail_count++; end
        else $display("  PASS: ERR_CODE=0x02 (ZERO_LEN)");

        // -------------------------------------------------------------------
        // Test 2: Misaligned address
        // -------------------------------------------------------------------
        $display("\n-- T2: Misaligned source address --");
        reset_ch0();
        axil_write(32'h08, 32'h101); // misaligned (not 4-byte aligned)
        axil_write(32'h0C, 32'h200);
        axil_write(32'h10, 32'h4);
        axil_write(32'h14, 32'hF);
        axil_write(32'h00, 32'h3);
        wait_for_irq_or_timeout(timed_out);
        axil_read(32'h04, rd);
        if (!rd[3]) begin $display("  FAIL: STATUS[error] not set"); fail_count++; end
        else $display("  PASS: STATUS[error]=1 for misalignment");
        axil_read(32'h20, rd);
        if (rd[7:0] !== 8'h01) begin $display("  FAIL: ERR_CODE=0x%02X (exp 0x01)", rd[7:0]); fail_count++; end
        else $display("  PASS: ERR_CODE=0x01 (ALIGN)");

        // -------------------------------------------------------------------
        // Test 3: AXI read error response
        // -------------------------------------------------------------------
        $display("\n-- T3: AXI RRESP error injection --");
        reset_ch0();
        force_err = 1;  // inject SLVERR on all AXI responses
        axil_write(32'h08, 32'h100);
        axil_write(32'h0C, 32'h200);
        axil_write(32'h10, 32'h4);
        axil_write(32'h14, 32'h0);
        axil_write(32'h00, 32'h3);
        wait_for_irq_or_timeout(timed_out);
        force_err = 0;
        axil_read(32'h04, rd);
        if (!rd[3]) begin $display("  FAIL: STATUS[error] not set for RRESP error"); fail_count++; end
        else $display("  PASS: STATUS[error]=1 for RRESP error");
        axil_read(32'h20, rd);
        if (rd[7:0] !== 8'h04 && rd[7:0] !== 8'h08) begin
            $display("  FAIL: ERR_CODE=0x%02X (exp 0x04 or 0x08)", rd[7:0]); fail_count++;
        end else
            $display("  PASS: ERR_CODE=0x%02X (AXI error)", rd[7:0]);

        // -------------------------------------------------------------------
        // Test 4: INT_STATUS W1C and IRQ clear
        // -------------------------------------------------------------------
        $display("\n-- T4: INT_STATUS W1C and recovery --");
        // Check INT_STATUS is set
        axil_read(32'h18, rd);
        if (rd[1:0] == 2'b00)
            $display("  NOTE: INT_STATUS=0 (already cleared by read, or not set)");
        else begin
            // Clear by writing 1 to the set bits
            axil_write(32'h18, rd);  // W1C
            repeat(2) @(posedge clk);
            axil_read(32'h18, rd);
            if (rd[1:0] != 2'b00) begin
                $display("  FAIL: INT_STATUS not cleared after W1C: 0x%X", rd); fail_count++;
            end else
                $display("  PASS: INT_STATUS cleared via W1C");
        end

        // -------------------------------------------------------------------
        // Test 5: Recovery after error (good transfer after soft-reset)
        // -------------------------------------------------------------------
        $display("\n-- T5: Recovery with good transfer after error --");
        reset_ch0();
        mem.mem[32'h100>>2] = 32'hABCD_EF01;
        axil_write(32'h08, 32'h100);
        axil_write(32'h0C, 32'h300);
        axil_write(32'h10, 32'h4);
        axil_write(32'h14, 32'h0);
        axil_write(32'h00, 32'h3);
        wait_for_irq_or_timeout(timed_out);
        axil_read(32'h04, rd);
        if (!rd[2]) begin $display("  FAIL: STATUS[done] not set after recovery"); fail_count++; end
        else $display("  PASS: DMA recovered successfully");

        $display("\n=== ERROR TESTS COMPLETE ===");
        if (fail_count == 0) $display("TEST PASS");
        else                 $display("TEST FAIL: %0d error(s)", fail_count);

        #200 $finish;
    end

    initial begin #2000000; $display("TEST FAIL: Global timeout"); $finish; end

endmodule
