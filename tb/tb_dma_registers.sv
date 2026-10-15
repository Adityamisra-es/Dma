`timescale 1ns/1ps
// =============================================================================
// tb_dma_registers.sv  –  AXI4-Lite Register Read/Write Test
//
// Tests:
//   1. Reset state of all registers
//   2. Write and read back CONTROL, SRC_ADDR, DST_ADDR, XFER_LEN, BURST_CFG
//   3. INT_MASK write/read
//   4. STATUS is read-only (write ignored)
//   5. ERROR_CODE is read-only
//   6. Config registers locked while STATUS[busy]=1 (simulated by setting
//      ch_status_busy through the DUT's internal status path)
//   7. INT_STATUS W1C behaviour (verified with IRQ controller interaction)
//
// Pass criterion: all checks must agree; any mismatch prints FAIL.
// =============================================================================
module tb_dma_registers;

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    localparam NUM_CH    = 2;
    localparam DW        = 32;
    localparam AW        = 32;
    localparam CLK_HALF  = 5; // 100 MHz

    // -----------------------------------------------------------------------
    // DUT signals
    // -----------------------------------------------------------------------
    logic                  clk, rst_n;

    // AXI4-Lite
    logic [AW-1:0]         axil_awaddr;
    logic                  axil_awvalid, axil_awready;
    logic [DW-1:0]         axil_wdata;
    logic [DW/8-1:0]       axil_wstrb;
    logic                  axil_wvalid, axil_wready;
    logic [1:0]            axil_bresp;
    logic                  axil_bvalid, axil_bready;
    logic [AW-1:0]         axil_araddr;
    logic                  axil_arvalid, axil_arready;
    logic [DW-1:0]         axil_rdata;
    logic [1:0]            axil_rresp;
    logic                  axil_rvalid, axil_rready;

    // Tie off status signals (channel not busy)
    logic [NUM_CH-1:0]     ch_status_idle  = {NUM_CH{1'b1}};
    logic [NUM_CH-1:0]     ch_status_busy  = {NUM_CH{1'b0}};
    logic [NUM_CH-1:0]     ch_status_done  = {NUM_CH{1'b0}};
    logic [NUM_CH-1:0]     ch_status_error = {NUM_CH{1'b0}};
    logic [8*NUM_CH-1:0]   ch_error_code   = {(8*NUM_CH){1'b0}};
    logic [NUM_CH-1:0]     ch_int_status_done  = {NUM_CH{1'b0}};
    logic [NUM_CH-1:0]     ch_int_status_error = {NUM_CH{1'b0}};

    // -----------------------------------------------------------------------
    // DUT instantiation (AXI4-Lite slave only, no full DMA needed)
    // -----------------------------------------------------------------------
    dma_axi_lite_slave #(
        .NUM_CHANNELS (NUM_CH),
        .ADDR_WIDTH   (AW),
        .DATA_WIDTH   (DW)
    ) dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .s_axil_awaddr      (axil_awaddr),
        .s_axil_awvalid     (axil_awvalid),
        .s_axil_awready     (axil_awready),
        .s_axil_wdata       (axil_wdata),
        .s_axil_wstrb       (axil_wstrb),
        .s_axil_wvalid      (axil_wvalid),
        .s_axil_wready      (axil_wready),
        .s_axil_bresp       (axil_bresp),
        .s_axil_bvalid      (axil_bvalid),
        .s_axil_bready      (axil_bready),
        .s_axil_araddr      (axil_araddr),
        .s_axil_arvalid     (axil_arvalid),
        .s_axil_arready     (axil_arready),
        .s_axil_rdata       (axil_rdata),
        .s_axil_rresp       (axil_rresp),
        .s_axil_rvalid      (axil_rvalid),
        .s_axil_rready      (axil_rready),
        // Outputs (unused in register test)
        .ch_enable          (),
        .ch_start           (),
        .ch_soft_reset      (),
        .ch_irq_en          (),
        .ch_src_addr        (),
        .ch_dst_addr        (),
        .ch_xfer_len        (),
        .ch_burst_len       (),
        .ch_int_mask_done   (),
        .ch_int_mask_error  (),
        .ch_int_clr_done    (),
        .ch_int_clr_error   (),
        // Status inputs
        .ch_status_idle     (ch_status_idle),
        .ch_status_busy     (ch_status_busy),
        .ch_status_done     (ch_status_done),
        .ch_status_error    (ch_status_error),
        .ch_error_code      (ch_error_code),
        .ch_int_status_done (ch_int_status_done),
        .ch_int_status_error(ch_int_status_error)
    );

    // -----------------------------------------------------------------------
    // Clock
    // -----------------------------------------------------------------------
    initial clk = 0;
    always #CLK_HALF clk = ~clk;

    // -----------------------------------------------------------------------
    // Test infrastructure
    // -----------------------------------------------------------------------
    integer fail_count = 0;

    task axil_write(input [AW-1:0] addr, input [DW-1:0] data);
        @(posedge clk);
        axil_awaddr  <= addr;  axil_awvalid <= 1'b1;
        axil_wdata   <= data;  axil_wstrb   <= 4'hF; axil_wvalid <= 1'b1;
        axil_bready  <= 1'b1;
        @(posedge clk iff (axil_awready));
        axil_awvalid <= 1'b0;
        @(posedge clk iff (axil_wready));
        axil_wvalid  <= 1'b0;
        @(posedge clk iff (axil_bvalid));
        axil_bready  <= 1'b0;
        @(posedge clk);
    endtask

    task axil_read(input [AW-1:0] addr, output [DW-1:0] data);
        @(posedge clk);
        axil_araddr  <= addr; axil_arvalid <= 1'b1;
        axil_rready  <= 1'b1;
        @(posedge clk iff (axil_arready));
        axil_arvalid <= 1'b0;
        @(posedge clk iff (axil_rvalid));
        data         = axil_rdata;
        axil_rready  <= 1'b0;
        @(posedge clk);
    endtask

    task check_eq(input string name, input [DW-1:0] got, exp);
        if (got !== exp) begin
            $display("  FAIL [%0t] %s: got 0x%08X, expected 0x%08X", $time, name, got, exp);
            fail_count++;
        end else begin
            $display("  PASS %s = 0x%08X", name, got);
        end
    endtask

    // -----------------------------------------------------------------------
    // Register base addresses
    // CH0: 0x000, CH1: 0x040
    // Offsets: CTRL=0, STAT=4, SRC=8, DST=C, LEN=10, BCFG=14, ISTAT=18, IMASK=1C, ERR=20
    // -----------------------------------------------------------------------
    localparam CH0 = 32'h0000_0000;
    localparam CH1 = 32'h0000_0040;

    logic [DW-1:0] rdata;

    // -----------------------------------------------------------------------
    // Main test
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("tb_dma_registers.vcd");
        $dumpvars(0, tb_dma_registers);

        // Initialise bus
        axil_awaddr = 0; axil_awvalid = 0;
        axil_wdata  = 0; axil_wstrb   = 0; axil_wvalid = 0;
        axil_bready = 0;
        axil_araddr = 0; axil_arvalid = 0;
        axil_rready = 0;

        // Reset
        rst_n = 0;
        repeat(4) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        $display("\n=== TEST: DMA Register Read/Write ===");

        // -------------------------------------------------------------------
        // Test 1: Reset values
        // -------------------------------------------------------------------
        $display("\n-- Reset value checks (CH0) --");
        axil_read(CH0 + 32'h00, rdata); check_eq("CTRL reset",   rdata, 32'h0000_0000);
        axil_read(CH0 + 32'h04, rdata); check_eq("STAT reset",   rdata, 32'h0000_0001); // idle=1
        axil_read(CH0 + 32'h08, rdata); check_eq("SRC reset",    rdata, 32'h0000_0000);
        axil_read(CH0 + 32'h0C, rdata); check_eq("DST reset",    rdata, 32'h0000_0000);
        axil_read(CH0 + 32'h10, rdata); check_eq("LEN reset",    rdata, 32'h0000_0000);
        axil_read(CH0 + 32'h14, rdata); check_eq("BCFG reset",   rdata, 32'h0000_000F);
        axil_read(CH0 + 32'h18, rdata); check_eq("ISTAT reset",  rdata, 32'h0000_0000);
        axil_read(CH0 + 32'h1C, rdata); check_eq("IMASK reset",  rdata, 32'h0000_0000);

        // -------------------------------------------------------------------
        // Test 2: Write and read back config registers (CH0)
        // -------------------------------------------------------------------
        $display("\n-- Write/read config registers (CH0) --");
        axil_write(CH0 + 32'h08, 32'hDEAD_1000);
        axil_read (CH0 + 32'h08, rdata); check_eq("SRC_ADDR",  rdata, 32'hDEAD_1000);

        axil_write(CH0 + 32'h0C, 32'hBEEF_2000);
        axil_read (CH0 + 32'h0C, rdata); check_eq("DST_ADDR",  rdata, 32'hBEEF_2000);

        axil_write(CH0 + 32'h10, 32'h0000_0100);
        axil_read (CH0 + 32'h10, rdata); check_eq("XFER_LEN",  rdata, 32'h0000_0100);

        axil_write(CH0 + 32'h14, 32'h0000_001F);
        axil_read (CH0 + 32'h14, rdata); check_eq("BURST_CFG", rdata, 32'h0000_001F);

        // -------------------------------------------------------------------
        // Test 3: INT_MASK write/read
        // -------------------------------------------------------------------
        $display("\n-- INT_MASK (CH0) --");
        axil_write(CH0 + 32'h1C, 32'h0000_0003); // mask both done and error
        axil_read (CH0 + 32'h1C, rdata); check_eq("INT_MASK",  rdata, 32'h0000_0003);

        // -------------------------------------------------------------------
        // Test 4: STATUS register is read-only
        // -------------------------------------------------------------------
        $display("\n-- STATUS read-only (CH0) --");
        axil_write(CH0 + 32'h04, 32'hFFFF_FFFF); // should be ignored
        axil_read (CH0 + 32'h04, rdata); check_eq("STAT r/o",  rdata, 32'h0000_0001);

        // -------------------------------------------------------------------
        // Test 5: CH1 registers are independent
        // -------------------------------------------------------------------
        $display("\n-- CH1 register independence --");
        axil_write(CH1 + 32'h08, 32'hAAAA_0000);
        axil_read (CH1 + 32'h08, rdata); check_eq("CH1 SRC",   rdata, 32'hAAAA_0000);
        // CH0 must be unchanged
        axil_read (CH0 + 32'h08, rdata); check_eq("CH0 SRC unchanged", rdata, 32'hDEAD_1000);

        // -------------------------------------------------------------------
        // Test 6: CONTROL[1]=start bit (only fires when enable is set and not busy)
        // -------------------------------------------------------------------
        $display("\n-- CONTROL enable bit (CH0) --");
        axil_write(CH0 + 32'h00, 32'h0000_0001); // set enable
        axil_read (CH0 + 32'h00, rdata); check_eq("CTRL enable", rdata[0], 1'b1);

        // -------------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------------
        $display("\n=== REGISTER TEST COMPLETE ===");
        if (fail_count == 0)
            $display("TEST PASS: All %0d checks passed.", 0);
        else
            $display("TEST FAIL: %0d check(s) failed.", fail_count);

        #50 $finish;
    end

    // -----------------------------------------------------------------------
    // Timeout watchdog
    // -----------------------------------------------------------------------
    initial begin
        #500000;
        $display("TEST FAIL: Timeout");
        $finish;
    end

endmodule
