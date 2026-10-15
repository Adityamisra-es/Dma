`timescale 1ns/1ps
// =============================================================================
// dma_axi_lite_slave.v  –  AXI4-Lite Configuration Slave
//
// Implements the CPU-facing register file.
// Supports NUM_CHANNELS independent channel register banks.
//
// Register map  (channel n at byte offset n * 'h40)
// +---------+-------------+-------+-------+------------------------------------+
// | Offset  | Name        | R/W   | Reset | Description                        |
// +---------+-------------+-------+-------+------------------------------------+
// | 0x00    | CONTROL     | RW    | 0x00  | [0]=enable [1]=start               |
// |         |             |       |       | [2]=soft_reset [3]=irq_en          |
// | 0x04    | STATUS      | RO    | 0x01  | [0]=idle [1]=busy [2]=done [3]=err |
// | 0x08    | SRC_ADDR    | RW*   | 0x00  | Source byte address                |
// | 0x0C    | DST_ADDR    | RW*   | 0x00  | Destination byte address           |
// | 0x10    | XFER_LEN    | RW*   | 0x00  | Transfer length in bytes           |
// | 0x14    | BURST_CFG   | RW*   | 0x0F  | [7:0] max burst beats (ARLEN val)  |
// | 0x18    | INT_STATUS  | RW1C  | 0x00  | [0]=done_irq [1]=error_irq         |
// | 0x1C    | INT_MASK    | RW    | 0x00  | [0]=mask_done [1]=mask_error       |
// | 0x20    | ERROR_CODE  | RO    | 0x00  | Error reason code                  |
// +---------+-------------+-------+-------+------------------------------------+
// (* locked while channel is busy)
//
// AXI4-Lite Write: AWVALID/WVALID may arrive in any order. Both are latched
// independently; the register write commits when both are captured.
// =============================================================================
module dma_axi_lite_slave #(
    parameter NUM_CHANNELS = 2,
    parameter ADDR_WIDTH   = 32,
    parameter DATA_WIDTH   = 32
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // -----------------------------------------------------------------------
    // AXI4-Lite Slave  (CPU → DMA configuration port)
    // -----------------------------------------------------------------------
    // Write address channel
    input  wire [ADDR_WIDTH-1:0]       s_axil_awaddr,
    input  wire                        s_axil_awvalid,
    output reg                         s_axil_awready,

    // Write data channel
    input  wire [DATA_WIDTH-1:0]       s_axil_wdata,
    input  wire [DATA_WIDTH/8-1:0]     s_axil_wstrb,
    input  wire                        s_axil_wvalid,
    output reg                         s_axil_wready,

    // Write response channel
    output reg  [1:0]                  s_axil_bresp,
    output reg                         s_axil_bvalid,
    input  wire                        s_axil_bready,

    // Read address channel
    input  wire [ADDR_WIDTH-1:0]       s_axil_araddr,
    input  wire                        s_axil_arvalid,
    output reg                         s_axil_arready,

    // Read data channel
    output reg  [DATA_WIDTH-1:0]       s_axil_rdata,
    output reg  [1:0]                  s_axil_rresp,
    output reg                         s_axil_rvalid,
    input  wire                        s_axil_rready,

    // -----------------------------------------------------------------------
    // Register outputs to controller (flat buses, channel n at bit n*W)
    // -----------------------------------------------------------------------
    output reg  [NUM_CHANNELS-1:0]         ch_enable,
    output reg  [NUM_CHANNELS-1:0]         ch_start,       // one-cycle start pulse
    output reg  [NUM_CHANNELS-1:0]         ch_soft_reset,
    output reg  [NUM_CHANNELS-1:0]         ch_irq_en,

    output reg  [32*NUM_CHANNELS-1:0]      ch_src_addr,
    output reg  [32*NUM_CHANNELS-1:0]      ch_dst_addr,
    output reg  [32*NUM_CHANNELS-1:0]      ch_xfer_len,
    output reg  [8*NUM_CHANNELS-1:0]       ch_burst_len,   // ARLEN field value

    output reg  [NUM_CHANNELS-1:0]         ch_int_mask_done,
    output reg  [NUM_CHANNELS-1:0]         ch_int_mask_error,

    // W1C clear strobes (one-cycle pulse)
    output reg  [NUM_CHANNELS-1:0]         ch_int_clr_done,
    output reg  [NUM_CHANNELS-1:0]         ch_int_clr_error,

    // -----------------------------------------------------------------------
    // Status inputs from controller
    // -----------------------------------------------------------------------
    input  wire [NUM_CHANNELS-1:0]         ch_status_idle,
    input  wire [NUM_CHANNELS-1:0]         ch_status_busy,
    input  wire [NUM_CHANNELS-1:0]         ch_status_done,
    input  wire [NUM_CHANNELS-1:0]         ch_status_error,
    input  wire [8*NUM_CHANNELS-1:0]       ch_error_code,

    // Interrupt status (from interrupt controller)
    input  wire [NUM_CHANNELS-1:0]         ch_int_status_done,
    input  wire [NUM_CHANNELS-1:0]         ch_int_status_error
);

    // -----------------------------------------------------------------------
    // Local parameters
    // -----------------------------------------------------------------------
    localparam CH_STRIDE  = 6'd32;          // 32 bytes (8 registers × 4 bytes)
    // Register offsets within a channel bank
    localparam REG_CTRL   = 3'd0;  // 0x00
    localparam REG_STAT   = 3'd1;  // 0x04
    localparam REG_SRC    = 3'd2;  // 0x08
    localparam REG_DST    = 3'd3;  // 0x0C
    localparam REG_LEN    = 3'd4;  // 0x10
    localparam REG_BCFG   = 3'd5;  // 0x14
    localparam REG_ISTAT  = 3'd6;  // 0x18
    localparam REG_IMASK  = 3'd7;  // 0x1C
    // ERROR_CODE lives at 0x20 = next bank – but we map it within a 64-byte bank
    // We actually use 9 registers per channel with 0x40 stride
    localparam CH_BANK    = 7'h40; // 64-byte stride between channels

    // -----------------------------------------------------------------------
    // Write address / write data capture
    // AXI4-Lite: address and data may arrive independently
    // -----------------------------------------------------------------------
    reg                  aw_captured;
    reg [ADDR_WIDTH-1:0] aw_addr_reg;

    reg                  wd_captured;
    reg [DATA_WIDTH-1:0] wd_data_reg;
    reg [DATA_WIDTH/8-1:0] wd_strb_reg;

    wire write_pending = aw_captured && wd_captured;

    // -----------------------------------------------------------------------
    // Channel + register decode (from captured write address)
    // -----------------------------------------------------------------------
    wire [$clog2(NUM_CHANNELS)-1:0] wr_ch_sel;
    wire [4:0]                       wr_reg_sel;

    assign wr_ch_sel  = aw_addr_reg[ADDR_WIDTH-1:6];   // upper bits select channel (÷64)
    assign wr_reg_sel = aw_addr_reg[5:2];               // bits [5:2] → register index

    wire [$clog2(NUM_CHANNELS)-1:0] rd_ch_sel;
    wire [4:0]                       rd_reg_sel;
    assign rd_ch_sel  = s_axil_araddr[ADDR_WIDTH-1:6];
    assign rd_reg_sel = s_axil_araddr[5:2];

    // -----------------------------------------------------------------------
    // Helper – channel busy (write-protect config registers when active)
    // -----------------------------------------------------------------------
    wire ch_busy_wr = ch_status_busy[wr_ch_sel];
    wire ch_busy_rd = ch_status_busy[rd_ch_sel];

    // -----------------------------------------------------------------------
    // AXI4-Lite Write Handshakes
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_awready <= 1'b0;
            aw_captured    <= 1'b0;
            aw_addr_reg    <= {ADDR_WIDTH{1'b0}};
        end else begin
            // Deassert after one-cycle acceptance
            s_axil_awready <= 1'b0;

            if (s_axil_awvalid && !aw_captured) begin
                s_axil_awready <= 1'b1;
                aw_addr_reg    <= s_axil_awaddr;
                aw_captured    <= 1'b1;
            end else if (write_pending) begin
                aw_captured <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_wready <= 1'b0;
            wd_captured   <= 1'b0;
            wd_data_reg   <= {DATA_WIDTH{1'b0}};
            wd_strb_reg   <= {(DATA_WIDTH/8){1'b0}};
        end else begin
            s_axil_wready <= 1'b0;

            if (s_axil_wvalid && !wd_captured) begin
                s_axil_wready <= 1'b1;
                wd_data_reg   <= s_axil_wdata;
                wd_strb_reg   <= s_axil_wstrb;
                wd_captured   <= 1'b1;
            end else if (write_pending) begin
                wd_captured <= 1'b0;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Register Write  (committed when write_pending, before clearing captures)
    // -----------------------------------------------------------------------
    integer ch;

    // Strobe-aware byte write helper (combinational – produces masked data)
    function [31:0] apply_strobe;
        input [31:0] old_val;
        input [31:0] new_val;
        input [3:0]  strb;
        begin
            apply_strobe[7:0]   = strb[0] ? new_val[7:0]   : old_val[7:0];
            apply_strobe[15:8]  = strb[1] ? new_val[15:8]  : old_val[15:8];
            apply_strobe[23:16] = strb[2] ? new_val[23:16] : old_val[23:16];
            apply_strobe[31:24] = strb[3] ? new_val[31:24] : old_val[31:24];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch_enable         <= {NUM_CHANNELS{1'b0}};
            ch_start          <= {NUM_CHANNELS{1'b0}};
            ch_soft_reset     <= {NUM_CHANNELS{1'b0}};
            ch_irq_en         <= {NUM_CHANNELS{1'b0}};
            ch_int_mask_done  <= {NUM_CHANNELS{1'b0}};
            ch_int_mask_error <= {NUM_CHANNELS{1'b0}};
            ch_int_clr_done   <= {NUM_CHANNELS{1'b0}};
            ch_int_clr_error  <= {NUM_CHANNELS{1'b0}};
            for (ch = 0; ch < NUM_CHANNELS; ch = ch + 1) begin
                ch_src_addr[32*ch +: 32] <= 32'd0;
                ch_dst_addr[32*ch +: 32] <= 32'd0;
                ch_xfer_len[32*ch +: 32] <= 32'd0;
                ch_burst_len[8*ch  +:  8] <= 8'h0F; // default: 16 beats
            end
        end else begin
            // Default: clear one-cycle strobes
            ch_start        <= {NUM_CHANNELS{1'b0}};
            ch_soft_reset   <= {NUM_CHANNELS{1'b0}};
            ch_int_clr_done <= {NUM_CHANNELS{1'b0}};
            ch_int_clr_error<= {NUM_CHANNELS{1'b0}};

            if (write_pending && (wr_ch_sel < NUM_CHANNELS)) begin
                case (wr_reg_sel)
                    // 0x00: CONTROL
                    5'd0: begin
                        ch_enable[wr_ch_sel]     <= wd_strb_reg[0] ? wd_data_reg[0] : ch_enable[wr_ch_sel];
                        // start only fires if channel is not busy
                        if (wd_strb_reg[0] && wd_data_reg[1] && !ch_busy_wr)
                            ch_start[wr_ch_sel]  <= 1'b1;
                        ch_soft_reset[wr_ch_sel] <= wd_strb_reg[0] ? wd_data_reg[2] : ch_soft_reset[wr_ch_sel];
                        ch_irq_en[wr_ch_sel]     <= wd_strb_reg[0] ? wd_data_reg[3] : ch_irq_en[wr_ch_sel];
                    end
                    // 0x04: STATUS (read-only, ignore writes)
                    5'd1: ;
                    // 0x08: SRC_ADDR (locked when busy)
                    5'd2: if (!ch_busy_wr)
                        ch_src_addr[32*wr_ch_sel +: 32] <=
                            apply_strobe(ch_src_addr[32*wr_ch_sel +: 32], wd_data_reg, wd_strb_reg);
                    // 0x0C: DST_ADDR
                    5'd3: if (!ch_busy_wr)
                        ch_dst_addr[32*wr_ch_sel +: 32] <=
                            apply_strobe(ch_dst_addr[32*wr_ch_sel +: 32], wd_data_reg, wd_strb_reg);
                    // 0x10: XFER_LEN
                    5'd4: if (!ch_busy_wr)
                        ch_xfer_len[32*wr_ch_sel +: 32] <=
                            apply_strobe(ch_xfer_len[32*wr_ch_sel +: 32], wd_data_reg, wd_strb_reg);
                    // 0x14: BURST_CFG [7:0]
                    5'd5: if (!ch_busy_wr)
                        ch_burst_len[8*wr_ch_sel +: 8] <=
                            wd_strb_reg[0] ? wd_data_reg[7:0] : ch_burst_len[8*wr_ch_sel +: 8];
                    // 0x18: INT_STATUS  (W1C)
                    5'd6: begin
                        ch_int_clr_done [wr_ch_sel] <= wd_strb_reg[0] & wd_data_reg[0];
                        ch_int_clr_error[wr_ch_sel] <= wd_strb_reg[0] & wd_data_reg[1];
                    end
                    // 0x1C: INT_MASK
                    5'd7: begin
                        ch_int_mask_done [wr_ch_sel] <= wd_strb_reg[0] ? wd_data_reg[0] : ch_int_mask_done[wr_ch_sel];
                        ch_int_mask_error[wr_ch_sel] <= wd_strb_reg[0] ? wd_data_reg[1] : ch_int_mask_error[wr_ch_sel];
                    end
                    // 0x20: ERROR_CODE (read-only, ignore)
                    5'd8: ;
                    default: ;
                endcase
            end
        end
    end

    // -----------------------------------------------------------------------
    // Write Response
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_bvalid <= 1'b0;
            s_axil_bresp  <= 2'b00;
        end else begin
            if (write_pending && !s_axil_bvalid) begin
                s_axil_bvalid <= 1'b1;
                // SLVERR if channel index out of range; OKAY otherwise
                s_axil_bresp  <= (wr_ch_sel < NUM_CHANNELS) ? 2'b00 : 2'b10;
            end else if (s_axil_bvalid && s_axil_bready) begin
                s_axil_bvalid <= 1'b0;
            end
        end
    end

    // -----------------------------------------------------------------------
    // AXI4-Lite Read  –  single-cycle latency
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_arready <= 1'b0;
            s_axil_rvalid  <= 1'b0;
            s_axil_rdata   <= {DATA_WIDTH{1'b0}};
            s_axil_rresp   <= 2'b00;
        end else begin
            s_axil_arready <= 1'b0;

            if (s_axil_arvalid && !s_axil_rvalid) begin
                s_axil_arready <= 1'b1;
                s_axil_rvalid  <= 1'b1;
                s_axil_rresp   <= 2'b00;
                s_axil_rdata   <= 32'd0; // default

                if (rd_ch_sel < NUM_CHANNELS) begin
                    case (rd_reg_sel)
                        5'd0: s_axil_rdata <= {28'd0,
                                               ch_irq_en[rd_ch_sel],
                                               ch_soft_reset[rd_ch_sel],
                                               ch_start[rd_ch_sel],
                                               ch_enable[rd_ch_sel]};
                        5'd1: s_axil_rdata <= {28'd0,
                                               ch_status_error[rd_ch_sel],
                                               ch_status_done[rd_ch_sel],
                                               ch_status_busy[rd_ch_sel],
                                               ch_status_idle[rd_ch_sel]};
                        5'd2: s_axil_rdata <= ch_src_addr[32*rd_ch_sel +: 32];
                        5'd3: s_axil_rdata <= ch_dst_addr[32*rd_ch_sel +: 32];
                        5'd4: s_axil_rdata <= ch_xfer_len[32*rd_ch_sel +: 32];
                        5'd5: s_axil_rdata <= {24'd0, ch_burst_len[8*rd_ch_sel +: 8]};
                        5'd6: s_axil_rdata <= {30'd0,
                                               ch_int_status_error[rd_ch_sel],
                                               ch_int_status_done[rd_ch_sel]};
                        5'd7: s_axil_rdata <= {30'd0,
                                               ch_int_mask_error[rd_ch_sel],
                                               ch_int_mask_done[rd_ch_sel]};
                        5'd8: s_axil_rdata <= {24'd0, ch_error_code[8*rd_ch_sel +: 8]};
                        default: begin
                            s_axil_rdata <= 32'd0;
                            s_axil_rresp <= 2'b10; // SLVERR
                        end
                    endcase
                end else begin
                    s_axil_rresp <= 2'b10; // SLVERR – out of range
                end
            end else if (s_axil_rvalid && s_axil_rready) begin
                s_axil_rvalid <= 1'b0;
            end
        end
    end

endmodule
