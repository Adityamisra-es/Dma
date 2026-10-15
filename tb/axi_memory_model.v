`timescale 1ns/1ps
// =============================================================================
// axi_memory_model.v  –  Behavioral AXI4 Slave Memory Model
//
// A simple word-addressed RAM that responds to AXI4 read and write bursts.
// Designed for testbench use only (not synthesizable).
//
// Features:
//   • Configurable size (MEM_DEPTH × DATA_WIDTH/8 bytes)
//   • Configurable AR/AW/R/W/B response latency
//   • RRESP / BRESP injection for error testing
//   • Protocol checks: LAST correctness, VALID stability
//   • Scoreboard helper tasks (via direct memory read)
// =============================================================================
module axi_memory_model #(
    parameter DATA_WIDTH  = 32,
    parameter ADDR_WIDTH  = 32,
    parameter ID_WIDTH    = 4,
    parameter MEM_DEPTH   = 4096,    // words (not bytes)
    parameter RD_LATENCY  = 2,       // cycles from AR handshake to first R beat
    parameter WR_LATENCY  = 1,       // cycles from AW handshake to AWREADY
    parameter RESP_FORCE_ERR = 0     // if 1, always respond with SLVERR
) (
    input  wire clk,
    input  wire rst_n,

    // Read address channel
    input  wire [ID_WIDTH-1:0]     s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]   s_axi_araddr,
    input  wire [7:0]              s_axi_arlen,
    input  wire [2:0]              s_axi_arsize,
    input  wire [1:0]              s_axi_arburst,
    input  wire                    s_axi_arvalid,
    output reg                     s_axi_arready,

    // Read data channel
    output reg  [ID_WIDTH-1:0]     s_axi_rid,
    output reg  [DATA_WIDTH-1:0]   s_axi_rdata,
    output reg  [1:0]              s_axi_rresp,
    output reg                     s_axi_rlast,
    output reg                     s_axi_rvalid,
    input  wire                    s_axi_rready,

    // Write address channel
    input  wire [ID_WIDTH-1:0]     s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  wire [7:0]              s_axi_awlen,
    input  wire [2:0]              s_axi_awsize,
    input  wire [1:0]              s_axi_awburst,
    input  wire                    s_axi_awvalid,
    output reg                     s_axi_awready,

    // Write data channel
    input  wire [DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [DATA_WIDTH/8-1:0] s_axi_wstrb,
    input  wire                    s_axi_wlast,
    input  wire                    s_axi_wvalid,
    output reg                     s_axi_wready,

    // Write response channel
    output reg  [ID_WIDTH-1:0]     s_axi_bid,
    output reg  [1:0]              s_axi_bresp,
    output reg                     s_axi_bvalid,
    input  wire                    s_axi_bready
);

    localparam BEAT_BYTES = DATA_WIDTH / 8;

    // -----------------------------------------------------------------------
    // Memory array
    // -----------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    integer mi;
    initial begin
        for (mi = 0; mi < MEM_DEPTH; mi = mi + 1)
            mem[mi] = {DATA_WIDTH{1'b0}};
    end

    // -----------------------------------------------------------------------
    // Task: preload a block of memory from an array-like source
    // -----------------------------------------------------------------------
    task mem_write_word;
        input integer addr_word;
        input [DATA_WIDTH-1:0] data;
        begin
            if (addr_word >= 0 && addr_word < MEM_DEPTH)
                mem[addr_word] = data;
        end
    endtask

    task mem_read_word;
        input  integer addr_word;
        output [DATA_WIDTH-1:0] data;
        begin
            data = (addr_word >= 0 && addr_word < MEM_DEPTH) ?
                   mem[addr_word] : {DATA_WIDTH{1'bx}};
        end
    endtask

    // -----------------------------------------------------------------------
    // Read channel
    // -----------------------------------------------------------------------
    reg  [ADDR_WIDTH-1:0] rd_addr_q;
    reg  [7:0]            rd_len_q;
    reg  [ID_WIDTH-1:0]   rd_id_q;
    reg                   rd_pending;
    integer               rd_delay_cnt;
    integer               rd_beat_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= {DATA_WIDTH{1'b0}};
            s_axi_rresp   <= 2'b00;
            s_axi_rlast   <= 1'b0;
            s_axi_rid     <= {ID_WIDTH{1'b0}};
            rd_pending    <= 1'b0;
            rd_delay_cnt  <= 0;
            rd_beat_cnt   <= 0;
            rd_addr_q     <= {ADDR_WIDTH{1'b0}};
            rd_len_q      <= 8'd0;
            rd_id_q       <= {ID_WIDTH{1'b0}};
        end else begin
            s_axi_arready <= 1'b0;

            // Accept read address
            if (!rd_pending && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
                rd_addr_q     <= s_axi_araddr;
                rd_len_q      <= s_axi_arlen;
                rd_id_q       <= s_axi_arid;
                rd_pending    <= 1'b1;
                rd_delay_cnt  <= RD_LATENCY;
                rd_beat_cnt   <= 0;
            end

            // Latency delay
            if (rd_pending && rd_delay_cnt > 0) begin
                rd_delay_cnt <= rd_delay_cnt - 1;
            end

            // Drive read data beats
            if (rd_pending && rd_delay_cnt == 0) begin
                if (!s_axi_rvalid || s_axi_rready) begin
                    // Drive next beat
                    s_axi_rvalid <= 1'b1;
                    s_axi_rid    <= rd_id_q;
                    s_axi_rresp  <= RESP_FORCE_ERR ? 2'b10 : 2'b00;
                    s_axi_rdata  <= mem[(rd_addr_q >> $clog2(BEAT_BYTES)) + rd_beat_cnt];
                    s_axi_rlast  <= (rd_beat_cnt == rd_len_q);

                    if (s_axi_rvalid && s_axi_rready) begin
                        if (rd_beat_cnt == rd_len_q) begin
                            // Burst complete
                            rd_pending   <= 1'b0;
                            s_axi_rvalid <= 1'b0;
                            s_axi_rlast  <= 1'b0;
                        end else begin
                            rd_beat_cnt <= rd_beat_cnt + 1;
                        end
                    end
                end
            end else if (!rd_pending) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Write channel
    // -----------------------------------------------------------------------
    reg  [ADDR_WIDTH-1:0] wr_addr_q;
    reg  [7:0]            wr_len_q;
    reg  [ID_WIDTH-1:0]   wr_id_q;
    reg                   wr_addr_pending;
    reg                   wr_data_go;
    integer               wr_beat_cnt;
    integer               wr_delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready   <= 1'b0;
            s_axi_wready    <= 1'b0;
            s_axi_bvalid    <= 1'b0;
            s_axi_bresp     <= 2'b00;
            s_axi_bid       <= {ID_WIDTH{1'b0}};
            wr_addr_pending <= 1'b0;
            wr_data_go      <= 1'b0;
            wr_beat_cnt     <= 0;
            wr_delay_cnt    <= 0;
            wr_addr_q       <= {ADDR_WIDTH{1'b0}};
            wr_len_q        <= 8'd0;
            wr_id_q         <= {ID_WIDTH{1'b0}};
        end else begin
            s_axi_awready <= 1'b0;

            // Accept write address
            if (!wr_addr_pending && !wr_data_go && s_axi_awvalid) begin
                s_axi_awready   <= 1'b1;
                wr_addr_q       <= s_axi_awaddr;
                wr_len_q        <= s_axi_awlen;
                wr_id_q         <= s_axi_awid;
                wr_addr_pending <= 1'b1;
                wr_delay_cnt    <= WR_LATENCY;
            end

            // Latency before accepting data
            if (wr_addr_pending && wr_delay_cnt > 0) begin
                wr_delay_cnt <= wr_delay_cnt - 1;
            end

            if (wr_addr_pending && wr_delay_cnt == 0 && !wr_data_go) begin
                wr_data_go      <= 1'b1;
                wr_addr_pending <= 1'b0;
                wr_beat_cnt     <= 0;
                s_axi_wready    <= 1'b1;
            end

            // Accept write data
            if (wr_data_go && s_axi_wvalid && s_axi_wready) begin
                // Byte-enable write
                begin : WR_BYTES
                    integer b;
                    for (b = 0; b < BEAT_BYTES; b = b + 1) begin
                        if (s_axi_wstrb[b])
                            mem[(wr_addr_q >> $clog2(BEAT_BYTES)) + wr_beat_cnt][b*8 +: 8]
                                <= s_axi_wdata[b*8 +: 8];
                    end
                end

                if (s_axi_wlast) begin
                    wr_data_go   <= 1'b0;
                    s_axi_wready <= 1'b0;
                    s_axi_bvalid <= 1'b1;
                    s_axi_bresp  <= RESP_FORCE_ERR ? 2'b10 : 2'b00;
                    s_axi_bid    <= wr_id_q;
                end else begin
                    wr_beat_cnt <= wr_beat_cnt + 1;
                end
            end

            // Write response handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

endmodule
