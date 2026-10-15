`timescale 1ns/1ps
// =============================================================================
// dma_arbiter.v  –  Round-Robin Channel Arbiter
//
// Selects one of NUM_CHANNELS requesting channels using a round-robin policy
// that starts searching one position after the last-granted channel.
// Once a grant is issued the arbiter is busy and ignores new requests until
// the active transfer asserts xfer_done for one cycle.
//
// Outputs:
//   grant       – one-hot channel grant vector
//   grant_idx   – binary index of granted channel
//   grant_valid – asserted whenever at least one bit in grant is set
// =============================================================================
module dma_arbiter #(
    parameter NUM_CHANNELS = 2
) (
    input  wire                            clk,
    input  wire                            rst_n,

    input  wire [NUM_CHANNELS-1:0]         request,      // pending channel requests
    input  wire                            xfer_done,    // active transfer finished

    output reg  [NUM_CHANNELS-1:0]         grant,        // one-hot
    output reg  [$clog2(NUM_CHANNELS)-1:0] grant_idx,    // binary
    output wire                            grant_valid
);

    localparam CH_W = $clog2(NUM_CHANNELS);

    reg [CH_W-1:0]         last_grant;
    reg                    busy;

    assign grant_valid = |grant;

    // -------------------------------------------------------------------------
    // Combinational arbitration  (round-robin rotation)
    // Scan NUM_CHANNELS positions starting from (last_grant + 1) mod NUM_CHANNELS
    // -------------------------------------------------------------------------
    integer arb_k;
    integer arb_idx;
    reg                    found;
    reg [CH_W-1:0]         next_grant;
    reg [NUM_CHANNELS-1:0] next_grant_oh;

    always @(*) begin
        found         = 1'b0;
        next_grant    = {CH_W{1'b0}};
        next_grant_oh = {NUM_CHANNELS{1'b0}};

        for (arb_k = 0; arb_k < NUM_CHANNELS; arb_k = arb_k + 1) begin
            arb_idx = last_grant + 1 + arb_k;
            if (arb_idx >= NUM_CHANNELS) arb_idx = arb_idx - NUM_CHANNELS;
            if (!found && request[arb_idx]) begin
                found                  = 1'b1;
                next_grant             = arb_idx[CH_W-1:0];
                next_grant_oh[arb_idx] = 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Sequential state
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant      <= {NUM_CHANNELS{1'b0}};
            grant_idx  <= {CH_W{1'b0}};
            last_grant <= {CH_W{1'b0}};
            busy       <= 1'b0;
        end else begin
            if (busy) begin
                // Hold until transfer completes
                if (xfer_done) begin
                    busy  <= 1'b0;
                    grant <= {NUM_CHANNELS{1'b0}};
                end
            end else if (found) begin
                grant      <= next_grant_oh;
                grant_idx  <= next_grant;
                last_grant <= next_grant;
                busy       <= 1'b1;
            end
        end
    end

endmodule
