`timescale 1ns/1ps
// =============================================================================
// dma_interrupt_controller.v  –  Interrupt Management
//
// Captures per-channel done and error events.  Supports:
//   • Per-channel interrupt masking (1 = masked / suppressed)
//   • Write-one-to-clear (W1C) status bits via int_clr_* inputs
//   • A single wired-OR IRQ output
//
// INT_STATUS bits are set on the rising edge of the corresponding event
// and cleared when software writes '1' to the corresponding bit of
// INT_STATUS through the AXI4-Lite slave (producing a one-cycle pulse
// on int_clr_done / int_clr_error).
// =============================================================================
module dma_interrupt_controller #(
    parameter NUM_CHANNELS = 2
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // Events from the controller (one-cycle pulses)
    input  wire [NUM_CHANNELS-1:0] ch_done,          // transfer completed OK
    input  wire [NUM_CHANNELS-1:0] ch_error,         // transfer ended with error

    // Mask registers (from AXI4-Lite slave)
    input  wire [NUM_CHANNELS-1:0] int_mask_done,    // 1 = mask done  interrupt
    input  wire [NUM_CHANNELS-1:0] int_mask_error,   // 1 = mask error interrupt

    // W1C clear strobes (one-cycle pulse from AXI4-Lite slave write)
    input  wire [NUM_CHANNELS-1:0] int_clr_done,
    input  wire [NUM_CHANNELS-1:0] int_clr_error,

    // Status (readable via AXI4-Lite)
    output reg  [NUM_CHANNELS-1:0] int_status_done,
    output reg  [NUM_CHANNELS-1:0] int_status_error,

    // Combined interrupt request output (level, active-high)
    output wire                    irq
);

    // -------------------------------------------------------------------------
    // Sticky status registers with W1C clear
    // Priority: set > clear (if event and clear arrive simultaneously, set wins)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_status_done  <= {NUM_CHANNELS{1'b0}};
            int_status_error <= {NUM_CHANNELS{1'b0}};
        end else begin
            int_status_done  <= (int_status_done  | ch_done)  & ~int_clr_done;
            int_status_error <= (int_status_error | ch_error) & ~int_clr_error;
        end
    end

    // -------------------------------------------------------------------------
    // IRQ: any unmasked interrupt pending
    // -------------------------------------------------------------------------
    assign irq = |(int_status_done  & ~int_mask_done) |
                 |(int_status_error & ~int_mask_error);

endmodule
