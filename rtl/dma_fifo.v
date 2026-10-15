`timescale 1ns/1ps
// =============================================================================
// dma_fifo.v  –  Synchronous Parameterized FIFO
//
// Parameters:
//   DATA_WIDTH : width of each FIFO word (default 32)
//   DEPTH      : number of words; MUST be a power of 2 (default 16)
//
// The MSB of the pointer pair is used to distinguish full from empty so that
// simultaneous read and write is handled correctly without a data-count
// comparison.
//
// full  = pointers differ only in their MSBs (wrapped differently)
// empty = pointers are equal
// =============================================================================
module dma_fifo #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 16
) (
    input  wire                   clk,
    input  wire                   rst_n,

    // Write port
    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    output wire                   full,
    output reg                    overflow,   // write attempted while full (1 cycle)

    // Read port
    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  rd_data,
    output wire                   empty,
    output reg                    underflow,  // read attempted while empty (1 cycle)

    // Word count (0 … DEPTH)
    output wire [$clog2(DEPTH):0] count
);

    localparam PTR_W = $clog2(DEPTH); // index bits; pointer is PTR_W+1 wide

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [PTR_W:0]        wr_ptr;   // one extra bit for full/empty detection
    reg [PTR_W:0]        rd_ptr;

    // ---------------------------------------------------------------------------
    // Status flags
    // ---------------------------------------------------------------------------
    // full  : MSBs differ, lower bits equal  →  write has wrapped N/2 more than read
    assign full  = (wr_ptr[PTR_W] != rd_ptr[PTR_W]) &&
                   (wr_ptr[PTR_W-1:0] == rd_ptr[PTR_W-1:0]);
    assign empty = (wr_ptr == rd_ptr);
    assign count = wr_ptr - rd_ptr;

    // Read data is combinational from the current rd_ptr slot
    assign rd_data = mem[rd_ptr[PTR_W-1:0]];

    // ---------------------------------------------------------------------------
    // Sequential logic
    // ---------------------------------------------------------------------------
    integer init_idx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr    <= {(PTR_W+1){1'b0}};
            rd_ptr    <= {(PTR_W+1){1'b0}};
            overflow  <= 1'b0;
            underflow <= 1'b0;
            // Clear memory on reset (simulation convenience; synthesisers may omit)
            for (init_idx = 0; init_idx < DEPTH; init_idx = init_idx + 1)
                mem[init_idx] <= {DATA_WIDTH{1'b0}};
        end else begin
            overflow  <= 1'b0;   // pulse for one cycle only
            underflow <= 1'b0;

            // Write side
            if (wr_en) begin
                if (!full) begin
                    mem[wr_ptr[PTR_W-1:0]] <= wr_data;
                    wr_ptr <= wr_ptr + 1'b1;
                end else begin
                    overflow <= 1'b1;
                end
            end

            // Read side (advance pointer; data already visible combinationally)
            if (rd_en) begin
                if (!empty) begin
                    rd_ptr <= rd_ptr + 1'b1;
                end else begin
                    underflow <= 1'b1;
                end
            end
        end
    end

endmodule
