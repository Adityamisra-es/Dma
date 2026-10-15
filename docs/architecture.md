# DMA Controller – Architecture

## Overview

A parameterized, synthesizable AXI4-compliant Direct Memory Access controller supporting memory-to-memory and memory-to-peripheral transfers via a round-robin multi-channel scheduler.

```
                    CPU
                     │  AXI4-Lite (config)
                     ▼
          ┌──────────────────────┐
          │  dma_axi_lite_slave  │  Register file (per-channel banks)
          └──────────┬───────────┘
                     │ ch_enable, ch_start, ch_src/dst/len …
                     ▼
          ┌──────────────────────┐
          │    dma_controller    │  Main FSM (one channel at a time)
          │   ┌────────────┐    │
          │   │dma_arbiter │    │  Round-robin channel selection
          │   └────────────┘    │
          └───┬───────────┬─────┘
              │           │
              ▼           ▼
    ┌──────────────┐  ┌─────────────────────────┐
    │ dma_address_ │  │    dma_burst_engine      │
    │  generator   │  │  ┌────────┐ ┌─────────┐ │
    │ (src + dst)  │  │  │ RD FSM│ │ WR FSM  │ │
    └──────────────┘  │  └───┬────┘ └────┬────┘ │
                      └──────┼───────────┼───────┘
                             │           │
                         AXI4 AR/R   AXI4 AW/W/B
                             │           │
                             └────┬──────┘
                                  │
                           ┌──────┴──────┐
                           │  dma_fifo   │   Elastic buffer
                           └─────────────┘
                                  │
                         ┌────────┴────────┐
                         │  dma_interrupt_ │
                         │   controller    │
                         └────────┬────────┘
                                  │ irq
```

## Module Descriptions

| Module | Lines | Purpose |
|--------|-------|---------|
| `dma_top.v` | ~200 | Integration, port mapping |
| `dma_axi_lite_slave.v` | ~280 | AXI4-Lite register bank |
| `dma_controller.v` | ~300 | Main FSM + arbitration coordination |
| `dma_burst_engine.v` | ~250 | AXI4 read/write FSMs |
| `dma_fifo.v` | ~80 | Synchronous parameterized FIFO |
| `dma_address_generator.v` | ~60 | Address sequencing |
| `dma_arbiter.v` | ~100 | Round-robin arbiter |
| `dma_interrupt_controller.v` | ~60 | IRQ management |

## Transfer Flow

```
CPU writes SRC_ADDR, DST_ADDR, XFER_LEN, BURST_CFG
CPU writes CONTROL[enable=1, start=1]
     ↓
dma_controller: IDLE → ARB → LOAD → VALIDATE → COMPUTE
     ↓
Burst loop:
  COMPUTE: burst_beats = min(remaining, max_burst, src_4kb, dst_4kb)
  BURST_INIT: issue rd_start + wr_start simultaneously
  BURST_WAIT: read engine fills FIFO; write engine drains FIFO
  UPDATE: advance src/dst by burst_bytes; decrement bytes_remain
  → back to COMPUTE if bytes_remain > 0
     ↓
DONE: STATUS[done]=1, irq_done pulse
     ↓
interrupt_controller: INT_STATUS[done]=1, irq → CPU
```

## AXI4 Protocol Compliance

- **Handshake**: transfer only on `VALID && READY` at rising edge
- **VALID stability**: once asserted, VALID and payload held until handshake
- **Backpressure**: read RREADY deasserted when FIFO full; write WVALID deasserted when FIFO empty
- **LAST**: asserted on final beat of each burst; computed from ARLEN/AWLEN
- **4 KB boundary**: controller computes `min(remaining, page_limit)` per burst
- **Burst type**: always INCR (2'b01)
- **Beat size**: always `log2(DATA_WIDTH/8)` (natural word size)
- **Response checking**: RRESP ≠ OKAY → ERR_RD_RESP; BRESP ≠ OKAY → ERR_WR_RESP

## Outstanding Transactions

- **1 outstanding read** per channel (sequenced; next read starts after rd_done)
- **1 outstanding write** per channel (paired with read)
- This conservative approach eliminates reorder complexity while remaining fully AXI4-legal

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| NUM_CHANNELS | 2 | Number of independent DMA channels |
| DATA_WIDTH | 32 | AXI data bus width (bits) |
| ADDR_WIDTH | 32 | AXI address width (bits) |
| FIFO_DEPTH | 16 | FIFO words (must be power of 2) |
| ID_WIDTH | 4 | AXI ID width |
