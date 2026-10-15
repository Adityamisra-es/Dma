# High-Performance DMA Controller

A parameterized, synthesizable AXI4 DMA controller for VLSI/RTL portfolio and internship projects.

## Features

- **AXI4-Lite** CPU configuration slave
- **AXI4** master for high-throughput data movement  
- **Memory-to-memory** and **memory-to-peripheral** transfers
- **2 DMA channels** (parameterizable up to 4) with **round-robin arbitration**
- **Burst transfers** up to 256 beats; **4 KB boundary auto-splitting**
- **Parameterizable FIFO** between read and write engines
- **Interrupt controller** with per-channel done/error, masking, and W1C status
- **Error detection**: alignment, zero-length, AXI RRESP/BRESP errors
- **Full testbench suite** with AXI memory model and data scoreboard

## Tool Requirements

| Tool | Version | Windows Path |
|------|---------|-------------|
| Icarus Verilog | 12.0+ | `C:\iverilog\bin\iverilog.exe` |
| VVP runtime | 12.0+ | `C:\iverilog\bin\vvp.exe` |
| GTKWave | any | `C:\iverilog\gtkwave\bin\gtkwave.exe` |

Download Icarus Verilog for Windows: https://bleyer.org/icarus/

## Project Structure

```
dma-controller/
├── rtl/
│   ├── dma_top.v                  Top-level integration
│   ├── dma_axi_lite_slave.v       CPU config register bank
│   ├── dma_controller.v           Main FSM
│   ├── dma_address_generator.v    Address sequencing
│   ├── dma_burst_engine.v         AXI4 R/W burst FSMs
│   ├── dma_fifo.v                 Synchronous FIFO
│   ├── dma_arbiter.v              Round-robin channel arbiter
│   └── dma_interrupt_controller.v IRQ management
├── tb/
│   ├── axi_memory_model.v         AXI4 slave RAM (sim only)
│   ├── tb_dma_registers.sv        Register R/W test
│   ├── tb_dma_single_transfer.sv  1-beat M2M test
│   ├── tb_dma_burst_transfer.sv   Burst + 4KB boundary test
│   ├── tb_dma_errors.sv           Error handling test
│   └── tb_dma_multichannel.sv     2-channel round-robin test
├── sim/
│   ├── Makefile                   GNU Make build rules
│   └── run_tests.ps1              PowerShell test runner
├── docs/
│   ├── architecture.md            Block diagram + module descriptions
│   ├── register_map.md            Full register reference
│   └── verification_report.md     Coverage + run instructions
└── README.md
```

## Quick Start

```powershell
# Run all tests
cd e:\dma\sim
.\run_tests.ps1

# Run one test
.\run_tests.ps1 -Test tb_dma_single_transfer

# View waveforms
C:\iverilog\gtkwave\bin\gtkwave.exe tb_dma_single_transfer.vcd
```

## Register Map Summary

| Offset | Register | Access | Description |
|--------|----------|--------|-------------|
| ch×0x40 + 0x00 | CONTROL | RW | enable, start, soft_reset, irq_en |
| ch×0x40 + 0x04 | STATUS | RO | idle, busy, done, error |
| ch×0x40 + 0x08 | SRC_ADDR | RW* | Source byte address |
| ch×0x40 + 0x0C | DST_ADDR | RW* | Destination byte address |
| ch×0x40 + 0x10 | XFER_LEN | RW* | Transfer length (bytes) |
| ch×0x40 + 0x14 | BURST_CFG | RW* | AXI ARLEN value (max_beats-1) |
| ch×0x40 + 0x18 | INT_STATUS | RW1C | done_irq, error_irq |
| ch×0x40 + 0x1C | INT_MASK | RW | mask_done, mask_error |
| ch×0x40 + 0x20 | ERROR_CODE | RO | Error reason (0x01/0x02/0x04/0x08) |

*\* Locked while STATUS[busy]=1*

## Design Parameters

```verilog
dma_top #(
    .NUM_CHANNELS (2),   // 1–4 channels
    .DATA_WIDTH   (32),  // data bus width
    .ADDR_WIDTH   (32),  // address width
    .FIFO_DEPTH   (16),  // FIFO words (power-of-2)
    .ID_WIDTH     (4)    // AXI ID width
) u_dma ( ... );
```

## Known Limitations

- One AXI4 master port shared across all channels (serialize via arbiter)
- 1 outstanding read + 1 outstanding write per transfer (conservative, correct)
- Fixed beat size = DATA_WIDTH/8 bytes (no narrow transfers)
- No scatter-gather support

## References

- ARM IHI0022E – AMBA AXI4 Protocol Specification  
- ARM IHI0022E §A3 – 4 KB boundary restriction  
- Icarus Verilog: http://iverilog.icarus.com/
