# DMA Controller – Verification Report

## Test Suite

| # | Testbench | Description | Status |
|---|-----------|-------------|--------|
| 1 | `tb_dma_registers` | AXI4-Lite register R/W, reset values, read-only enforcement | Run to verify |
| 2 | `tb_dma_single_transfer` | 1-beat M2M: seed source, DMA, compare destination | Run to verify |
| 3 | `tb_dma_burst_transfer` | 8-beat, 256-byte multi-burst, 4KB boundary | Run to verify |
| 4 | `tb_dma_errors` | Zero-len, misalign, RRESP/BRESP injection, W1C, recovery | Run to verify |
| 5 | `tb_dma_multichannel` | 2-channel concurrent start, round-robin arbitration, scoreboard | Run to verify |

## How to Run

### Windows (PowerShell)
```powershell
cd e:\dma\sim
.\run_tests.ps1
```

### Single test
```powershell
.\run_tests.ps1 -Test tb_dma_registers
```

### Make (if available)
```powershell
cd e:\dma\sim
make all
```

### Manual compile + run
```powershell
$IV = "C:\iverilog\bin\iverilog.exe"
$VVP = "C:\iverilog\bin\vvp.exe"

# Register test (slave-only)
& $IV -g2005-sv -o tb_dma_registers.vvp `
    ..\rtl\dma_axi_lite_slave.v `
    ..\tb\tb_dma_registers.sv -s tb_dma_registers
& $VVP tb_dma_registers.vvp

# Full DMA single transfer
& $IV -g2005-sv -o tb_dma_single_transfer.vvp `
    ..\rtl\dma_fifo.v ..\rtl\dma_address_generator.v `
    ..\rtl\dma_arbiter.v ..\rtl\dma_interrupt_controller.v `
    ..\rtl\dma_axi_lite_slave.v ..\rtl\dma_burst_engine.v `
    ..\rtl\dma_controller.v ..\rtl\dma_top.v `
    ..\tb\axi_memory_model.v `
    ..\tb\tb_dma_single_transfer.sv -s tb_dma_single_transfer
& $VVP tb_dma_single_transfer.vvp
```

### View waveforms
```powershell
C:\iverilog\gtkwave\bin\gtkwave.exe tb_dma_single_transfer.vcd
```

## Test Coverage

| Feature | Covered By |
|---------|-----------|
| AXI4-Lite write handshake | T1 |
| AXI4-Lite read handshake | T1 |
| AXI4-Lite AWVALID/WVALID independence | T1 |
| Register reset values | T1 |
| Register write-protect when busy | T1 |
| Single-beat DMA transfer | T2 |
| Completion interrupt | T2, T3 |
| Multi-beat burst | T3 |
| Multi-burst (len > max_burst) | T3 |
| 4 KB boundary splitting | T3 |
| AXI backpressure (FIFO full → stall RREADY) | T3 |
| Zero-length error | T4 |
| Alignment error | T4 |
| AXI read error (RRESP=SLVERR) | T4 |
| AXI write error (BRESP=SLVERR) | T4 |
| INT_STATUS W1C | T4 |
| Recovery after error + soft-reset | T4 |
| Multi-channel concurrent start | T5 |
| Round-robin arbitration | T5 |
| Per-channel data integrity | T5 |

## Known Limitations

1. **1 outstanding transaction per direction**: The burst engine supports one read and one write in flight at a time. This is correct AXI4 but not maximum throughput. Pipelining is left as a future enhancement.

2. **Fixed beat size**: Always `DATA_WIDTH/8` bytes (4 for 32-bit). Narrow transfers and byte-enables on partial beats are not implemented.

3. **Shared AXI4 port**: All channels share one AXI4 master port (via the arbiter). True simultaneous multi-channel operation would require multiple master ports.

4. **No scatter-gather**: Descriptors are simple src/dst/len triples; linked-list or scatter-gather modes are not supported.

5. **Address width**: 32-bit addresses only. 64-bit addressing requires parameter change and minor RTL updates.
