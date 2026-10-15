# DMA Controller – Register Map

## Address Map

Each channel has a 64-byte (0x40) register bank.  
Channel *n* register base = `n × 0x40`.

| Channel | Base Address |
|---------|-------------|
| 0 | 0x0000_0000 |
| 1 | 0x0000_0040 |
| 2 | 0x0000_0080 |
| 3 | 0x0000_00C0 |

## Per-Channel Registers

| Offset | Name | Access | Reset | Description |
|--------|------|--------|-------|-------------|
| 0x00 | CONTROL | RW | 0x00 | Channel control (see bits below) |
| 0x04 | STATUS | RO | 0x01 | Channel status (see bits below) |
| 0x08 | SRC_ADDR | RW* | 0x00 | Source byte address |
| 0x0C | DST_ADDR | RW* | 0x00 | Destination byte address |
| 0x10 | XFER_LEN | RW* | 0x00 | Transfer size in bytes |
| 0x14 | BURST_CFG | RW* | 0x0F | AXI ARLEN value (beats − 1) |
| 0x18 | INT_STATUS | RW1C | 0x00 | Interrupt status |
| 0x1C | INT_MASK | RW | 0x00 | Interrupt mask |
| 0x20 | ERROR_CODE | RO | 0x00 | Error reason code |

*\* Writes ignored when STATUS[busy]=1.*

## Register Bit Fields

### CONTROL (0x00)
| Bit | Name | Description |
|-----|------|-------------|
| 0 | enable | Enable this channel |
| 1 | start | Write 1 to trigger transfer (self-clearing one-cycle pulse) |
| 2 | soft_reset | Reset channel state machine and status (self-clearing) |
| 3 | irq_en | Reserved for per-channel IRQ routing (future) |

### STATUS (0x04) — Read Only
| Bit | Name | Description |
|-----|------|-------------|
| 0 | idle | Channel is idle (no transfer in progress) |
| 1 | busy | Transfer in progress |
| 2 | done | Last transfer completed successfully |
| 3 | error | Last transfer ended in error |

*Note: done and error persist until soft_reset is issued.*

### SRC_ADDR / DST_ADDR (0x08 / 0x0C)
Full 32-bit byte address. Must be naturally aligned to DATA_WIDTH/8 bytes (4-byte alignment for 32-bit bus).

### XFER_LEN (0x10)
Transfer length in bytes. Must be a non-zero multiple of DATA_WIDTH/8.

### BURST_CFG (0x14)
| Bits | Name | Description |
|------|------|-------------|
| 7:0 | max_burst_len | AXI ARLEN value (beats − 1). Max burst = max_burst_len + 1 beats. Default = 0x0F (16 beats). |

### INT_STATUS (0x18) — Write-1-to-Clear
| Bit | Name | Description |
|-----|------|-------------|
| 0 | done_irq | Set when transfer completes; write 1 to clear |
| 1 | error_irq | Set on transfer error; write 1 to clear |

### INT_MASK (0x1C)
| Bit | Name | Description |
|-----|------|-------------|
| 0 | mask_done | 1 = suppress done interrupt from reaching irq pin |
| 1 | mask_error | 1 = suppress error interrupt from reaching irq pin |

### ERROR_CODE (0x20) — Read Only
| Code | Name | Trigger |
|------|------|---------|
| 0x01 | ERR_ALIGN | SRC/DST/LEN not aligned to beat size |
| 0x02 | ERR_ZERO_LEN | XFER_LEN = 0 |
| 0x04 | ERR_RD_RESP | AXI RRESP ≠ OKAY |
| 0x08 | ERR_WR_RESP | AXI BRESP ≠ OKAY |

## Programming Sequence

```
// 1. Set addresses and length
WRITE CH0+0x08 = SRC_ADDR
WRITE CH0+0x0C = DST_ADDR
WRITE CH0+0x10 = XFER_LEN
WRITE CH0+0x14 = BURST_CFG (e.g. 0x0F for 16-beat bursts)

// 2. Optionally unmask interrupts
WRITE CH0+0x1C = 0x00 (unmask both done and error)

// 3. Enable and start
WRITE CH0+0x00 = 0x03 (enable=1, start=1)

// 4. Poll or wait for IRQ
// Poll: READ CH0+0x04 → wait for bit[2] (done) or bit[3] (error)
// IRQ: handle interrupt, read INT_STATUS, clear with W1C

// 5. On error: read ERROR_CODE, soft-reset, reprogram
WRITE CH0+0x00 = 0x04 (soft_reset)
WRITE CH0+0x00 = 0x00 (release reset)
```
