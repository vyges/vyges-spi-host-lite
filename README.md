# vyges-spi-host-lite

Lightweight SPI master controller with native TL-UL slave interface. Vyges original IP — no OpenTitan RACL/lifecycle dependencies.

## Features

- SPI modes 0–3 (CPOL/CPHA configurable via register)
- Configurable clock divider: SCLK = clk_i / (2 * (div+1))
- 8/16/32-bit transfer length
- TX and RX FIFOs (configurable depth, default 8)
- Single chip-select output (software-controlled)
- Interrupts: tx_empty, rx_full, idle

## Register Map

| Offset | Name    | Access | Description |
|--------|---------|--------|-------------|
| 0x00   | CTRL    | RW     | [0] enable, [1] cpol, [2] cpha, [4:3] xfer_len (0=8,1=16,2=32) |
| 0x04   | STATUS  | RO     | [0] busy, [1] tx_full, [2] tx_empty, [3] rx_full, [4] rx_empty |
| 0x08   | DIV     | RW     | [15:0] clock divider value |
| 0x0C   | TXDATA  | WO     | Write to push TX FIFO |
| 0x10   | RXDATA  | RO     | Read to pop RX FIFO |
| 0x14   | CSCTRL  | RW     | [0] cs_assert (1 = drive CS low) |
| 0x18   | INTR_EN | RW     | [0] tx_empty_en, [1] rx_full_en, [2] idle_en |
| 0x1C   | INTR_ST | RW     | Write-1-to-clear interrupt status |

## Parameters

| Parameter  | Default | Description |
|------------|---------|-------------|
| FIFO_DEPTH | 8       | TX and RX FIFO depth (entries) |

## Usage

Direct TL-UL crossbar connection — no bridge IP needed.

```yaml
# soc-spec.yaml
peripherals:
  - ip: vyges-spi-host-lite
    instance: u_spi
    base_address: '0x40200000'
    size: 4KB
```

## Target Application

ADXL355 MEMS accelerometer for vibration monitoring / predictive maintenance edge sensor SoC.

## License

Apache-2.0 — see [LICENSE](LICENSE).
