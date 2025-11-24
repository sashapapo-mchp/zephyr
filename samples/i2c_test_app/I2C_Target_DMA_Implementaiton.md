# I2C Target DMA Implementation (SERCOM_G1)

This document summarizes the changes enabling I2C Target (slave) DMA for the Microchip SERCOM_G1 I2C driver and how to use it in an application.

## Overview
- Goal: Support DMA when the device operates as an I2C Target (slave).
- Mechanism: Use Zephyr’s I2C Target buffer-mode API callbacks in combination with the generic DMA API.
  - Host read (target TX): driver asks application for a TX buffer via `buf_read_requested()` and starts a DMA transfer from memory to the SERCOM I2CS data register.
  - Host write (target RX): driver pre-arms a DMA transfer from the SERCOM I2CS data register into an internal buffer, then delivers the data to the application via `buf_write_received()` when the buffer fills and at STOP.

## Configuration
Enable the following Kconfig options in the application:
- `CONFIG_I2C=y`
- `CONFIG_I2C_TARGET=y`
- `CONFIG_I2C_TARGET_BUFFER_MODE=y`
- `CONFIG_I2C_MCHP_DMA_DRIVEN=y`
- Optional tuning: `CONFIG_I2C_MCHP_TARGET_MAX_BUF_SIZE` (default 256)
- Flash storage helpers (boot-count + last-byte tracking) use the board's
  `storage` partition on `flash0`, so enable:
  - `CONFIG_FLASH=y`
  - `CONFIG_FLASH_MAP=y`
  - `CONFIG_FLASH_MAP_LABELS=y`

UART/console and GPIO/LED are optional and unrelated to DMA.

## Devicetree Requirements
Ensure the I2C instance (e.g., SERCOM7 on sam_e54_xpro) provides DMA phandles:
- `dmas = <&dmac RX_CH RX_TRIG>, <&dmac TX_CH TX_TRIG>;`
- `dma-names = "rx", "tx";`
- SERCOM compatible for Microchip implementation: `compatible = "microchip,sercom-g1-i2c"` (the sam_e54_xpro board already sets this).

## Driver Changes
File references:
- drivers/i2c/i2c_mchp_sercom_g1.c:1
- drivers/i2c/Kconfig.mchp:1
- dts/bindings/i2c/microchip,sercom-g1-i2c.yaml:1 (no changes needed, just context)

### Kconfig
- Relax DMA enable dependency to cover target buffer-mode use:
  - `CONFIG_I2C_MCHP_DMA_DRIVEN` now depends on `I2C_CALLBACK || (I2C_TARGET && I2C_TARGET_BUFFER_MODE)`.
- Added `CONFIG_I2C_MCHP_TARGET_MAX_BUF_SIZE` for the internal RX DMA buffer size.

### Mode-aware DMA register selection
- The DMA source/destination register helper functions now choose the correct data register based on current mode:
  - Controller (master): `&I2C_REGS->I2CM.SERCOM_DATA`
  - Target (slave): `&I2C_REGS->I2CS.SERCOM_DATA`
- See: drivers/i2c/i2c_mchp_sercom_g1.c:240

### Target DMA state (buffer mode)
Added to `struct i2c_mchp_dev_data` under `CONFIG_I2C_TARGET_BUFFER_MODE`:
- TX (host reading from target): `tgt_tx_buf`, `tgt_tx_len`, `tgt_tx_dma_active`
- RX (host writing to target): `tgt_rx_buf[CONFIG_I2C_MCHP_TARGET_MAX_BUF_SIZE]`, `tgt_rx_block_size`, `tgt_rx_dma_active`

### Buffer-mode callback registration
`i2c_mchp_target_register()` now stores buffer-mode callbacks when enabled:
- `.buf_read_requested` and `.buf_write_received`
- See: drivers/i2c/i2c_mchp_sercom_g1.c:1760

### DMA helpers (target mode)
New static functions when `CONFIG_I2C_MCHP_DMA_DRIVEN && CONFIG_I2C_TARGET_BUFFER_MODE`:
- `i2c_target_dma_rx_start(const struct device *dev)`
  - PERIPHERAL_TO_MEMORY; dest: internal `tgt_rx_buf`; src: I2CS.DATA; slot: DT “rx”
- `i2c_target_dma_tx_start(const struct device *dev, uint8_t *ptr, uint32_t len)`
  - MEMORY_TO_PERIPHERAL; src: provided buffer; dest: I2CS.DATA; slot: DT “tx”
- DMA completion callbacks
  - RX done: delivers full blocks to `.buf_write_received()` and re-arms for the next block
  - TX done: marks TX inactive and NACKs additional reads
- See: drivers/i2c/i2c_mchp_sercom_g1.c:1320, drivers/i2c/i2c_mchp_sercom_g1.c:1480

### Address-match flow
- On AMATCH, check direction with `SERCOM_I2CS.STATUS.DIR`:
  - Read (host reading):
    - Call `buf_read_requested(&ptr, &len)`; if a buffer is provided, the driver arms TX DMA immediately for the entire buffer (no manual “first byte” write) and issues `RECEIVE_ACK_NAK` once so the host can begin clocking data. Extra clocks after the buffered payload are NACKed automatically. Otherwise, fall back to byte mode.
  - Write (host writing):
    - Call `write_requested()`; start RX DMA into internal buffer (`tgt_rx_buf`).
- See: drivers/i2c/i2c_mchp_sercom_g1.c:1242

### Matched-address tracking
- The driver records which `i2c_target_config` instance matched the incoming address and passes that pointer to every callback, so applications can register multiple configs (one per address) without extra API fields.
- Target callbacks can inspect the config pointer (as the sample does) to pick different reply profiles for each address.

### DRDY handling with DMA active
- While TX or RX DMA is active, the driver masks the DRDY interrupt to avoid unnecessary ISR re-entry. DRDY is automatically re-enabled once DMA stops.
- When DMA is not active, existing byte-wise logic remains.
- See: drivers/i2c/i2c_mchp_sercom_g1.c:1384

### STOP (PREC) handling and finalization
- On `PREC`:
  - If RX DMA active: stop the DMAC channel first so the write-back descriptor captures the remaining beats, then call `dma_get_status()` to compute residual bytes (`block_size - pending_length`); deliver the tail via `.buf_write_received(buf, have)` and clear `tgt_rx_dma_active`.
  - If TX DMA active: stop DMA and clear `tgt_tx_dma_active`.
  - Always call the target’s `.stop()`.
- See: drivers/i2c/i2c_mchp_sercom_g1.c:1440

### Error handling
- On target `ERROR` interrupt: clear flag and call `.stop()`; DMA (if active) is stopped during STOP or by callback if an error status is delivered.
- TX DMA completion with error: driver stops DMA and NACKs further reads.

## Application Usage (sample)
- The sample `samples/i2c_test_app` demonstrates both byte-mode and buffer-mode:
  - Buffer-mode callbacks in `src/main.c:1`:
    - `buf_write_received(config, ptr, len)`: consumes RX DMA chunks
    - `buf_read_requested(config, &ptr, &len)`: provides a TX buffer burst for DMA
  - Configuration enables: `CONFIG_I2C_TARGET_BUFFER_MODE=y`, `CONFIG_I2C_MCHP_DMA_DRIVEN=y` (see `prj.conf:1`).
  - Shell command `i2c stats` (added to the sample) tallies RX/TX DMA blocks via the buffer-mode callbacks so you can monitor transfers without extra instrumentation.
  - The sample registers one `i2c_target_config` per address and keys off the callback’s `config` pointer to differentiate responses: address `0x33` returns `0x11/0x22/0x33/0x44`, while `0x66` returns `0xA1/0xB2/0xC3/0xD4`.
  - Persistent flash-backed state (boot counter + last target byte) is automatically loaded from the `storage_partition` on `flash0`. Use the `storage [info|save|erase]` shell command to inspect or force commits; writes are throttled and auto-flushed a few seconds after the last I2C write.

## Build and Run
- Build: `west build -b sam_e54_xpro samples/i2c_test_app -p auto`
- Flash: `west flash`
- Serial: 115200 8N1 (SERCOM2)
- I2C pins (SERCOM7): SDA `PD08`, SCL `PD09`

## Notes & Limitations
- TX path sends exactly the bytes returned by `buf_read_requested()` for each read; after DMA completes the driver NACKs further bytes until a new start.
- RX path aggregates data into an internal buffer and streams it to `.buf_write_received()` on full blocks and at STOP for any remainder.
- If your host master transfers bursts larger than `CONFIG_I2C_MCHP_TARGET_MAX_BUF_SIZE`, the RX DMA callback will repeatedly deliver blocks.
- Advanced flows (e.g., chained TX buffers for one read) can be added by re-invoking `buf_read_requested()` and re-arming TX DMA mid-burst, but are not implemented here.
- `dma_get_status()` now reports accurate `pending_length` for the DMAC G1 because the driver reads `DMAC_ACTIVE.BTCNT` while active and the write-back descriptor after stopping. Use this when debugging target DMA transfers or validating residual bytes.

## Tuning
- `CONFIG_I2C_MCHP_TARGET_MAX_BUF_SIZE`: increase for larger write bursts from the host.
- The driver automatically masks `DRDY` while target DMA is active, so the ISR only fires again once DMA is halted (STOP, error, or completion).

## File Map
- Driver
  - drivers/i2c/i2c_mchp_sercom_g1.c:1
- Kconfig
  - drivers/i2c/Kconfig.mchp:1
- Sample
  - samples/i2c_test_app/prj.conf:1
  - samples/i2c_test_app/src/main.c:1
- Shell helper `i2c mode target <addr> [secondary]` re-registers the target
    addresses so you can exercise AMODE=two-addresses directly from the console.

---
If you want, we can extend TX to support multi-buffer reads by reissuing `buf_read_requested()` in the TX DMA completion path and restarting DMA until STOP.

