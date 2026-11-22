# Design: Phase 4 — Single-MZM 64 kSPS Real-Time Closed-Loop Control

## Thread & Interrupt Architecture
1. **`DRDY#` Fast Path (Highest Priority, `15.625 us` period)**:
   - Triggered on `PA11` falling edge.
   - Reads `ADS131M02` 12-byte frame via `SPI2` DMA, feeds `CH0` into Goertzel `f0`/`2*f0` and accumulates `CH1` DC monitor sum, pulls next `DITHER1` sample from `dsp_pilot_gen`, and writes `DAC8568` `CH6` (`VOUTG`) via `SPI1` DMA.
   - Every `1280` samples (`20 ms`), posts a fixed-size block result struct to `k_msgq` without blocking; increments `drop_count` and triggers `FAULT` if `k_msgq` overflows or any DMA/CRC timeout occurs.
   - Zero calls to `i2c_*`, `printk`, `k_sleep`, or mutexes on the `64 kSPS` path.
2. **Control Thread (`ctrl_thread`, Priority 5, `2.5 KiB` stack)**:
   - Consumes `10` blocks (`5 Hz`), runs `ctrl_bias_engine`, updates `VA` DC bias on `DAC8568` `CH0`, or steps the two-pass `cal bias` state machine using `dsp_online_lsq`.
   - On any fault, immediately pulls all 8 `TS5A23159` switches LOW and resets bias outputs safely.
3. **CLI & Telemetry Thread (`cli_service`, Priority 8, `2.0 KiB` stack)**:
   - Interrupt/ring-buffer driven `USART1` command parser (`status`, `cal bias`, `start`, `stop`, `target quad|min|max|custom <deg>`, `save`, `load`, `scan vpi`).
   - Persists/restores calibration records to `U21` (`P24C02A` via `eeprom_board`).
