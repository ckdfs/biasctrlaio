## ADDED Requirements

### Requirement: Deterministic 64 kSPS Real-Time Pipeline & Fault Safety
The firmware SHALL execute `64 kSPS` ADC capture, Goertzel update, and `DITHER1` DAC output without blocking primitives and SHALL trip into `FAULT` (disabling all 8 `TS5A23159` switches) on any DMA timeout, CRC error, or queue overflow.

#### Scenario: Zero-Drop 64 kSPS Operation
- **WHEN** `rt_pipeline` runs continuously on `STM32H503CBT6` with `20261110-MZM` attached
- **THEN** `DRDY#` processes `64,000` frames/s with `drop_count == 0` and `crc_error_count == 0` while posting `50 Hz` harmonic blocks to `ctrl_thread`.

### Requirement: Closed-Loop Single-MZM Locking & EEPROM Calibration Persistence
`ctrl_thread` and `cal_service` SHALL execute two-pass `O(1)`-RAM calibration, lock single-MZM bias (`QUAD`, `MAX`, `MIN`, `CUSTOM`), and persist calibration parameters to `U21` (`P24C02A`).

#### Scenario: Calibration Persistence Across Reboot
- **WHEN** the user runs `cal bias` and `save` via CLI, then power-cycles the controller
- **THEN** on next boot `cal_service` loads and CRC-verifies `U21`, restoring `Vpi`, affine matrix `M`, and DC calibration anchors automatically.
