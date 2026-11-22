# Design: Phase 2 — Peripheral Drivers, Two-Axis Abstraction & Split-Board Integration

## Two-Axis Extensibility
1. **Controller Axis (`boards/xidian/<board>/`)**: MCU-specific clocks, `SPI1`/`SPI2`, `I2C1`, `GPDMA1`, and `FPC2` pin assignments stay strictly inside Devicetree and board Kconfig.
2. **Analog Daughterboard Axis (`include/bias/analog_board.h` + `eeprom_schema.h`)**:
   - `analog_board_ops_t` abstracts channel count, DAC DC channel mapping (`VA..VF`), pilot DAC sources (`DITHER1`/`DITHER2`), switch routing matrix (`pilot_mux`), subtractor gain/offset, TIA gain, and calibration EEPROM store.
   - `U21` (`P24C02A`, `0x50`, `256 B`, `8 B` page write with `5 ms` wait) stores a magic header, schema version, board ID, modulator `Vpi`, TIA gain, and zero/affine calibration parameters protected by CRC32.

## Critical Hardware & Real-Time Constraints
- **FPC2 Slew Rate**: All `FPC2` SPI and GPIO outputs (`PA0–PA4, PA6, PA8, PA15, PB6, PA5, PA7, PB0–PB2, PA12, PB12, PB13, PB15`) MUST remain `low-speed`.
- **ADS131M02 (`U9`) Startup & Reset**: Wait `>= 5 ms` after boot for `X2` (`8.192 MHz` active oscillator) stabilization before SPI access; pull `PA12` (`SYNC2`) low for `>= 250 us` (> 2048 `CLKIN` cycles) for hardware reset; verify ID=`0x22`; configure OSR=128 (`64 kSPS` HR mode) + CRC enabled.
- **DAC8568 (`U10`)**: Enable internal `2.5 V` reference; assign `CH0–CH5` (`VOUTA–VOUTF`) to `VA–VF` DC bias and `CH6/CH7` (`VOUTG/VOUTH`) to `DITHER1/DITHER2`; implement DMA TX (`SPI1` + `GPDMA1`) with `SYNC#` rise + `LDAC#` pulse completion and a 100-write startup smoke test (fast-fail on timeout).
- **TS5A23159 (`pilot_mux`)**: 8 switch lines (`PA0, PA1, PA2, PA3, PA4, PA6, PA15, PB6`) default LOW (`REF_SW` disconnected) at boot and force LOW on any fault.
- **OLED1 (`0x3C`) & Dial Debounce**: Single-color `128x64` (`1024 B`) page buffer over `I2C1` (mutex-protected with `U21`); `KEY1–KEY3` (`PB3–PB5`) use `20 ms` `k_work_delayable` software debounce.
