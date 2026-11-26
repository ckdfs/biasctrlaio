# Proposal: Phase 2 — Peripheral Drivers, Two-Axis Hardware Abstraction & Split-Board Integration

## Why
Phase 1 confirmed the `STM32H503CBT6` baseboard clocks, `USART1` console, `I2C1` bus (`0x3C` `OLED1` + `0x50` `U21` over `FPC2`), `LED1`, and `KEY1–KEY3`. Phase 2 builds the decoupled Zephyr out-of-tree peripheral driver layer and the two-axis hardware abstraction (`analog_board_ops_t` + `U21` `P24C02A` EEPROM descriptor) so the controller MCU and the analog daughterboard (`20261110-MZM`) can evolve independently, followed by split-board integration verification.

## What Changes
- Define `include/bias/analog_board.h` and `include/bias/eeprom_schema.h` plus Zephyr out-of-tree module metadata (`zephyr/module.yml`, `dts/bindings/`) and enable `SPI1`/`SPI2`/`GPDMA1` (`low-speed` `FPC2` slew) in `boards/xidian/biasctrl_h503/`.
- Implement decoupled drivers under `drivers/`: `pilot_mux` (8-ch `TS5A23159`), `dac8568` (6 DC + 2 Pilot + DMA TX), `ads131m02` (24-bit 4-word CRC + 64 kSPS fast `DRDY#` DMA path), `eeprom_board` (`P24C02A`), `oled_lite` (`<= 1 KiB` framebuffer), and `analog_board` (`20261110-MZM` backend + 20 ms software debounce for `KEY1–KEY3`).
- Upgrade `app/` into a Phase 2 split-board hardware diagnostic CLI while keeping `RAM <= 24576 B` and `Flash <= 110592 B`.

## Out of Scope
- Legacy single-board hardware compatibility (`DAC` single-channel software superposition compatibility is explicitly excluded).
- Goertzel DSP harmonic demodulation and PID closed-loop bias control (Phase 3 and Phase 4).
