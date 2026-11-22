# Proposal: Phase 1 — STM32H503CBT6 Digital Baseboard Bring-Up

## Why
Before integrating the analog daughterboard (`20261110-MZM`) over `FPC2`, we must validate the new Zephyr RTOS v4.5.0 board definition for the `STM32H503CBT6` baseboard (`20261110-底板`) in isolation: clocks (`8 MHz HSE -> 250 MHz SYSCLK`), UART console (`USART1` via `STM32F042`), user LED (`PC13`), dial switch (`PB3/PB4/PB5`), `I2C1` bus scan (`PB7/PB8`), safe default states for all `FPC2` pins (including `low-speed` slew rate and `PA8` MCO1 disabled), and strict RAM/Flash budget compliance.

## What Changes
- Add out-of-tree Zephyr board `boards/xidian/biasctrl_h503/` (DTS, pinctrl overrides, defconfig, Kconfig, `pyocd` runner).
- Add minimal Bring-Up Zephyr application under `app/` (`prj.conf`, `CMakeLists.txt`, `app/src/main.c`) demonstrating boot banner, heartbeat LED, dial key events, `I2C1` probe (`0x3C` / `0x50` without failing if analog EEPROM is idle), and `FPC2` safe idle pin states.
- Enforce `scripts/check_mem_budget.py` (`RAM <= 24576 B`, `Flash <= 110592 B`) on the compiled `zephyr.elf`.

## Out of Scope
- `SPI1` (`DAC8568`) / `SPI2` (`ADS131M02`) data transfers and 64 kSPS ISR pipeline (deferred to Phase 2).
- `OLED1` graphics rendering or `P24C02A` schema parsing beyond basic I2C ACK probing (deferred to Phase 2).
- DSP / Goertzel / PID control loops (deferred to Phase 3–4).
