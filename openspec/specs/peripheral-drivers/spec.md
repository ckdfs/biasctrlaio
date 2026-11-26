# peripheral-drivers Specification

## Purpose

Provides the two-axis split-board hardware abstraction, high-speed ADC/DAC DMA drivers, pilot multiplexer control, and shared I2C1 OLED1/dial/EEPROM services.

## Requirements

### Requirement: Two-Axis Split-Board Hardware Abstraction
The firmware SHALL separate MCU Devicetree bindings (`boards/xidian/biasctrl_h503/`) from analog daughterboard capabilities (`include/bias/analog_board.h`, `include/bias/eeprom_schema.h`) so switching either board does not modify `lib/` or high-level `app/` control logic.

#### Scenario: Daughterboard Descriptor & Safe Boot
- **WHEN** the baseboard boots with `20261110-MZM` attached over `FPC2`
- **THEN** `drivers/eeprom_board/` probes `0x50` (`P24C02A`) and validates CRC32 before applying daughterboard parameters, and all 8 `TS5A23159` switches stay LOW until explicitly enabled.

### Requirement: High-Speed ADC/DAC & Low-Speed Ribbon Drivers
The drivers `ads131m02` and `dac8568` SHALL expose both a portable initialization/register API and a zero-lock `64 kSPS` DMA fast path while keeping all `FPC2` pins configured as `low-speed`.

#### Scenario: ADS131M02 Initialization and 64 kSPS Sampling
- **WHEN** `ads131m02` initializes after waiting `>= 5 ms` for `X2` and pulsing `PA12` low for `>= 250 us`
- **THEN** it reads device ID `0x22`, configures OSR=128 HR mode, and produces `DRDY#` interrupts at `64.000 kHz` with CRC-verified 24-bit frames.

#### Scenario: DAC8568 Separation & DMA Smoke Test
- **WHEN** `dac8568` initializes
- **THEN** it enables the internal `2.5 V` reference, passes a 100-transfer DMA TX smoke test without timeout, and independently updates DC channels `CH0–CH5` and pilot channels `CH6–CH7`.

### Requirement: Shared I2C1 OLED1 & Software-Debounced Dial
`oled_lite` (`0x3C`, `<= 1024 B` framebuffer) and `eeprom_board` (`0x50`) SHALL serialize access to `I2C1`, and `KEY1–KEY3` SHALL apply `20 ms` software debounce.

#### Scenario: Debounced Dial Events
- **WHEN** a user toggles or presses `WS-001CB` (`KEY1–KEY3`)
- **THEN** contact bounce shorter than `20 ms` is suppressed so each physical actuation emits one clean event.
