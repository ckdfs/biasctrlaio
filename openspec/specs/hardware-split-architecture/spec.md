# Hardware Split Architecture Specification

## Overview
All firmware targets the split two-board hardware architecture:
- **Digital Controller Baseboard** (`20261110-底板`, currently populated with `STM32H503CBT6` LQFP48)
- **Analog Daughterboard** (`20261110-MZM`, interconnected via 30-pin `FPC2`)

Legacy single-board hardware compatibility is explicitly excluded.

## Requirements

### Requirement: Digital Baseboard Pin & Peripheral Contract (`STM32H503CBT6`)
The Zephyr board definition MUST configure the exact pin mapping, clock tree, and electrical slew rates of `20261110-底板`:
- **System Clock**: `8 MHz` external passive crystal (`PH0`/`PH1`, `X1`) driving `PLL1` (`M=2, N=125, P=2`) to `SYSCLK = HCLK = 250 MHz`.
- **Console / Debug UART**: `USART1` (`PA9` TX, `PA10` RX, `AF7`) bridged through `U19` (`STM32F042` DAPLink VCP).
- **DAC SPI Bus (`SPI1`)**: `PA5` (`SCK`), `PA7` (`MOSI`, `AF5`), `PB1` (`SYNC#`), `PB2` (`LDAC#`), `PB0` (`CLR#`).
- **ADC SPI Bus (`SPI2`)**: `PB13` (`SCK`), `PB14` (`MISO`), `PB15` (`MOSI`, `AF5`), `PB12` (`CS#`), `PA11` (`DRDY#` falling-edge EXTI), `PA12` (`SYNC#/RESET#`).
- **Pilot Multiplexer Switches**: `PA0, PA1, PA2, PA3, PA4, PA6, PA15, PB6` as push-pull outputs, initialized LOW (`REF_SW` / disconnected) on boot and fault.
- **Shared I2C Bus (`I2C1`)**: `PB8` (`SCL`), `PB7` (`SDA`, `AF4` open-drain with external `4.7 kΩ` pull-ups) shared by `OLED1` (`0x3C`) and `U21` `P24C02A` (`0x50`).
- **User Interface & Spare**: `PB3/PB4/PB5` (`KEY1-3`, active-low dial switch `WS-001CB`), `PC13` (`LED1`, active-high), `PA8` configured as low-speed GPIO (`MCO1` disabled).
- **SWD Runner**: `pyocd` targeting `stm32h503cbtx`.

#### Scenario: FPC2 Slew Rate Protection
- **WHEN** any digital signal crosses `FPC2` (`PA0-PA4, PA6, PA8, PA15, PB6, PA5, PA7, PB0-PB2, PA12, PB12, PB13, PB15`)
- **THEN** its Zephyr pinctrl / GPIO property MUST override `slew-rate` to `"low-speed"` (not `very-high-speed`) to suppress ribbon-cable inductive ringing.

#### Scenario: Analog Board Clock Startup & Reset Sequencing
- **WHEN** firmware initializes `ADS131M02` (`U9`) on the analog daughterboard
- **THEN** it MUST wait at least `5 ms` after power-up for local `8.192 MHz` oscillator `X2` stabilization and assert `PA12` (`SYNC2`) low for at least `250 µs` (> 2048 `CLKIN` cycles).

### Requirement: Analog Daughterboard 6-DC + 2-Pilot Topology
The `20261110-MZM` daughterboard separates DC bias generation from AC pilot tone injection:
- `DAC8568` `CH0–CH5` (`VOUTA–VOUTF`) provide filtered DC bias voltages `VA–VF`.
- `DAC8568` `CH6` (`VOUTG` -> `DITHER1`) and `CH7` (`VOUTH` -> `DITHER2`) generate AC pilot tones routed through four `TS5A23159` dual SPDT switches (`U14, U16, U17, U20`):
  - `DITHER1` routes to `VA` (`PA0`), `VB` (`PA1`), `VD` (`PA4`), `VE` (`PA6`).
  - `DITHER2` routes to `VB` (`PA2`), `VC` (`PA3`), `VE` (`PA15`), `VF` (`PB6`).

### Requirement: Two-Axis Extensibility & MCU Resource Budget
- **MCU Portability**: Switching controller MCUs MUST require only a new Zephyr board under `boards/xidian/<board>/` without editing `drivers/`, `lib/`, or `app/`.
- **Analog Daughterboard Portability**: Analog frontend capabilities, channel routing, gains, and calibration metadata MUST be abstracted behind `analog_board_ops_t` and `U21` (`P24C02A`) EEPROM descriptors.
- **STM32H503CBT6 Hard Budget**: Firmware static RAM (`data + bss`) MUST NOT exceed `24576 B` (`24 KiB` of `32 KiB`), and Flash (`text + data`) MUST NOT exceed `110592 B` (`108 KiB` of `128 KiB`).
