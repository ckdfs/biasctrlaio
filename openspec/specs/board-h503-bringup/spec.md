# board-h503-bringup Specification

## Purpose

Defines the Zephyr RTOS v4.5.0 board support package, clock tree, pin mapping, FPC2 slew-rate limits, and memory budget contract for the STM32H503CBT6 digital baseboard.

## Requirements

### Requirement: STM32H503CBT6 Zephyr Board Support (`biasctrl_h503`)
The firmware repository SHALL provide an out-of-tree Zephyr board `biasctrl_h503` targeting `STM32H503CBT6` with `8 MHz` HSE PLL to `250 MHz` SYSCLK, `USART1` console (`PA9/PA10`), `I2C1` (`PB7/PB8`), `LED1` (`PC13`), `KEY1-3` (`PB3/PB4/PB5`), `low-speed` slew-rate overrides on all `FPC2` pins, `PA8` configured as low-speed GPIO (`MCO1` disabled), and `pyocd` (`stm32h503cbtx`) runner configuration.

#### Scenario: Clean Build and Memory Budget Compliance
- **WHEN** running `west build -b biasctrl_h503 -d build/h503_bringup app --pristine`
- **THEN** the build MUST succeed and `scripts/check_mem_budget.py` MUST verify `RAM <= 24576 B` and `Flash <= 110592 B`.

#### Scenario: Safe Idle FPC2 Pin States on Boot
- **WHEN** the Bring-Up firmware boots on `20261110-底板`
- **THEN** all 8 pilot switch pins (`PA0, PA1, PA2, PA3, PA4, PA6, PA15, PB6`) and `PA8` MUST remain LOW, and `CLR#`, `SYNC#`, `LDAC#`, `ADC_CS#`, `ADC_SYNC#` MUST remain HIGH.
