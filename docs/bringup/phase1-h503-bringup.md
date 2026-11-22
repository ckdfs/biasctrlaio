# Phase 1 Bring-Up Hardware Verification (`biasctrl_h503`)

## Flash & Boot Verification (2026-11-22)

- **Firmware Build**: `build/h503_bringup/zephyr/zephyr.elf` (commit `3413de0`)
  - Flash: `22,264 / 110,592 B` (`20.1%`)
  - Static RAM: `5,728 / 24,576 B` (`23.3%`)
- **SWD Probe**: `Devanarchy DAP42 CMSIS-DAP` (`4D3233200643324B07802400`)
- **Flash Command**: `west flash -d build/h503_bringup` (exit `0`, programmed `22,528` bytes at `19.85 kB/s`)
- **Serial Capture** (`/dev/cu.usbmodem1102` @ `115200 bps`):
  ```text
  *** Booting Zephyr OS build 41d10b597304 ***
  biasctrl_h503 bring-up, configured SYSCLK 250000000 Hz
  I2C1 ACK at 0x3C
  I2C1 ACK at 0x50
  I2C1 scan complete, 2 responding addresses
  ```
- **Verified Automatically**:
  - `8 MHz` HSE + `PLL1` (`250 MHz` SYSCLK) clock startup
  - `USART1` (`PA9/PA10`) console output via onboard `STM32F042` VCP
  - `I2C1` (`PB8/PB7`) ACK at `0x3C` (`OLED1`) and `0x50` (`U21` `P24C02A` over `FPC2`)

## Human Visual / Tactile Verification (Confirmed 2026-11-22)
- [x] Confirm `LED1` (`PC13`) blinks steadily at `1 Hz` (`500 ms` toggle) — Confirmed by user
- [x] Toggle/press `U18` (`KEY1`, `KEY2`, `KEY3`) and confirm serial console prints `KEY1 pressed` / `KEY2 pressed` / `KEY3 pressed` — Confirmed by user (software 20 ms debounce scheduled in Phase 2/5)
