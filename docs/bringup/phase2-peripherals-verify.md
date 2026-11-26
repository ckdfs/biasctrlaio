# Phase 2 Split-Board Peripherals Verification (`biasctrl_h503` + `20261110-MZM`)

## Flash & Automated Diagnostics (2026-11-22)

- **Firmware Build**: `build/h503_p2/zephyr/zephyr.elf`
  - Flash: `44,920 / 110,592 B` (`40.6%`)
  - Static RAM: `9,404 / 24,576 B` (`38.3%`)
- **SWD Probe**: `Devanarchy DAP42 CMSIS-DAP` (`4D3233200643324B07802400`)
- **Flash Command**: `west flash -d build/h503_p2` (exit `0`, erased `49,152` B, programmed `45,056` B at `20.51 kB/s`)
- **Board / Clock Update**: `PLL1` `div-q = 5` (`PLL1_Q = 100 MHz`, `SYSCLK = 250 MHz`), `SPI1` `6.25 MHz` (`bias_dac`), `SPI2` `12.5 MHz` (`bias_adc`), all `FPC2` pins kept at `slew-rate = "low-speed"`.
- **Serial Capture** (`/dev/cu.usbmodem102` @ `115200 bps`, raw log `.agent-bridge/p2_hw_verify_uart.log`):
  ```text
  *** Booting Zephyr OS build 41d10b597304 ***
  biasctrl_h503 Phase 2 diag, SYSCLK 250000000 Hz
  analog_board init (20261110-MZM): 0
  oled_lite init: 0
  keys debounce init (20 ms): 0
  eeprom probe: 0
  ads131m02 init (64 kSPS): 0
  ```
- **Verified Automatically**:
  - **`I2C1` (`PB8/PB7`)**: ACK at `0x3C` (`OLED1`) and `0x50` (`U21` `P24C02A`).
  - **`DAC8568` (`SPI1` + `GPDMA1`)**: Internal `2.5 V` ref enabled, 100-write DMA TX startup smoke test passed (`init=0 status=0`), `dac dc 0 32768 -> 0`, `dac pilot 0 32768 -> 0`, `dac dc/pilot 0 0 -> 0`.
  - **`TS5A23159` (`pilot_mux`)**: All 8 switches default LOW on boot; `mux 0 0 1 -> 0`, `mux 0 1 1 -> 0`, dual-pilot conflict rejected (`mux 1 1 1 -> -16` `-EBUSY`), unmapped route rejected (`mux 0 2 1 -> -2` `-ENOENT`), `mux off -> 0`.
  - **`ADS131M02` (`SPI2` + `GPDMA1` + `PA11` `DRDY#` EXTI)**:
    - `ID=0x2205`, `STAT=0x0103`, `MODE=0x1111` (`24-bit`, CRC-16-CCITT enabled).
    - `64 kSPS` (`CLK=0x0322`, TBM): `1000/1000` CRC-verified frames in `15,607 us` (`64,009 SPS`, `err=0`, `stop=0`; `CH0 min=-7362063 mean=5151 max=6154969`, `CH1 min=-14407 mean=4503 max=6615331`).
    - `32 kSPS` (`CLK=0x0302`, OSR=128 HR): `1000/1000` CRC-verified frames in `31,215 us` (`32,003 SPS`, `err=0`, `stop=0`; `CH0 min=-117612 mean=1337 max=3676`, `CH1 min=-123830 mean=-12922 max=-10668`).
  - **`P24C02A` (`eeprom_board`)**: `eeprom write-default -> 0` (`crc=0xA0D5AC42`) persisted across reset (`eeprom probe: 0`, `id=20261110-MZM ver=1 len=148 tia=10000 ohm`).
  - **`OLED1` (`oled_lite`) & `KEY1–KEY3`**: `oled_lite` upgraded to `5x7` (`6x8` cell) font (`oled P2 HW VERIFY OK -> 0`), `keys debounce init (20 ms): 0`, `KEY1=0(0) KEY2=0(0) KEY3=0(0)`.

## Human Visual / Tactile Verification (Confirmed 2026-11-22)
- [x] Confirm `OLED1` clearly displays `BIASCTRL H503` and `P2 HW VERIFY OK` using the `5x7` font — Confirmed by user
- [x] Confirm `LED1` (`PC13`) blinks steadily at `1 Hz` (`500 ms` toggle, indicating zero boot faults) — Confirmed by user
- [x] Toggle/press `KEY1`, `KEY2`, `KEY3` (`WS-001CB`) and confirm clean single-event `KEYx pressed` / `released` messages on `115200 bps` UART (`20 ms` debounce) — Confirmed by user
