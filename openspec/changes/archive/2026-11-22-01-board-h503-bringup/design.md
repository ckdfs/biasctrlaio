# Design: Phase 1 — STM32H503CBT6 Digital Baseboard Bring-Up

## Architecture Decisions
1. **Out-of-Tree Board Layout (`boards/xidian/biasctrl_h503/`)**:
   - Reference Zephyr `v4.5.0` board `embedsky/tq_h503a` for `stm32h503cbtx` Zephyr v2 board YAML (`board.yml`, `Kconfig.biasctrl_h503`, `biasctrl_h503_defconfig`, `biasctrl_h503.dts`, `board.cmake`).
   - Do NOT rely on `tq_h503a` pinouts — `tq_h503a` puts `USART1` on `PB14/PB15`, whereas `20261110-底板` routes `USART1` to `PA9/PA10` and `SPI2` to `PB13/PB14/PB15`.
2. **Clock Tree (`8 MHz HSE -> 250 MHz SYSCLK`)**:
   - Configure `&clk_hse` = `8 MHz`, `&pll` with `div-m = 2`, `mul-n = 125`, `div-p = 2`, `div-q = 2`, `div-r = 2` (`VCO = 500 MHz`, `SYSCLK = 250 MHz`).
3. **FPC2 Pin Safety & Slew Rate Overrides**:
   - Override `spi1_sck_pa5`, `spi1_mosi_pa7`, `spi2_sck_pb13`, `spi2_miso_pb14`, `spi2_mosi_pb15` pinctrl nodes to `slew-rate = "low-speed"` in `biasctrl_h503.dts`.
   - Declare DTS GPIO nodes for the 8 pilot switch pins (`PA0, PA1, PA2, PA3, PA4, PA6, PA15, PB6`) and `PA8` (`spare_pa8`) initialized to output-low (`GPIO_ACTIVE_HIGH` inactive) during Bring-Up.
   - Declare chip-select and reset/control GPIOs (`PB0` CLR#, `PB1` SYNC#, `PB2` LDAC#, `PB12` ADC_CS#, `PA12` ADC_SYNC#) initialized to their hardware-safe inactive levels (`CLR#=1, SYNC#=1, LDAC#=1, ADC_CS#=1, ADC_SYNC#=1`).
4. **Zephyr Flash Runner**:
   - Configure `board.cmake` to use `pyocd` (`board_runner_args(pyocd "--target=stm32h503cbtx")`) so `west flash` works out of the box with `STM32F042` CMSIS-DAP.
5. **Memory Budget & Fast-Fail**:
   - Enable `CONFIG_NEWLIB_LIBC=n` / `CONFIG_PICOLIBC=y`, `CONFIG_PRINTK=y`, minimal console + GPIO + I2C drivers.
   - Any initialization error (missing device readiness) must log an explicit error and halt/indicate fault rather than continuing silently.
