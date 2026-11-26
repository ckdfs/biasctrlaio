# Tasks: Phase 2 — Peripheral Drivers, Two-Axis Abstraction & Split-Board Integration

- [x] p2_hw_iface Define two-axis board abstraction (`analog_board.h`, `eeprom_schema.h`), Zephyr module, and enable `spi1`/`spi2`/`gpdma1` in `biasctrl_h503`
  owned: include/bias/, zephyr/, dts/bindings/, boards/xidian/biasctrl_h503/
  depends: p1_hw_verify
  verify: test -f include/bias/analog_board.h && test -f include/bias/eeprom_schema.h && test -f zephyr/module.yml

- [x] p2_drv_pilot_mux Implement 8-channel `TS5A23159` pilot multiplexer matrix driver and host unit tests
  owned: drivers/pilot_mux/, tests/host/pilot_mux/
  depends: p2_hw_iface
  verify: test -f drivers/pilot_mux/CMakeLists.txt

- [x] p2_drv_dac8568 Implement `DAC8568` Zephyr driver (6 DC + 2 Pilot channels, internal 2.5 V ref, DMA TX + smoke test)
  owned: drivers/dac8568/
  depends: p2_hw_iface
  verify: test -f drivers/dac8568/CMakeLists.txt

- [x] p2_drv_ads131m02 Implement `ADS131M02` Zephyr driver (`>=5 ms` X2 wait, `>=250 us` reset, ID `0x22`, 64 kSPS HR, fast DMA path)
  owned: drivers/ads131m02/
  depends: p2_hw_iface
  verify: test -f drivers/ads131m02/CMakeLists.txt

- [x] p2_drv_eeprom_oled Implement `P24C02A` EEPROM descriptor store, `<= 1 KiB` `OLED1` driver, and `20 ms` `KEY1-3` debounce helper
  owned: drivers/eeprom_board/, drivers/oled_lite/
  depends: p2_hw_iface
  verify: test -f drivers/eeprom_board/CMakeLists.txt && test -f drivers/oled_lite/CMakeLists.txt

- [x] p2_board_integration_app Implement `20261110-MZM` `analog_board_ops_t` backend and Phase 2 diagnostic CLI app under RAM/Flash budget
  owned: drivers/analog_board/, app/
  depends: p2_drv_pilot_mux, p2_drv_dac8568, p2_drv_ads131m02, p2_drv_eeprom_oled
  verify: bash -c 'source scripts/env.sh && west build -b biasctrl_h503 -d build/h503_p2 app --pristine && python3 scripts/check_mem_budget.py --elf build/h503_p2/zephyr/zephyr.elf'

- [x] p2_hw_verify Flash and verify split-board peripherals (`ADS131M02`, `DAC8568`, `TS5A23159`, `P24C02A`, `OLED1`, debounced `KEY1-3`) on hardware (human-gated)
  owned: docs/bringup/
  depends: p2_board_integration_app
  verify: bash -c 'source scripts/env.sh && west flash -d build/h503_p2'
