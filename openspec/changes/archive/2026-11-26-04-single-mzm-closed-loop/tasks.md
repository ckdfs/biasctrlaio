# Tasks: Phase 4 — Single-MZM 64 kSPS Real-Time Closed-Loop Control

- [x] p4_rt_pipeline Implement `64 kSPS` real-time `DRDY#` -> `SPI2` DMA -> Goertzel -> `DITHER1` -> `SPI1` DMA pipeline (`app/src/rt_pipeline.*`) with fast-fail fault cutoff
  owned: app/src/rt_pipeline.c, app/src/rt_pipeline.h
  depends: p2_board_integration_app, p3_mod_mzm_and_sim
  verify: bash -ec 'source scripts/env.sh; west build -b biasctrl_h503 -d build/h503_p4 app --pristine; test -f build/h503_p4/CMakeFiles/app.dir/src/rt_pipeline.c.obj; python3 scripts/check_mem_budget.py --elf build/h503_p4/zephyr/zephyr.elf'

- [x] p4_ctrl_and_cal Implement `5 Hz` `ctrl_thread`, two-pass `O(1)`-RAM `cal bias`, and `U21` `P24C02A` EEPROM calibration persistence (`app/src/ctrl_thread.*`, `app/src/cal_service.*`)
  owned: app/src/ctrl_thread.c, app/src/ctrl_thread.h, app/src/cal_service.c, app/src/cal_service.h
  depends: p4_rt_pipeline
  verify: bash -ec 'source scripts/env.sh; west build -b biasctrl_h503 -d build/h503_p4 app; python3 scripts/check_mem_budget.py --elf build/h503_p4/zephyr/zephyr.elf'

- [x] p4_cli_and_ztest Implement non-blocking `cli_service`, integrate `app/src/main.c`, and add mock-backend Zephyr integration tests (`tests/ztest/`) under RAM/Flash budget
  owned: app/, tests/ztest/
  depends: p4_ctrl_and_cal
  verify: bash -ec 'source scripts/env.sh; export PATH="$ZEPHYR_SDK_INSTALL_DIR/hosttools/usr/bin:$PATH"; west build -b biasctrl_h503 -d build/h503_p4 app --pristine; python3 scripts/check_mem_budget.py --elf build/h503_p4/zephyr/zephyr.elf; west build -b qemu_cortex_m3 -d build/ztest_p4 tests/ztest --pristine -t run'

- [x] p4_hw_closed_loop Flash and verify single-MZM `64 kSPS` closed-loop locking (`QUAD`/`MAX`/`MIN`/`CUSTOM`) and `U21` calibration persistence on hardware (human-gated)
  owned: docs/bringup/
  depends: p4_cli_and_ztest, p2_hw_verify
  verify: bash -ec 'source scripts/env.sh; west flash -d build/h503_p4'
