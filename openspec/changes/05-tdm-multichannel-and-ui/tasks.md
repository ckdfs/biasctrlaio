# Tasks: Phase 5 — 6-DC / 2-Pilot TDM Multi-Channel Control & Local UI

- [ ] p5_tdm_scheduler Implement `ctrl_mod_dpmzm`, `ctrl_tdm_scheduler`, arbitrary bias-point switching, and host 6-ch TDM simulation (`< 1 deg` error)
  owned: lib/control/, tests/host/sim/
  depends: p4_cli_and_ztest
  verify: cmake --build build/host-tests && ctest --test-dir build/host-tests --output-on-failure

- [ ] p5_ui_oled_dial Implement `app/src/ui_service.*` (`20 ms` debounced `WS-001CB` dial menu, `1 KiB` `OLED1` display, LED patterns) at lowest priority
  owned: app/src/ui_service.c, app/src/ui_service.h, app/src/main.c
  depends: p4_cli_and_ztest
  verify: bash -c 'source scripts/env.sh && west build -b biasctrl_h503 -d build/h503_p5 app --pristine && python3 scripts/check_mem_budget.py --elf build/h503_p5/zephyr/zephyr.elf'

- [ ] p5_full_system_verify Integrate TDM + UI into final firmware, pass full host tests & RAM/Flash budget gate, and verify 6-ch + OLED/dial on hardware (human-gated)
  owned: app/, docs/bringup/
  depends: p5_tdm_scheduler, p5_ui_oled_dial, p4_hw_closed_loop
  verify: bash -c 'source scripts/env.sh && west build -b biasctrl_h503 -d build/h503_p5 app --pristine && python3 scripts/check_mem_budget.py --elf build/h503_p5/zephyr/zephyr.elf'
