# Tasks: Phase 5 — Switchless Single-MZM Seamless Switching, DPMZM Fixed Dual-Pilot Control & Local UI

- [ ] p5_tdm_scheduler Implement single-MZM runtime seamless target switching, switchless fixed dual-pilot `ctrl_mod_dpmzm` (`VA/VB/VC`), host optical simulations (`< 1 deg` error), and update `docs/architecture.md`, `docs/design/algorithm.md`, and `docs/design/optical.md`
  owned: lib/control/, tests/host/sim/, docs/architecture.md, docs/design/algorithm.md, docs/design/optical.md
  depends: p4_cli_and_ztest
  verify: cmake --build build/host-tests && ctest --test-dir build/host-tests --output-on-failure && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [ ] p5_ui_oled_dial Implement `app/src/ui_service.*` (`20 ms` debounced `WS-001CB` dial menu, `1 KiB` `OLED1` display, `LED1` patterns) at lowest priority and update `docs/user-guide.md` and `docs/architecture.md`
  owned: app/src/ui_service.c, app/src/ui_service.h, app/src/main.c, docs/user-guide.md, docs/architecture.md
  depends: p4_cli_and_ztest
  verify: bash -c 'source scripts/env.sh && west build -b biasctrl_h503 -d build/h503_p5 app --pristine && python3 scripts/check_mem_budget.py --elf build/h503_p5/zephyr/zephyr.elf && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync'

- [ ] p5_full_system_verify Integrate seamless switching, switchless DPMZM fixed dual-pilot control, and local UI into final firmware; pass full host/QEMU tests & RAM/Flash budget gate; verify on hardware (human-gated) and update `docs/` and `README.md`
  owned: app/, docs/, README.md
  depends: p5_tdm_scheduler, p5_ui_oled_dial, p4_hw_closed_loop
  verify: bash -c 'source scripts/env.sh && west build -b biasctrl_h503 -d build/h503_p5 app --pristine && python3 scripts/check_mem_budget.py --elf build/h503_p5/zephyr/zephyr.elf && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync'
