# Tasks: Phase 1 — STM32H503CBT6 Digital Baseboard Bring-Up

- [ ] p1_board_def Create Zephyr out-of-tree board definition for STM32H503CBT6 (`biasctrl_h503`)
  owned: boards/xidian/biasctrl_h503/
  depends:
  verify: test -f boards/xidian/biasctrl_h503/biasctrl_h503.dts && test -f boards/xidian/biasctrl_h503/board.yml

- [ ] p1_bringup_app Implement minimal Bring-Up Zephyr app (`USART1` banner, `LED1` blink, `KEY1-3` input, `I2C1` scan, `FPC2` safe idle)
  owned: app/
  depends: p1_board_def
  verify: bash -c 'source scripts/env.sh && west build -b biasctrl_h503 -d build/h503_bringup app --pristine && python3 scripts/check_mem_budget.py --elf build/h503_bringup/zephyr/zephyr.elf'

- [ ] p1_hw_verify Flash and verify Bring-Up firmware on connected STM32H503CBT6 hardware (human-gated)
  owned: docs/bringup/
  depends: p1_bringup_app
  verify: bash -c 'source scripts/env.sh && west flash -d build/h503_bringup'
