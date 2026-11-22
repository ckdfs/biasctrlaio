# Proposal: Phase 5 — 6-DC / 2-Pilot TDM Multi-Channel Control, Seamless Switching & Local UI

## Why
Unlock the full hardware capability of the `20261110-MZM` daughterboard (`VA–VF` 6 DC bias channels + `DITHER1`/`DITHER2` dual pilot tones routed across 8 `TS5A23159` switches), integrate the arbitrary bias-point seamless switching algorithm from `MasterThesisTest`, and expose local OLED/dial control (`OLED1` + `WS-001CB`) without degrading `64 kSPS` real-time timing or exceeding `STM32H503CBT6` memory budgets.

## What Changes
- Implement `ctrl_mod_dpmzm.c`, `ctrl_tdm_scheduler.c`, and arbitrary bias-point seamless switching in `lib/control/`, validated by a 6-channel TDM optical closed-loop simulator under `tests/host/sim/`.
- Implement `app/src/ui_service.{c,h}` (`OLED1` `1 KiB` status/channel/target screens + 20 ms debounced `WS-001CB` dial menu + `LED1` state patterns) at lowest thread priority.
- Integrate TDM scheduler + local UI into `app/`, verify full-firmware `STM32H503CBT6` RAM/Flash budgets (`<= 24576 B` / `<= 110592 B`), and complete final system hardware verification.

## Out of Scope
- Legacy single-board hardware support.
