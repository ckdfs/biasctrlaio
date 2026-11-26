# Proposal: Phase 5 — Switchless Single-MZM Seamless Switching, DPMZM Fixed Dual-Pilot Control & Local UI

## Why
The reworked prototype hardware (`20261110-底板` with `LMR51430 VSYS = 5.339 V` direct to `+5V`; `20261110-MZM` with `U7` bypassed and all four `TS5A23159` switches `U14/U16/U17/U20` removed and fixed pilot jumpers `DITHER1 -> VA (C48.1)`, `DITHER2 -> VC (C56.1)`, `VB` DC-only) eliminates analog-switch fault paths and establishes a stable split-board platform for single-MZM and 3-arm DPMZM (`CN1: VA/VB/VC`) operation. Phase 5 adds runtime arbitrary bias-point seamless switching for single MZM, fixed dual-pilot 3-arm closed-loop control for DPMZM, and standalone local `OLED1`/`WS-001CB` control while keeping `64 kSPS` real-time timing deterministic and staying within `STM32H503CBT6` memory budgets.

## What Changes
- Implement runtime arbitrary bias-point seamless switching (`QUAD <-> MIN <-> MAX <-> CUSTOM`) for single MZM (`VA`) and switchless 3-arm DPMZM fixed dual-pilot closed-loop control (`ctrl_mod_dpmzm` over `CN1: VA/VB/VC`) in `lib/control/`, validated by host optical simulations under `tests/host/sim/`.
- Implement `app/src/ui_service.{c,h}` (`OLED1` `1 KiB` status/channel/target screens + `20 ms`-debounced `WS-001CB` dial menu + `LED1` state patterns) at lowest thread priority.
- Integrate seamless switching, switchless DPMZM fixed dual-pilot support, and local UI into `app/`, pass `STM32H503CBT6` RAM/Flash budgets (`<= 24576 B` / `<= 110592 B`), complete hardware verification, and synchronize `docs/architecture.md`, `docs/design/algorithm.md`, `docs/user-guide.md`, and `README.md`.

## Out of Scope
- Legacy single-board hardware compatibility.
- 6-channel `VA–VF` `TS5A23159` analog-switch time-division multiplexing (excluded by the reworked switchless `20261110-MZM` prototype).
