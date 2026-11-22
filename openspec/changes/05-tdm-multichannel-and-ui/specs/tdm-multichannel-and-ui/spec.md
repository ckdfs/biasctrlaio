## ADDED Requirements

### Requirement: 6-DC / 2-Pilot TDM Multi-Channel Bias Control & Seamless Target Switching
`lib/control/` and `ctrl_thread` SHALL schedule 6 DC bias channels (`VA..VF`) and 2 gated pilot sources (`DITHER1`, `DITHER2`) with guard intervals across `TS5A23159` switches and SHALL execute arbitrary target-point switching without cross-period jump.

#### Scenario: Host 6-Channel TDM Closed-Loop Convergence
- **WHEN** `tests/host/sim` runs the 6-channel TDM optical plant simulation (`DPMZM`/`DPQPSK`) under thermal drift and arbitrary target switching
- **THEN** all active bias channels settle within `< 1.0 deg` steady-state phase error without switching-induced branch slips.

### Requirement: Non-Intrusive OLED1 & Debounced Dial Local UI
`ui_service` SHALL display controller state, active channel, lock status, and phase/bias readout on `OLED1` (`<= 1 KiB` framebuffer) and accept `20 ms`-debounced `WS-001CB` dial inputs without increasing `64 kSPS` `DRDY#` drop count above zero.

#### Scenario: Full System Budget & Zero-Drop UI Coexistence
- **WHEN** TDM multi-channel control, CLI telemetry, and `OLED1`/dial UI run concurrently on `STM32H503CBT6`
- **THEN** static RAM remains `<= 24576 B`, Flash remains `<= 110592 B`, and `rt_pipeline` maintains `drop_count == 0`.
