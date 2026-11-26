## Purpose

Defines runtime seamless bias-point switching for single MZM, switchless 3-arm DPMZM fixed dual-pilot closed-loop control, and non-intrusive OLED1/dial local UI on reworked `20261110-MZM` hardware.

## ADDED Requirements

### Requirement: Single-MZM Seamless Target Switching & Switchless DPMZM Fixed Dual-Pilot Control
`lib/control/`, `rt_pipeline`, and `ctrl_thread` SHALL support runtime target switching (`QUAD <-> MIN <-> MAX <-> CUSTOM`) without cross-period branch slip and SHALL control 3-arm DPMZM bias (`CN1: VA/VB/VC`) using fixed dual-pilot tones (`DITHER1 -> VA`, `DITHER2 -> VC`, `VB` DC-only) without analog-switch switching.

#### Scenario: Single-MZM Runtime Seamless Target Switching
- **WHEN** the active target switches dynamically across `QUAD`, `MIN`, `MAX`, and `CUSTOM <deg>` while single-MZM closed-loop control is running
- **THEN** the controller maintains continuous physical branch tracking without integrator windup or cross-period slip and settles within `< 1.0 deg` steady-state phase error.

#### Scenario: Host Switchless DPMZM 3-Arm Fixed Dual-Pilot Convergence
- **WHEN** `tests/host/sim` executes the 3-arm DPMZM (`VA/VB/VC`) optical plant simulation with fixed dual-pilot excitation (`f1`, `f2`, and `f1 +/- f2` parent extraction) under thermal drift
- **THEN** all three bias arms settle within `< 1.0 deg` steady-state phase error without analog-switch guard intervals.

### Requirement: Non-Intrusive OLED1 & Debounced Dial Local UI
`ui_service` SHALL display controller state, active mode/channel, target, lock status, and bias readout on `OLED1` (`<= 1 KiB` framebuffer) and process `20 ms`-debounced `WS-001CB` dial inputs (`KEY1–KEY3`) without increasing `64 kSPS` `DRDY#` drop count above zero.

#### Scenario: Full System Budget & Zero-Drop UI Coexistence
- **WHEN** seamless target switching, switchless dual-pilot real-time control, CLI telemetry, and `OLED1`/dial UI run concurrently on `STM32H503CBT6`
- **THEN** static RAM remains `<= 24576 B`, Flash remains `<= 110592 B`, and `rt_pipeline` maintains `drop_count == 0`.
