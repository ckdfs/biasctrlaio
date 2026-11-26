# core-dsp-control-lib Specification

## Purpose

Provides the hardware-independent C11 DSP harmonic extraction, O(1)-RAM online least-squares calibration solver, and physical MZM closed-loop control library.

## Requirements

### Requirement: Hardware-Independent DSP & O(1)-RAM Calibration
`lib/dsp/` SHALL provide zero-leakage Goertzel harmonic extraction, dual-channel pilot synthesis, and an `O(1)`-RAM online least-squares affine solver (`< 1 KiB` total state) compiling cleanly on both host and `arm-zephyr-eabi`.

#### Scenario: O(1)-RAM Affine Fit Equivalence
- **WHEN** synthetic pass-2 MZM harmonic samples (`H1s, H2s` with ~82 deg hardware lag and affine cross-coupling) are streamed step-by-step into `dsp_online_lsq`
- **THEN** the recovered `[o, M]` matrix matches batch least squares within `1e-4` relative error while using `< 1 KiB` of RAM.

### Requirement: Modulator Strategy & Physical Closed-Loop Simulation
`lib/control/` SHALL preserve all `control-invariants` under `modulator_strategy_t` and verify convergence across `QUAD`, `MIN`, `MAX`, and `CUSTOM` targets against a physical MZM optical plant model on host.

#### Scenario: Host Closed-Loop Lock Across Targets
- **WHEN** `tests/host/sim` executes closed-loop MZM simulations for `QUAD (90 deg)`, `MAX (180 deg)`, `MIN (0 deg)`, and `CUSTOM (17, 45, 135 deg)`
- **THEN** steady-state optical phase error converges within `1.5 deg` and all 5 lock criteria assert `locked == true` without cross-period slip.
