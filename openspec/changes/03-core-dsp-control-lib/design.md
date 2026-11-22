# Design: Phase 3 — Portable C11 DSP & Control Algorithm Library

## Zero-Dependency Pure C11 Boundary
- Files under `lib/dsp/` and `lib/control/` MUST NOT include `<zephyr/...>` or `<stm32...>` headers; they compile identically with host `cc`/`clang` and `arm-zephyr-eabi-gcc`.
- Memory-sensitive design for `STM32H503CBT6`:
  - Replace `s_scan_*` / `s_fit_*` (`~8.8 KiB`) from `biascontrol_h523` with `dsp_online_lsq_t` accumulating normal equations $X^T X$ and $X^T y$ (`3x3` symmetric matrix + `3x2` RHS = `< 120 B`) plus a tiny sliding window for pass-1 `Vpi` minima detection (`< 256 B` total).
  - Support two independent pilot tone generators (`DITHER1`, `DITHER2`) and dual-frequency Goertzel demodulators (`1000 Hz` single-channel and `962.5 Hz` / `1762.5 Hz` TDM modes) with configurable integer-cycle block lengths.

## Preserved Control Invariants
- All 8 invariants from `openspec/specs/control-invariants/spec.md` are enforced by unit tests:
  1. Integer pilot cycles per Goertzel block (`N = 1280` at `64 kSPS / 1 kHz`).
  2. H2 Q-component (`h2_mag * sin(h2_phase)`).
  3. Bessel ratio scaling (`J1(m_now)/J1(m_cal)`, `J2(m_now)/J2(m_cal)`).
  4. Unit-circle normalization of `[obs_x, obs_y]`.
  5. Cold-start observer seeding + `sin^2(phi_target)` voltage spring + `obs_dc` EMA (`alpha = 0.50`) + 5-criteria lock streak (`25` updates).
  6. `CH1` DC strictly monitor-only (never mixed into primary phase error).
