# Proposal: Phase 3 — Portable C11 DSP & Control Algorithm Library

## Why
The core Goertzel demodulation, harmonic phase observer, anti-windup PI loop, and modulator strategy patterns verified in `~/code/biascontrol_h523` must be extracted into a hardware-independent C11 library (`lib/dsp/` and `lib/control/`). Because `STM32H503CBT6` has `32 KiB` SRAM (vs `272 KiB` on H523), pass-2 calibration must also be refactored from `8.8 KiB` static scan arrays into an `O(1)`-RAM online normal-equation least-squares solver (`< 1 KiB`), verified on host with a physical MZM optical simulator.

## What Changes
- Port and extend `lib/dsp/` (`dsp_types.h`, `dsp_goertzel`, dual-channel `dsp_pilot_gen`, `dsp_online_lsq` `O(1)`-RAM affine solver).
- Port and decouple `lib/control/` (`ctrl_pid`, `ctrl_observer`, `ctrl_modulator.h` strategy vtable, `ctrl_mod_mzm`, `ctrl_bias_engine`).
- Build a host CMake/CTest unit and physical closed-loop simulation test suite under `tests/host/`.

## Out of Scope
- Zephyr threads, SPI/I2C drivers, and physical hardware flashing (integrated in Phase 4).
