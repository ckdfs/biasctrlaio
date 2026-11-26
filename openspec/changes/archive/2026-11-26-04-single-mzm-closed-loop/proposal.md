# Proposal: Phase 4 — Single-MZM 64 kSPS Real-Time Closed-Loop Control & Calibration Persistence

## Why
Combine the Phase 2 split-board peripheral drivers with the Phase 3 DSP/control library inside Zephyr RTOS to achieve deterministic `64 kSPS` real-time sampling/dither synthesis, `5 Hz` closed-loop bias locking on the `20261110-MZM` daughterboard (`VA` DC bias + `DITHER1` over `PA0` switch), two-pass calibration persisted in `U21` (`P24C02A`), and non-blocking serial telemetry/control within `STM32H503CBT6`'s `24 KiB` RAM / `108 KiB` Flash limits.

## What Changes
- Implement `app/src/rt_pipeline.{c,h}` (`DRDY#` -> `SPI2` DMA -> Goertzel + `DITHER1` -> `SPI1` DMA, `1280`-sample `k_msgq` handoff, zero-drop monitor, fast-fail `FAULT` cutoff).
- Implement `app/src/ctrl_thread.{c,h}` and `app/src/cal_service.{c,h}` (`5 Hz` observer + PI loop, two-pass `cal bias` with `dsp_online_lsq`, `U21` EEPROM save/load).
- Implement `app/src/cli_service.{c,h}` (non-blocking UART async CLI compatible with host scan scripts) and mock-backend `ztest` integration tests under `tests/ztest/`.

## Out of Scope
- 6-channel TDM multi-modulator scheduling (`DPMZM`/`DPQPSK`), arbitrary bias-point seamless switching, and local `OLED1`/dial menu UI (Phase 5).
