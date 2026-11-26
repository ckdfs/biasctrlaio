# Design: Phase 5 — Switchless Single-MZM Seamless Switching, DPMZM Fixed Dual-Pilot Control & Local UI

## 1. Reworked Switchless Hardware Baseline (`20261110-底板` + `20261110-MZM`)
- **Baseboard `+5V` Supply**: `U7` (`TPS7A2050`) removed; `U3` (`LMR51430`) `R9` reworked so `VSYS = 5.339 V` feeds `+5V` directly (within `DAC8568` `5.5 V` limit).
- **Fixed Pilot Jumpers (`TS5A23159` Removed)**: All four `U14/U16/U17/U20` switches are unpopulated; pilot wiring is fixed as:
  - `DAC8568 CH6` (`U10.7 DITHER1`) -> `C48.1` (`VA`, `CN1` DC channel 0)
  - `DAC8568 CH7` (`U10.10 DITHER2`) -> `C56.1` (`VC`, `CN1` DC channel 2)
  - `DAC8568 CH1` (`VB`, `CN1` DC channel 1) stays DC-only
- **GPIO Safety**: All 8 baseboard switch-control GPIOs (`PA0, PA1, PA2, PA3, PA4, PA6, PA15, PB6`) remain configured as `low-speed` outputs held LOW (`0`) on boot and `FAULT`; `analog_board_ops_t` stays binary-compatible.

## 2. Single-MZM Runtime Seamless Target Switching (`VA`)
- Supports changing target bias points (`QUAD <-> MIN <-> MAX <-> CUSTOM <deg>`) while `ctrl_thread` remains in `CTRL_STATE_RUNNING`, without stopping `rt_pipeline` or cutting pilot excitation.
- Preserves branch continuity on the physical interference curve:
  - Tracks current continuous optical phase `phi_now` and updates target anchor/half-fringe clamps `[lo, hi]` around the active fringe copy without cross-period jumps (`|delta phi| < pi`).
  - Re-seeds observer `[obs_x, obs_y]`, target sine/cosine projection, and `sin^2(phi_target)` voltage spring + `obs_dc` state smoothly while preserving anti-windup integrator state.

## 3. Switchless DPMZM 3-Arm Fixed Dual-Pilot Control (`CN1: VA/VB/VC`)
- **Channel Roles**:
  - `VA` (`CH0` DC) + fixed `DITHER1` (`CH6`, frequency `f1`): child MZM-I phase `phi_1`.
  - `VC` (`CH2` DC) + fixed `DITHER2` (`CH7`, frequency `f2`): child MZM-Q phase `phi_2`.
  - `VB` (`CH1` DC, DC-only): parent phase `phi_3` demodulated from single-PD cross-frequency intermodulation components (`f1 + f2`, `f1 - f2`), eliminating analog-switch switching transients and guard intervals.
- **Orthogonal Integer-Cycle Frequencies (`64 kSPS`)**: Choose `f1` and `f2` such that `f1`, `2*f1`, `f2`, `2*f2`, `f1 + f2`, and `f1 - f2` each complete an exact integer number of cycles per coherent window (zero Goertzel spectral leakage).
- **`15.625 us` `DRDY#` ISR Budget**: Dual-pilot `SPI1` updates (`DITHER1`/`DITHER2`) plus `VA/VB/VC` `5 Hz` async DC updates use zero-DMA-IRQ `SPI1` scheduling (e.g., alternating-sample `DITHER1`/`DITHER2` frame emission or overlapped transfer with post-Goertzel DC slot) verified by DWT cycle instrumentation to keep `drop_count == 0` at `64 kSPS`.

## 4. Low-Priority Local UI (`ui_service`)
- Dedicated low-priority Zephyr thread (`<= 1.5 KiB` stack) polling `I2C1`-mutex-protected `oled_lite` (`1 KiB` framebuffer, `<= 5 Hz`) and consuming `20 ms`-debounced `WS-001CB` dial events (`KEY1` Prev/Dec, `KEY2` Next/Inc, `KEY3` Confirm/Start-Stop).
- Provides standalone operation without USB UART: mode/target selection (`QUAD`, `MIN`, `MAX`, `CUSTOM`), start/stop closed loop, trigger `cal bias`, and `save` to `U21`, plus `LED1` status blink patterns (`500 ms` normal / `150 ms` fault).
- Zero blocking primitives shared with `rt_pipeline`; `OLED1` transactions never preempt `DRDY#` ISR timing.
