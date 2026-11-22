# Design: Phase 5 — 6-DC / 2-Pilot TDM Multi-Channel Control & Local UI

## 6-DC + 2-Pilot TDM Architecture (Aligned with `physicalpcb` & `MasterThesisTest`)
- **Dual Pilot Frequencies**: `DITHER1` (`962.5 Hz`, `DAC8568 CH6`) and `DITHER2` (`1762.5 Hz`, `DAC8568 CH7`) at `64 kSPS` sampling with exact guard intervals (`guard_s`) and measurement windows (`window_s`) matching `pilot_mux_single_pd_revision.json`.
- **Switch Routing Matrix**:
  - Slot group A: `DITHER1 -> VA (PA0)` & `DITHER2 -> VC (PA3)`
  - Slot group B: `DITHER1 -> VB (PA1)` & `DITHER2 -> VB (PA2)` (or independent `DITHER1 -> VD (PA4)` / `DITHER2 -> VF (PB6)` / `VE (PA6/PA15)`)
  - Guard interval turns off active switches (`REF_SW`) and discards transient Goertzel samples before accumulating the slot measurement window.
- **Arbitrary Bias-Point Seamless Switching**:
  - Supports runtime target switching (`QUAD <-> MIN <-> MAX <-> CUSTOM`) without cross-period jump or integrator windup, reusing the branch-continuous phase tracking verified in `MasterThesisTest`.

## Low-Priority Local UI (`ui_service`)
- Dedicated low-priority Zephyr thread (`1.5 KiB` stack) polling `I2C1`-mutex-protected `oled_lite` at `<= 5 Hz` and consuming 20 ms debounced `KEY1` (Prev/Dec), `KEY2` (Next/Inc), `KEY3` (Confirm/Start-Stop) events.
- Zero blocking calls shared with `rt_pipeline`; `OLED1` writes are non-preemptive wrt `DRDY#` ISR.
