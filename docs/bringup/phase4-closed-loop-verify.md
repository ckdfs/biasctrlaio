# Phase 4 Single-MZM 64 kSPS Closed-Loop Hardware Verification (`biasctrl_h503` + `20261110-MZM`)

## 1. Firmware Build & Flash (2026-11-26, Polarity-Corrected)

- **ELF**: `build/h503_p4/zephyr/zephyr.elf`
  - `text=79576 B, data=1152 B, bss=14662 B`
  - **Flash**: `80,728 / 110,592 B (73.0%)` — PASS
  - **RAM**: `15,814 / 24,576 B (64.3%)` — PASS
- **SWD Probe**: `Devanarchy DAP42 CMSIS-DAP` (`4D3233200643324B07802400`)
- **Flash Command**: `west flash -d build/h503_p4` (`exit 0`, erased `81,920 B` across `10` sectors, programmed `80,896 B` across `79` pages at `20.27 kB/s`)
- **Regression Suites (Post-Update)**:
  - `-Wall -Wextra -Werror`: 0 warnings across `drivers/analog_board/mzm_20261110.c`, `drivers/dac8568/dac8568.c`, and all 5 `app/src/*.c` files (`main.c`, `rt_pipeline.c`, `cal_service.c`, `ctrl_thread.c`, `cli_service.c`)
  - QEMU `qemu_cortex_m3` `ztest`: `5/5 PASS` (`27.059 s`, `PROJECT EXECUTION SUCCESSFUL`)
  - Host CTest: `13/13 PASS` (`5.26 s`)

## 2. Schematic-Matched `20261110-MZM` Descriptor, `DAC8568` Fix & Pre-Optical Verification

From the `20261110-MZM` schematic netlist (`U10` `DAC8568` `0..5.000 V` internal `2.500 V` reference, `U11B` `OPA4192` subtractor $V_{\mathrm{VA}} = 4(V_{\mathrm{VOA}} - 2.500\ \mathrm{V}) \in [-10.0, +10.0]\ \mathrm{V}$, AC pilot gain $-R_{23}/R_{22} = -12\ \mathrm{k\Omega}/120\ \mathrm{k\Omega} = -0.10\ \mathrm{V/V}$, and `U12A` TIA `R17 = 47 k\Omega`), the default `U21` EEPROM descriptor and electrode-referred pilot scaling were updated and flashed:
- `subtractor_gain_ppm = 8000000`, `subtractor_offset_uv = -10000000`, `tia_gain_ohms = 47000` (`VA = -10.0..+10.0 V`, `0 V` idle `safe_code = 32768`)
- `RT_PIPELINE_PILOT_MAX_V = 0.25f` V peak at `VA` electrode (`SCAN_PILOT_V = 0.13f` V peak)

Aligned `drivers/dac8568/dac8568.c` with `biascontrol_h523/src/drivers/drv_dac8568.c`:
1. Switched channel frame command from `0x00` (`WRITE_REG`) to `0x03` (`CMD_WRITE_UPDATE`, updates DAC output immediately on `SYNC#` rising edge), while widening the belt-and-suspenders `LDAC#` low pulse to `32` NOPs for `low-speed` `PB2`.
2. Updated `startup()` (`POST_KERNEL 90`, using `k_busy_wait`) to execute `RESET` (`0x07000000`) + `2 ms` settle -> `REF_ENABLE` (`0x08000001`) + `5 ms` (`C46 = 10 uF`) settle -> `CH_ALL = 32768` (`0x03F80000`, `0.000 V` bias / `2.500 V` dither), followed by 100 mid-scale smoke writes.
3. Kept `SPI1` at `12.5 MHz` (`SPI1->CFG1 = 0x2007801F`) with `wait_limit = 8 us` in `app/src/rt_pipeline.c`; live SWD readback confirms `tx_word = 0x03080000` at idle (`safe_code = 32768`).

During initial live `64 kSPS` testing on `STM32H503CBT6`, `scan vpi` tripped `rt_fault=-16` (`-EBUSY`) because `GPDMA1` completion IRQ busy-waiting and thread-side `SPI1` DC writes overlapped the next `15.625 us` `DRDY#` edge. Resolved with four changes while preserving all `FPC2` `low-speed` slew settings:
1. **`drivers/dac8568/dac8568.{c,h}`**: Replaced the `GPDMA1` TC ISR busy-wait with zero-DMA-IRQ register polling (`finish_if_eot_locked()`) and added ISR-safe `dac8568_set_dc_code_async()`.
2. **`boards/xidian/biasctrl_h503/biasctrl_h503.dts`**: Increased `bias_dac` `spi-max-frequency` to `12.5 MHz` (matching `bias_adc`, halving 32-bit DAC wire time to `2.56 us`, passing the 100-write boot smoke test).
3. **`app/src/rt_pipeline.c`**: Replaced software-emulated `double` `hypot`/`atan2` in `publish_block()` with hardware single-precision `goertzel_result_fast()`.
4. **`app/src/rt_pipeline.c`**: Made `DRDY#` ISR the sole owner of `SPI1` while streaming—submitting `DITHER1` before Goertzel to overlap wire transfer with DSP math, and scheduling `5 Hz` `VA` DC writes inside the post-Goertzel ISR slot via `dac8568_set_dc_code_async()`.

### Pre-Optical UART Capture (`/dev/cu.usbmodem102` @ `115200 bps`, `.agent-bridge/p4_hw_preflight_uart.log`)
- **`U21` EEPROM Init & Persistence Check**:
  - `eeprom init` -> `eeprom init: ok crc=0x1E71EE41`
  - `reinit` -> `reinit: uncalibrated (err=-77), run cal bias`
  - Post-`pyocd reset` boot:
    ```text
    biasctrl_h503 Phase 4, SYSCLK 250000000 Hz
    analog_board init (20261110-MZM): 0
    oled_lite init: 0
    keys init: 0
    ctrl_thread init: -77
    cli_service start: 0
    Phase 4 CLI ready. Type 'help'.
    ```
    (`-77` = `-EBADMSG`: valid `20261110-MZM` `±10 V` descriptor with erased Phase 4 affine record, as expected prior to `cal bias`; `status` shows `state=IDLE init=1 ... bias=0 mV`.)
- **CLI Commands**: `help`, `status`, `target min|max|custom 135|quad` passed; `target custom abc` returned `-22` (`-EINVAL`).
- **Live `64 kSPS` `scan vpi` -> `stop` (3.5 s continuous stream with `5 Hz` `VA` steps)**:
  - Mid-scan (`2.5 s`): `state=SCAN ... bias=-9531 mV ... updates=12 pass=1 step=6 ... drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=-77 rt_fault=0` (`-10.000 V + 6/256 * 20.000 V = -9.531 V`)
  - Post-`stop` (`3.5 s`): `state=IDLE ... bias=0 mV ... updates=17 pass=1 step=8 ... drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=-77 rt_fault=0`

## 3. Optical-Path Closed-Loop Verification (2026-11-26)

Hardware rework prior to optical verification: modified baseboard `R9` so `LMR51430` delivers `VSYS = 5.339 V` directly to `+5V` (`U7` removed), and removed the four `TS5A23159` switches (`U14/U16/U17/U20`) with fixed pilot jumpers `U10.7 -> C48.1 (VA)` and `U10.10 -> C56.1 (VC)`.

Polarity fix in `app/src/ctrl_thread.c`: updated `ctrl_bias_scan_begin_fit(&scan, -1)` (`null_q_sign = -1`) to match the `20261110-MZM` `U12A` inverting TIA plus `2 kHz` analog/ADC phase lag, so `MIN(0 deg)` aligns with negative signed `H2` quadrature (`~ -1.708 rad`, optical dark null) and `MAX(180 deg)` aligns with positive signed `H2` quadrature (`~ +1.433 rad`, optical peak).

Full automated UART transcript saved in `.agent-bridge/p4_hw_closed_loop_uart.log` (`OPTICAL_VERIFY_PASS`, `/dev/cu.usbmodem102 @ 115200 bps`):

- [x] Connect external optical path (`Laser -> MZM -> Photodetector -> 20261110-MZM` AC/DC inputs, `VA` bias + `DITHER1` output to MZM)
- [x] Confirm `OLED1` (`BIASCTRL H503 P4`) and `LED1` (`500 ms` normal / `150 ms` fault) behavior visually (user-confirmed)
- [x] Run two-pass calibration (`cal bias`) and verify `Vpi`, `null`, and affine fit convergence:
  - Sweep DC span: `-0.992 mV` (dark null) .. `787.601 mV` (optical peak); peak quadrature `h1 ≈ 355.4 mV`, peak `h2 ≈ 5.72 mV`.
  - Converged calibration: **`Vpi = 5382 mV`**, **`null = -3029 mV`**, `updates = 1028`, `pass = 2`, `step = 256`, `drops = 0`, `crc_errs = 0`, `fault = 0`, `rt_fault = 0`.
- [x] Persist calibration (`save`), power-cycle/reset controller, and verify automatic `U21` reload (`ctrl_thread init: 0`):
  - `save` -> `save: ok`; post-`pyocd reset` boot printed `ctrl_thread init: 0`; `status` showed `state=IDLE init=1 cal=1 vpi=5382 mV null=-3029 mV load_err=0 drops=0 crc_errs=0 fault=0 rt_fault=0`.
- [x] Verify closed-loop locking across `QUAD` (`90 deg`), `MAX` (`180 deg`), `MIN` (`0 deg`), and `CUSTOM <deg>` with `drops=0`, `crc_errs=0`, `locked=1`, `hold_assist=1`, and verified optical polarity:

| Target | Lock Time (`locked=1`) | Hold-Assist Time (`hold_assist=1`) | Settled `bias` | `h1` (`mag@phase`) | `h2` (`mag@phase`) | `dc` | Normalized Intensity | Updates | Errors (`drops`/`crc`/`fault`) |
| --- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `QUAD(90)` | `1.0 s` | `5.2 s` | `-340 mV` | `355.394 mV@-2.287 rad` | `0.311 mV@2.447 rad` | `418.561 mV` | `53.2%` | `46` | `0 / 0 / 0` |
| `MAX(180)` | `1.0 s` | `5.2 s` | `2285 mV` | `2.981 mV@0.853 rad` | `5.721 mV@1.433 rad` | `787.601 mV` | `100.0%` | `46` | `0 / 0 / 0` |
| `MIN(0)` | `2.1 s` | `7.3 s` | `-3131 mV` | `7.114 mV@0.857 rad` | `5.720 mV@-1.708 rad` | `-0.992 mV` | `0.0%` | `57` | `0 / 0 / 0` |
| `CUSTOM(135)` | `1.0 s` | `5.2 s` | `1006 mV` | `245.850 mV@-2.286 rad` | `3.991 mV@1.474 rad` | `678.135 mV` | `86.1%` | `46` | `0 / 0 / 0` |
