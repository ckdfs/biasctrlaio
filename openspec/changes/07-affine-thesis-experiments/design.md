# Design: Phase 7 — Single-MZM Complex-Phasor Affine Bias Control Thesis Experiments (C1–C6)

## 1. Cross-Domain Opto-Electronic Impact Matrix

| Domain | Design / Doc Target | Impact & Coupling Constraints |
| --- | --- | --- |
| Optical | `docs/design/optical.md` | Formalize explicit coordinate mapping $\phi_{\mathrm{thesis}} = \mathrm{wrap}(\phi_{\mathrm{fw}} + \pi)$ (`MATP=0° <-> MAX=180°`, `MITP=180° <-> MIN=0°`, `QTP-=270°(-90°) <-> QUAD=90°` rising DC slope, `QTP+=+90° <-> fw -90°` falling DC slope). Specify 17-point In-Situ DC PSI model $V_{\mathrm{CH1}}(V) = A + B\cos(\frac{\pi}{V_\pi}(V - V_{\mathrm{lock}}) + \phi_{\mathrm{true,thesis}})$ ($B > 0$), fixed `1550 nm` monitor range (`-0.992..787.601 mV`), `S3+` online tracking, and $J_0(m_{\mathrm{RF}}) \to 0$ protection. |
| Electrical | `docs/design/electrical.md` | Preserves `20261110-底板` + `20261110-MZM` baseline (`VSYS=5.339 V` direct to `+5V`, `DITHER1->VA`, `DITHER2->VC`, `VB` DC-only, `FPC2` strictly `low-speed`). Documents `ADS131M02 CLOCK=0x0322` (`TBM=1` overrides `OSR[2:0]` to `OSR=64`, functionally equivalent to `0x0332` in `ch03.tex`), `VA` low-pass $\tau_{\mathrm{LP}} = 16.32\text{ ms}$ driving DC PSI wait timing (`240 ms` first jump, `140 ms` steps), and nominal `0.10 V/V` vs actual `1 kHz` `0.09425 V/V` (`-18.75°`) pilot scaling (`159.2 mV` command -> `1.592 V` DAC -> `150.0 mV` electrode peak). |
| Mechanical / Bench | `docs/design/mechanical.md` | No mechanical or PCB wiring changes. Adds `RIGOL DG922 Pro CH1` (`192.168.99.115:5025`, `50 Ω` load, `0 V` DC offset) directly to the MZM RF port for C6/C5. |
| Algorithm | `docs/design/algorithm.md` | Upgrades onboard single-MZM calibration and closed-loop observer to Chapter 5 formulation: principal-axis projection `(theta1, theta2)`, `O(1)`-RAM Fitzgibbon ellipse + winding reflection + synchronous `CH1` DC regression, removal of `spring` and `obs_dc` self-subtraction, pure circular phase error, `T_wait` post-write block discard, `r < 0.15` integrator freeze, circle-residual EMA recalibration trigger, and `cal_mzm_record_v2` (`64 B` in `U21`). |
| PCB Errata | `docs/design/pcb-errata.md` | Retains `ERR-01..06` unchanged; records C1 onboard checks vs external-instrument exclusions. |
| Experiments | `docs/experiments/README.md`, `docs/experiments/reports/`, `docs/experiments/data/` | Registers `EXP-007` through `EXP-012` covering C1–C6 and thesis table handoff; stores raw acquisition streams in `.agent-bridge/exp_runs/<timestamp>/` and curated archives in `docs/experiments/data/`. |

## 2. Verified Bench Baseline & Guardrails
- **Target Hardware**: `STM32H503CBT6` (250 MHz, hard static budget `RAM <= 24,576 B`, `Flash <= 110,592 B`), Phase 4 baseline uses `15,814 B` RAM and `80,728 B` Flash (`8,762 B` RAM and `29,864 B` Flash headroom).
- **UART & SWD**: `/dev/cu.usbmodem102` (`115200 8N1`), `pyocd` / `west flash` over CMSIS-DAP.
- **DG922 Pro (`192.168.99.115:5025`)**: Initial query shows `:OUTP1? = 0`, `:OUTP1:LOAD? = 9.9E+37` (High-Z), `:SOUR1:APPL?` with `+0.125 V` offset. All automation code MUST explicitly program `:OUTP1:LOAD 50` and `:SOUR1:VOLT:OFFS 0` and verify SCPI readback before enabling output, and guarantee output off on cleanup/exceptions/signals.
- **Prohibited Actions & Files**:
  - Never read/write system temp directories outside `<project>/.agent-bridge/`.
  - Never auto-rollback Git (`git reset --hard`, `git checkout --`, `git restore`).
  - Never modify `openspec/changes/05-tdm-multichannel-and-ui/`, `boards/xidian/biasctrl_h503/biasctrl_h503.dts`, `drivers/ads131m02/`, or `MasterThesisTest/`.
  - Never run `pytest` tests requiring `numpy`/`scipy`/`matplotlib` under `tests/host/` (CI `host-tests` installs only `pytest>=8`); place Python experiment suite unit tests in `tests/experiments/` and run via `/opt/miniconda3/bin/python3 -m unittest discover -s tests/experiments -v`.

## 3. Phase B.1: Firmware Diagnostic & Telemetry CLI (`t1_fw_exp_cli`)

### 3.1 `rt_pipeline` Extensions (`app/src/rt_pipeline.{c,h}`)
- Extend `rt_pipeline_block_t` to expose signed signed-channel I/Q components and raw integer min/max extremes per `1280`-sample (`20 ms`) block:
  - In `rt_pipeline_ingest()` ISR: update `int32_t ch0_min, ch0_max, ch1_min, ch1_max` via 4 integer comparisons per sample (negligible CPU cycles within `15.625 µs` `DRDY#` budget).
  - In `publish_block()`: compute `(h1_i, h1_q, h2_i, h2_q)` from `goertzel_state_t` (`h->h1_magnitude`, `h->h1_phase`, etc. remain compatible) and convert `ch0_min/max`, `ch1_min/max` to `float` volts (`~320 B` msgq RAM increase).
- Add ISR-level DWT cycle timing statistics (zero sample array allocation):
  - Track `drdy_count`, `uint64_t total_cycles` using wrap-safe `uint32_t dt = now - prev_cycles`, `dt_min_cycles`, `dt_max_cycles` across `>= 10^5` `DRDY#` interrupts for `adc stat <n>`.
- Add burst sample capture buffer for `adc dump <n>`:
  - Static packed 24-bit signed (3 bytes/sample) capture buffer of `1280` samples (`3,840 B` static RAM, exact `20 ms` / `50 Hz` resolution contiguous burst at `64 kSPS`) with `pilot=0`.
  - Total firmware static RAM stays `<= 21.5 KiB` (well below `24,576 B`).
  - Support `1 <= n <= 16384` by capturing `ceil(n / 1280)` contiguous 1280-sample segments sequentially in idle state, printing `DUMP_SEG,idx,count,t_cycles` and `DUMP_DATA,<16 signed int32 counts per line>` (zero sample loss within each `20 ms` burst).
- Export helper to read/verify `ADS131M02` registers via existing `bias_ads131m02_read_reg()` when pipeline is idle (`drivers/ads131m02/` untouched).

### 3.2 `ctrl_thread` & `cli_service` Extensions (`app/src/ctrl_thread.{c,h}`, `app/src/cli_service.c`)
- Introduce `CTRL_STATE_EXP_HOLD` so `meas` and `step` keep `rt_pipeline` streaming continuously and drained by `ctrl_thread`, holding the commanded `VA` bias between synchronous host controller calls without dropping blocks (`drops == 0`) or resetting `VA` to `0 V` between calls; `stop` cleanly exits `CTRL_STATE_EXP_HOLD`, shuts down `rt_pipeline`, and resets `VA` to `0 V`.
- Implement CLI commands:
  1. `hw info`: outputs `SYSCLK=250000000`, `ADS131M02` `ID/STAT/MODE/CLOCK` (`0x0322`, `0x1111`, `ID>>8==0x22`), `U21` descriptor CRC & calibration record status/CRC, and current firmware build tags.
  2. `adc stat <n>` (`1000 <= n <= 1000000`): enters silent/pilot-off acquisition, collects `n` `DRDY` periods, stops cleanly, and outputs `ADC_STAT,n=<n>,total_cyc=<u64>,fs_hz=<f>,dt_min_cyc=<u32>,dt_max_cyc=<u32>,drops=<u32>,crc=<u32>`.
  3. `adc dump <n>` (`1 <= n <= 16384`): captures `n` raw `CH0` samples at `64 kSPS` (`pilot=0`) in `1280`-sample contiguous segments and prints `DUMP_SEG` + `DUMP_DATA` lines.
  4. `meas <bias_mv> <pilot_mv> <wait_blocks> <n_blocks>`: validates `bias_mv` within descriptor bounds `[lo, hi]` and `pilot_mv` within `[0.0, 250.0]` (`0.10 V/V` nominal scaling so `159.2 mV` commands `1.592 V` DAC -> `150.0 mV` electrode peak), transitions to `CTRL_STATE_EXP_HOLD` (or updates held bias/pilot cleanly), discards `wait_blocks` (`0..250`) complete 20 ms blocks after commit, and emits `n_blocks` (`1..2000`) CSV rows:
     `MEAS,seq,t_ms,bias_mv,code,pilot_mv,h1_i,h1_q,h2_i,h2_q,dc_mv,ch0_min_mv,ch0_max_mv,ch1_min_mv,ch1_max_mv,drops,crc`
     (formatted compactly `<= 135 B/line`, `< 12 ms` transmission at `115200 bps` < `20 ms` block period).
  5. `step <v0_mv> <v1_mv> <pilot_mv> <n_blocks>`: commits `v0_mv` and settles `15` blocks (`300 ms` > `18 * tau_LP`), commits `v1_mv` immediately on block boundary, and streams the immediately following `n_blocks` (`1..200`) consecutive 20 ms blocks with zero discard in identical CSV format (`STEP,...`).
  6. `telemetry on|off`: enables/disables per-update closed-loop CSV output in `RUNNING`:
     `TEL,k,t_ms,bias_mv,phi_hat_rad,err_rad,radius,h1_i,h1_q,h2_i,h2_q,dc_mv,locked,frozen,recal_count,drops,crc`.

## 4. Phase B.2: Onboard Thesis Chapter 5 Affine Calibration & Closed-Loop Upgrade (`t2_fw_thesis_affine`)

### 4.1 `O(1)`-RAM Principal-Axis Projection + Fitzgibbon Ellipse + Synchronous `CH1` DC Regression
- **Pass 1 (`lib/dsp/`, `lib/control/ctrl_bias_engine.{c,h}`)**:
  - Accumulate `2×2` real symmetric covariance matrices for complex `(Re(H1), Im(H1))` and `(Re(H2), Im(H2))` (`12` doubles total).
  - Solve closed-form 2×2 principal eigenvector angles `(h1_ref_phase, h2_ref_phase)`, enforcing consistent projection sign.
  - Track `Vpi` via `dsp_vpi_tracker_t` and record `CH1` DC min/max (`dc_span`).
- **Pass 2 (`lib/dsp/dsp_online_lsq.{c,h}`, `lib/control/ctrl_bias_engine.{c,h}`)**:
  - Project complex harmonics onto `(h1_ref_phase, h2_ref_phase)` to obtain signed `(s1, s2)` scalar observations `(X = s2, Y = s1)`.
  - Accumulate `O(1)`-RAM `6×6` Fitzgibbon direct conic scatter matrix (`21` lower-triangle doubles) on monomial `[X^2, X Y, Y^2, X, Y, 1]`, plus signed shoelace winding area $\sum (X_{k-1} Y_k - X_k Y_{k-1})$ along increasing `VA`, plus `3×3` linear least-squares of `CH1` DC (`dc_power`) against `[1, X, Y]`.
  - Solve generalized conic eigenproblem `S a = lambda C a` (`6×6`, `O(1)` RAM) for `(o, M^{1/2})` and choose reflection matrix `F = diag(1, +/-1)` so winding matches physical phase direction.
  - When `dc_span > 1 mV`, convert `[1, X, Y]` DC regression algebraically via `(F M^{1/2})^{-T}` to `(C_c, C_s)` in unit-circle coordinates $\chi$, determining canonical rotation $\hat{\phi}_c$ and true `null_v` (maximum `CH1` at thesis `0°` / minimum `CH1` at firmware dark-null `0°`, aligned to `0..2*Vpi` nearest pass-1 `null_v`). When `dc_span <= 1 mV` (legacy zero-DC unit tests), fall back cleanly to `dsp_online_lsq` phase regression so existing unit tests remain valid.
  - Enforce `_Static_assert(sizeof(ctrl_bias_scan_t) < 1024)` (actual ~350–450 B).

### 4.2 Observer, Integrator, `T_wait`, Freeze Protection & Recalibration Trigger
- **`ctrl_observer.{c,h}` & `ctrl_mod_mzm.{c,h}`**:
  - Remove `spring` voltage penalty and `obs_dc` self-subtraction from `ctrl_observer_update()`, preventing thermal bias drift from creating artificial static phase error.
  - Compute exact signed circular phase error `error = remainder(target_rad - inst_rad, DSP_TAU)` (target in firmware coordinates, `phi_hat` tracked continuously).
  - Do NOT clear lock solely because `|bias_v - target_v| > 0.30 * Vpi` (thermal drift can legitimately move bias past `0.30 * Vpi`); enforce actuator clamping only at `[min_v, max_v]`.
  - Normalize `r = hypot(x, y)`. When `r < 0.15f` (`J_0(m_RF)` null proximity or optical fade), set `frozen = true`, return `DSP_INCOMPLETE`, and instruct `ctrl_thread` to hold previous `bias_v` and integrator state WITHOUT transitioning to `CTRL_STATE_FAULT` (avoiding `180°` anti-phase relock across $J_0$ sign reversal).
  - Track normalized circle residual EMA `rho_ema = (1 - alpha) * rho_ema + alpha * fabsf(r - 1.0f)`. When `recal_auto` is enabled and `rho_ema > rho_thresh` for `hold_ticks` consecutive valid samples (`r >= 0.15f`), trigger fast autonomous recalibration (`ctrl_thread`) and re-lock at the prior target (`recal_count++`), logging timestamps for C5 recovery timing.
- **`ctrl_thread.{c,h}` & `cal_service.{c,h}`**:
  - Upgrade EEPROM calibration record to `cal_mzm_record_v2` (`magic = 0x324D5A4D "MZM2"`, `64 B` at `U21` `0xA0..0xDF` adding `h1_ref_phase, h2_ref_phase`, CRC-32 bound to descriptor CRC; maintain explicit fail-fast rejection of mismatched/corrupt records).
  - In `RUNNING`: after each bias write, discard `wait_blocks` (default `3` = `60 ms`, configurable `0..25`), then average `avg_blocks` (`1..16`, default `1`) valid 20 ms blocks before next update; allow configuring `pilot_mv` (default `159.2 mV`), dimensionless gain `G` (converted via `Vpi / pi`), `recal_auto`, `rho_thresh`, `hold_ticks` via CLI `ctrl cfg ...`.

## 5. Phase C: Python Host Automation, In-Situ DC PSI & Benchmark Suite (`t3_host_exp_suite`)
- **Location & Runtime**: Scripts under `scripts/experiments/`, tests under `tests/experiments/`, run with `/opt/miniconda3/bin/python3` (`numpy`, `scipy`, `matplotlib`, `pyserial`). Never use `/tmp`; raw logs under `.agent-bridge/exp_runs/<timestamp>/`, curated archives under `docs/experiments/data/`.
- **Core Modules**:
  1. `uart_client.py`: Deterministic serial transport for `/dev/cu.usbmodem102` (`115200 8N1`), line framing, strict command error checking (fast-fail on negative `err=`, timeout, or `drops > 0` / `crc > 0`).
  2. `dg922_client.py`: Hardened TCP/SCPI client (`192.168.99.115:5025`) with mandatory `:OUTP1:LOAD 50` + `:SOUR1:VOLT:OFFS 0` verification, exact 50 Ω `dBm -> Vpp` conversion (`Vpp = 2 * sqrt(2 * 50 * 10**((dBm - 30)/10))`, hard safety ceiling `Vpp <= 10.0`), and `atexit`/`SIGINT`/`SIGTERM` guaranteed RF output shutoff.
  3. `dc_psi.py`: **In-Situ 17-Point DC PSI** reference solver:
     - Input: `V_lock`, estimated `V_pi`, UART client.
     - Sequence: measure pre-scan `V_lock` (`pilot=0`, 3 blocks) -> `V_start = clamp(V_lock - V_pi, -9.8, 9.8 - 2*V_pi)` -> jump to `V_start` waiting `12` blocks (`240 ms`) -> 17 equidistant voltages `[V_start, V_start + 2*V_pi]` with `7`-block (`140 ms`) inter-step waits and `3` DC blocks/point -> return to `V_lock` waiting `12` blocks (`240 ms`) for post-scan drift check.
     - Fit: 4-parameter bounded nonlinear LSQ (`scipy.optimize.least_squares`) for $V_{\mathrm{CH1}}(V) = A + B\cos\!\bigl(\frac{\pi}{V_\pi}(V - V_{\mathrm{lock}}) + \phi_{\mathrm{true,thesis}}\bigr)$ with $B > 0, V_\pi > 0$; export `phi_true_thesis`, `phi_true_fw = wrap(phi_true_thesis - pi)`, `V_pi`, `A`, `B`, `fit_rmse_mv`, `sigma_phi_deg`, and `drift_mv`.
  4. `algorithms.py`: Synchronous `meas`-driven controllers executing on identical optical path, pilot (`159.2 mV` = `0.15 V` electrode), `T_wait`, and `20 ms` standard window:
     - **B1** (single-harmonic $H_1$ matching);
     - **B2** ($H_1/\bar{i}$ ratio);
     - **B3** (unsigned $S_1/S_2$ ratio);
     - **B4** (diagonal normalized `atan2`);
     - **Affine Frozen** (session-initial `(o, A)` held fixed);
     - **Affine Recal** (per-state recalibrated `(o, A)`);
     - **Affine `S3+`** (online strong-harmonic reference phase update + `±20 mrad` square-wave axis-ratio probe `(1+delta)^2 = -u_x du_x / (u_y du_y)` with `|u_x|<0.22` or `|u_y|<0.22` principal-axis rejection + `||u||<0.15` integrator freeze).
  5. `metrics.py` & `run_c1_c6.py` & `analyze_and_plot.py`: Implements circular mean, static RMS across targets, temporal RMS, short-term $\sigma$, `P95`, two-period half-amplitude $A_2 = |\bar{e}_{\mathrm{odd}} - \bar{e}_{\mathrm{even}}|/2$, $\kappa(A)$, $\sigma_{\min}(A)$, settling times (`1°` and `20 mrad` sustained for 10 steps), and full failure-denominator preservation; renders publication figures and Markdown/CSV tables.

## 6. Phase D: Staged Physical Verification Sequence (`t4`–`t9`)
- **`t4_exp_c1_c2` (C1 onboard + full C2)**:
  - Flash verified Phase 7 ELF via `west flash -d build/h503_p7` (after explicit user confirmation).
  - C1 onboard: 3 cold boots (`hw info`), `ID/STAT/MODE/CLOCK=0x0322` readback, `>= 10^5` `DRDY` cycle rate & zero-drop check, `U21` EEPROM reload check, dark `CH0` `adc dump 8192` `0–32 kHz` FFT & `192 kHz` alias spur audit.
  - C2 (all 4 items): quadrature `±0.1/0.3/0.5 V` × 30 steps `CH0` peak/clipping margin (`0%` saturation); `0–300 ms` `T_wait` scan (identify minimum $T_{\mathrm{wait}}$ for first-window error `<= 0.2°`); full-cycle `CH1` DC max vs `1.2 V` ADC margin; `0.15 V` and `0.08 V` pilot `16 blocks × 15 pairs` (`0.32 s`, `240` pairs) open-loop repeatability and $\kappa$.
- **`t5_exp_c3` (C3 bidirectional calibration)**: 3 sessions × 181-point forward/reverse scans (3 blocks/point/dir); in-fit median error, 5-fold blocked CV (labeled / ellipse+DC / diagonal), bidirectional origin difference, $\kappa$ & $\sigma_{\min}$, 10 reference-restart consistency trials, 512-code DAC INL spur check, and cross-session frozen retest (`P95 <= 1°`).
- **`t6_exp_c4` (C4 16-target 5-method closed-loop)**: Gain pre-check (`4 pts × 5 gains G in {0.05, 0.10, 0.15, 0.20, 0.30}` measuring $A_2$), then 16 equidistant + peak/null/quad dense targets × `±1 rad` seeds × 3 sessions × `>= 60` steps (last `20` steps) across B1, B2, B3, B4, and Affine, validated by In-Situ DC PSI plus onboard autonomous firmware spot-check.
- **`t7_exp_c6` (C6 `20/25 MHz` RF loading & $J_0$ zero crossing)**: After explicit user confirmation to enable `DG922 Pro` RF output, baseline cal with RF Off, sweep `Off / 0 / 3 / 6 / 9 / 13 dBm` (return to `Off` for hysteresis) × 8 targets × 3 sessions comparing B2, B3, B4, Frozen, Recal, and `S3+` via In-Situ DC PSI; then sweep RF power upward through the first $J_0(m_{\mathrm{RF}})=0$ null to verify `r < 0.15` freeze prevents `180°` mis-lock.
- **`t8_exp_c5` (C5 long-term stability & recovery)**: Onboard autonomous loop at `90°` and `45°` (`3 × 3 h` each) with continuous telemetry + periodic 3 s In-Situ DC PSI anchors (and brief RF zero-bias check at `90°`); 5-target × 10-trial forward/reverse step settling (`1°` and `20 mrad` for 10 consecutive updates); 5-trial controlled disturbance recalibration recovery timing.
- **`t9_report_and_handoff`**: Consolidate `EXP-007..EXP-012` into `docs/experiments/reports/` and `docs/experiments/data/`, register in `docs/experiments/README.md`, pass `--strict-doc-sync`, and emit explicit numerical mapping to `MasterThesisTest` `ch03.tex` Table 3.1 and `ch05.tex` C3 / Tables 5.5–5.7.
