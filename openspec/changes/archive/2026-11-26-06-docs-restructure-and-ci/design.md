# Design: Phase 6 — Domain Documentation Restructure, PCB Errata & GitHub Actions CI

## 1. Documentation Restructure Specification

### 1.1 Target Directory Tree
```text
docs/
├── README.md
├── architecture.md
├── user-guide.md
├── design/
│   ├── optical.md
│   ├── electrical.md
│   ├── pcb-errata.md
│   └── algorithm.md
└── experiments/
    ├── README.md
    ├── template.md
    ├── bringup/
    │   ├── phase1-h503-bringup.md
    │   ├── phase2-peripherals-verify.md
    │   └── phase4-closed-loop-verify.md
    ├── reports/.gitkeep
    └── data/.gitkeep
```
- Remove obsolete `docs/design.md` and `docs/bringup/` after migrating content.
- Write all `docs/` files in concise, practical engineering Chinese (no internet buzzwords, no "不是……而是……" rhetoric, no padded summaries).
- Every internal link across `README.md`, `AGENTS.md`, `docs/`, and `openspec/` MUST use repository-relative paths (zero local `file` scheme URIs, zero links into `build/` or `.agent-bridge/`).

### 1.2 Authoritative Technical Content by Document

#### `docs/architecture.md`
- Two-axis split-board decoupling (`boards/xidian/biasctrl_h503/` + `analog_board_ops_t` in `include/bias/analog_board.h` + `U21 P24C02A` descriptor in `include/bias/eeprom_schema.h`); legacy single-board hardware excluded.
- Mermaid top-level architecture diagram (`app/` + `lib/` + HAL + `drivers/`).
- Execution context & isolation table:
  - `DRDY#` EXTI + DMA RX (`app/src/rt_pipeline.c`): `64 kSPS` (`15.625 µs` = `3906` CPU cycles @ `250 MHz`), exclusive owner of `SPI1`/`SPI2` while streaming, zero-DMA-IRQ polling (`8 µs` wait limit), `goertzel_result_fast()`, blocks pushed to `rt_pipeline_blocks` (`k_msgq`, depth `10`, `50 Hz`). No I2C/UART/mutex/sleep; fast-fail `trip()` on CRC/SPI/queue error.
  - `ctrl_thread` (`app/src/ctrl_thread.c`): Zephyr priority `5`, stack `2560 B`, `DSP_CONTROL_DECIMATION = 10` (`5 Hz` / `0.2 s`), calibration & PI loop, delegates DC writes via `rt_pipeline_set_dc_code()`.
  - `cli_service` (`app/src/cli_service.c`): priority `8`, stack `2048 B`, `128 B` interrupt-driven ring buffer.
  - `main` (`app/src/main.c`): `500 ms` differential `OLED1` refresh via `bias_i2c1_mutex` + `LED1` heartbeat (`500 ms` normal / `150 ms` fault).
- `U21 (P24C02A, 256 B)` EEPROM map (`include/bias/eeprom_schema.h` + `app/src/cal_service.c`):
  - `0x00..0x93` (`148 B`): `struct bias_eeprom_descriptor_v1` (magic `0x53414942` `"BIAS"`, `schema_version=1`, `board_id="20261110-MZM"`, `tia_gain_ohms=47000`, 6 channels, trailing `CRC-32/ISO-HDLC`).
  - `0x94..0x9F` (`12 B`): page-alignment padding.
  - `0xA0..0xD7` (`56 B`): `struct cal_mzm_record_v1` (magic `0x314D5A4D` `"MZM1"`, `descriptor_crc`, `dsp_affine_fit_t fit`, `null_v`, `vpi_v`, `cal_pilot_v`, `obs_dc_seed`, `has_obs_dc_seed`, trailing `CRC-32/ISO-HDLC`).
  - `0xD8..0xFF` (`40 B`): reserved.
- Phase 4 verified baseline: `text=79,576 B, data=1,152 B, bss=14,662 B`; Flash `80,728 / 110,592 B (73.0%)`, RAM `15,814 / 24,576 B (64.3%)`; QEMU `ztest` `5/5 PASS`, Host CTest `13/13 PASS`.
- Phase 5 roadmap (`05-tdm-multichannel-and-ui`): `p5_tdm_scheduler` (single-MZM seamless switching + switchless DPMZM fixed dual-pilot), `p5_ui_oled_dial` (`ui_service.{c,h}` `<= 1.5 KiB` stack), `p5_full_system_verify`.

#### `docs/design/optical.md`
- Single-MZM dual-arm interference model:
  - $E_{\mathrm{out}}(t) = E_{\mathrm{in}}(t)\bigl[a_1 e^{j\varphi_1(t)} + a_2 e^{j\varphi_2(t)}\bigr]$, $P_L = 2 P_{\mathrm{in}}(a_1^2 + a_2^2)$, fringe visibility $\nu = \frac{2 a_1 a_2}{a_1^2 + a_2^2} \in [0, 1]$, extinction ratio $\mathrm{ER} = \frac{P_{\max}}{P_{\min}} = \frac{1+\nu}{1-\nu}$.
  - Standard cosine form $P_{\mathrm{out}}(t) = \frac{P_L}{2}[1 + \nu\cos\phi(t)]$. Clarify the $\pi$-radian coordinate convention used in firmware (`test_mzm.c` / `ctrl_mod_mzm.c` where $P \propto \frac{1-\cos\phi_{\mathrm{fw}}}{2}$ gives `MIN=0°`, `QUAD=90°`, `MAX=180°`) versus the `+cos` academic convention (`MATP=0°`, `QTP+=90°`, `MITP=180°`): $\phi_{\mathrm{fw}} = \phi + \pi$.
  - Phase decomposition $\phi(t) = \phi_b(t) + \xi(t) + \phi_{\mathrm{RF}}(t)$, $\phi_b(t) = \frac{\pi V_b(t)}{V_{\pi,\mathrm{DC}}} + \phi_0(t)$; distinguish $V_{\pi,\mathrm{DC}}$, pilot $V_\pi(f_d)$, and traveling-wave $V_{\pi,\mathrm{RF}}(f_{\mathrm{RF}})$; linear sensitivity $\delta\phi = \frac{\pi}{V_{\pi,\mathrm{DC}}}\delta V_b$; physical drift sources ($\phi_0(t)$ interface charge/thermal-optic drift vs slow $V_\pi$ shift).
- 3-arm DPMZM (optical I/Q modulator) model for Phase 5:
  - Push-pull finite-ER sub-MZM field $h(\phi) = \cos(\phi/2) + \delta e^{j\phi/2}$, nested field $E_{\mathrm{DPMZM}} = [(1+\delta_P)h_I + h_Q e^{j\phi_P}] E_{\mathrm{in}}/2$.
  - Single-PD total intensity detection on fixed dual-pilot topology (`VA` child-I modulated by `DITHER1` at $f_1$, `VC` child-Q modulated by `DITHER2` at $f_2$, `VB` parent DC-only): child-I from $f_1, 2f_1$, child-Q from $f_2, 2f_2$, parent phase $\phi_P$ from cross-frequency intermodulation $f_1 \pm f_2$.
  - Note push-pull vs single-arm common-phase shift $\chi_Q - \chi_I$ (`h = e^{j\chi}[\cos(\phi/2)+\delta e^{j\phi/2}]`) and manual polarization controller (`PC`) / `PBS` role (optional spectrum observation only; never a closed-loop polarization selector for single PD).
- Optical power & photodetector link budget:
  - 1550 nm DFB laser -> MZM -> 1:9 optical coupler (10% monitor tap) -> InGaAs PIN PD (typical $\mathcal{R} = 0.90\ \mathrm{A/W}$, dark current `0.55 nA`, $C_{\mathrm{PD}} = 5\ \mathrm{pF}$).
  - Nominal `-9 dBm` (`125.89 µW`) average monitor power -> $I_q = I_0 = 113.30\ \mu\mathrm{A}$, $I_{\max} \le 226.60\ \mu\mathrm{A}$, CH1 voltage across $R_{16}=3.3\ \mathrm{k\Omega}$ = `0.748 V` (`0.737 V` with ADC input load).
  - High-power ceiling $I_{\max} = 310\ \mu\mathrm{A}$ (`344.4 µW` peak) -> CH1 `1.023 V` (`85.2%` of `±1.2 V` ADC FSR, below `90%` limit).
- Operating point table (`MIN`, `QUAD`, `MAX`, `CUSTOM`), measured polarity contract (`null_q_sign = -1` verified on `20261110-MZM` where `MIN(0°)` has `H2` phase `~ -1.708 rad` and `MAX(180°)` has `~ +1.433 rad`), and Mermaid optical testbench diagram.

#### `docs/design/electrical.md`
- Split-board mechanics & PCB stackup:
  - Both boards `47.20 × 30.00 × 1.60 mm` (`R1.0` corners), `6.00 mm` board-to-board gap, `30 mm` Type-A 30-pin `0.5 mm` FFC (`KH-CL0.5-H2.0-30PIN`).
  - `20261110-底板`: 2-layer FR-4, `USB_P/N` `90.0 Ω` CPWG (`w=0.300 mm, s=0.154 mm, g=0.200 mm`, lengths `9.570 / 9.622 mm`).
  - `20261110-MZM`: 4-layer `JLC04161H-3313` (`1 oz` outer, `0.5 oz` inner, `0.100 mm` `3313` prepreg $\varepsilon_r = 4.1$): `Layer 1` top signal (`w=0.148 mm, g=0.200 mm` -> `50.0 Ω` single-ended, TIA summing node `$1N84` `w=0.127 mm` full-stack keepout); `Layer 15` solid zero-trace `GND`; `Layer 16` power trunks + 8 switch traces; `Layer 2` `GND` shield + `FPC2` + `TP1–TP13`.
- Complete 30-pin `FPC2` pin mapping table (baseboard pin 1..30 <-> MZM pin 30..1, `STM32H503CBT6` pin, `RN7–RN9` `33 Ω`, `RN10–RN11` `100 Ω`, `RN12–RN13` `4.7 kΩ` pull-downs, `RN14`/`R84` `10 kΩ` pull-ups, and mandatory `slew-rate = "low-speed"` rule in `boards/xidian/biasctrl_h503/biasctrl_h503.dts`).
- Power tree, IPC-2152 thermal/drop budget, and test-point coordinates (`底板 TP1–TP5`, `MZM TP1–TP13`):
  - External `±15 V` -> `U1 (TPS7A4901)` `+12 V`, `U2 (TPS7A3001)` `-12 V`, `U3 (LMR51430XF, 500 kHz FPWM)` `VSYS` (reworked via `R9` to `5.339 V` direct to `+5V` after removing MZM `U7`), baseboard `U5 (TPS7A2033)` `VDD33D`, MZM local `U5/U6 (TPS7A2033)` `VDD33D/VDD33A`.
- Analog signal chain quantitative equations:
  - DC bias driver (`DAC8568IDPW` `2.500 V` internal ref, `0..5.000 V`, `0x03 CMD_WRITE_UPDATE`, boot sequence `1 ms -> RESET(0x07000000) -> 2 ms -> REF_ENABLE(0x08000001) -> 5 ms (C46=10 µF) -> CH_ALL=32768`):
    - Subtractor (`U11` `OPA4192ID`): $V_b = 4(V_{\mathrm{DAC}} - 2.500\ \mathrm{V}) \in [-10.000, +10.000]\ \mathrm{V}$ (`subtractor_gain_ppm = 8000000`, `subtractor_offset_uv = -10000000`), step `305.2 µV/LSB` (`~0.182 mrad/LSB` at `Vpi=5.28 V`).
    - Non-inverting LPF: $R_{\mathrm{eq}} = 30\ \mathrm{k\Omega} \parallel 120\ \mathrm{k\Omega} = 24\ \mathrm{k\Omega}$, $C_{\mathrm{LP}} = 0.68\ \mu\mathrm{F}$ (`C51/C55/...`), $\tau_{\mathrm{LP}} = 16.32\ \mathrm{ms}$, $f_c \approx 9.75\ \mathrm{Hz}$; reference T-filter (`1 kΩ / 2 kΩ / 22 µF`, `~10.9 Hz`), electrode `1 kHz` noise density `~56 nV/√Hz`.
  - Independent AC pilot injection (`DAC8568 CH6 DITHER1 / CH7 DITHER2`):
    - $C_{\mathrm{inj}} = 100\ \mathrm{nF}$ C0G (`C48/C56`) + $R_{\mathrm{inj}} = 120\ \mathrm{k\Omega}$ + feedback $R_f = 12\ \mathrm{k\Omega} \parallel C_f = 4.7\ \mathrm{nF}$ C0G:
      $T_i(j\omega) = -0.1 \frac{j\omega R_{\mathrm{inj}}C_{\mathrm{inj}}}{1+j\omega R_{\mathrm{inj}}C_{\mathrm{inj}}} \frac{1}{1+j\omega R_f C_f}$.
    - At `1 kHz`: $|T_i| = 0.09425$, phase lag `-18.75°` beyond ideal `-180°`; firmware nominal scaling `-R23/R22 = -0.10 V/V` -> `RT_PIPELINE_PILOT_MAX_V = 0.25 V`, `SCAN_PILOT_V = 0.13 V` (`±1.30 V` / `0.919 Vrms` at `U10.7`).
    - Reworked switchless jumpers: `U10.7 -> C48.1 (VA)`, `U10.10 -> C56.1 (VC)`, `VB` DC-only (cross-reference `pcb-errata.md`).
  - Photoreceiver & ADC (`ADS131M02IRUKR`, ID `0x22xx`, `±1.2 V` FSR -> `1.2 / 8388608 V/LSB` ≈ `143 nV/LSB`, 24-bit + CCITT CRC-16, `CLOCK = 0x0322` for `64 kSPS` `TBM` / `0x0302` for `32 kSPS`):
    - DC monitor `CH1`: across `R16 = 3.3 kΩ` (`R15 = 5 kΩ` upper series).
    - AC TIA `CH0`: $C_{37} = 100\ \mathrm{nF}$ C0G ($\tau_{\mathrm{HP}} = 0.83\ \mathrm{ms}, f_{\mathrm{HP}} \approx 191.8\ \mathrm{Hz}$) -> inverting `U12` (`OPA4192ID`, `R17 = 47 kΩ || C39 = 10 pF` C0G) -> split anti-alias `R18 = 820 Ω` + `D1 (BAV199DW-7-F, ≤ 5 nA)` + `R91 = 180 Ω` + `C41 = 10 nF` C0G. Total input-referred noise `605.3 nV/√Hz @ 1 kHz`, `597.9 nV/√Hz @ 2 kHz`.
- Dual-crystal heterodyne clock & `15.625 µs` `DRDY#` ISR timing:
  - Baseboard `X1 = 8.000 MHz` passive (`PLL1 M=2, N=125, P=2 -> 250 MHz SYSCLK`, `PA8 MCO1` disabled) + MZM local `X2 = 8.192 MHz` active CMOS (`±30 ppm`, `3.18 mm` trace via `R92 = 33 Ω`).
  - Alias beat $|8.192 - 8.000| = 192\ \mathrm{kHz} = 3 f_{\mathrm{data}}$ sits at `ADS131M02` `sinc3` 3rd notch (`155.7 dB` rejection within `±60 ppm`).
  - Startup & sync: `>= 6 ms` (`>= 5 ms` spec) `X2` settle + `300 µs` (`>= 250 µs` spec) `SYNC#` low reset pulse.
  - `SPI1`/`SPI2` at `12.5 MHz` (`2.56 µs` 32-bit DAC wire time), zero-DMA-IRQ polling (`wait_limit = 8 µs`), `DRDY#` ISR sole owner of `SPI1`/`SPI2`.

#### `docs/design/pcb-errata.md`
Structured errata table + detailed root-cause / rework / verification / next-rev fix for:
- **ERR-01 (`+5V` LDO `U7` & `LMR51430` `R9`)**: On `20261110-MZM`, `U7 (TPS7A2050PDQNR, X2SON-4)` was removed and bypassed; on `20261110-底板`, `U3 (LMR51430)` feedback resistor `R9` (BOM `88.7 kΩ`) was reworked so `VSYS = 5.339 V` (within `DAC8568` `5.5 V` max rating) feeds `+5V` directly. (Also explicitly correct earlier docs that wrote "底板 U7" — `U7` is on `20261110-MZM`, while `R9` is on `20261110-底板`.) Status: 已飞线规避，下版改板核查 `U7` 封装/散热与焊盘。
- **ERR-02 (模拟开关 `U14/U16/U17/U20` 拆除与固定双导频飞线)**: All four `TS5A23159RSER` (`UQFN-10`) switches on `20261110-MZM` removed; fixed jumpers `U10.7 (DITHER1) -> C48.1 (VA)` and `U10.10 (DITHER2) -> C56.1 (VC)`, `VB` DC-only; 8 baseboard switch GPIOs (`PA0..PB6`) remain `low-speed` LOW. Status: 已改固定双导频（Phase 5 适配无开关方案）。
- **ERR-03 (`VF` 通道 `R56` 错焊)**: `20261110-MZM` `R56` populated as `1 kΩ` instead of BOM `30 kΩ`. Currently unused (`VA/VB/VC` only). Status: 待返修（不影响当前单 MZM / 3 臂 DPMZM）。
- **ERR-04 (主控 MCU 料号差异 `STM32H523CET6` vs `STM32H503CBT6`)**: `20261110-底板` BOM specifies `STM32H523CET6` (`272 KiB` SRAM / `512 KiB` Flash), whereas prototype board is populated with `STM32H503CBT6` (`32 KiB` SRAM / `128 KiB` Flash), imposing hard firmware budgets `RAM <= 24,576 B`, `Flash <= 110,592 B` and driving `O(1)`-RAM calibration. Status: 已在固件端适配。
- **ERR-05 (装配机械间隙与 BOM 核对项)**: `OLED1` rear clearance against `USB1` shell pins (`2.5 mm` spacer + polyimide tape, pin tail `<= 1.50 mm`); `H1` BOM (`HX PZ1.27-1x3P WT`) vs enclosure window height (`PM1.27`, `3.10 mm`) check on assembly. Status: 装配工艺规避 / 下版核对。
- **ERR-06 (`FPC2` 压摆率与 `64 kSPS` `SPI1` 时序冲突)**: `very-high-speed` on `FPC2` causes inductive ringing; original `GPDMA1` TC IRQ wait + thread-side `SPI1` writes triggered `rt_fault=-16 (-EBUSY)` at `64 kSPS`. Resolved in DTS (`low-speed` enforced) + `12.5 MHz` `SPI1` + zero-DMA-IRQ ISR ownership (`dac8568_set_dc_code_async()`). Status: 已通过 DTS 与驱动修复验证（`drops=0`）。

#### `docs/design/algorithm.md`
- Jacobi-Anger harmonic model ($J_1(m), J_2(m)$), dual-inverting chain polarity & phase propagation, weak-axis observability ($\kappa(A) \simeq 4/m$, tangential phase noise $\sigma_\phi^2(\phi_b) = \mathbf{t}^T A^{-1}\Sigma_n A^{-\mathsf{T}}\mathbf{t}$).
- Zero-leakage Goertzel (`N = 1280` @ `64 kSPS`, `f0 = 1 kHz`, `k=20, 40`, `goertzel_result_fast()`, `DSP_CONTROL_DECIMATION = 10` -> `5 Hz` loop).
- `O(1)`-RAM two-pass scan calibration (`lib/dsp/dsp_online_lsq.c`, `lib/control/ctrl_bias_engine.c`, `< 256 B` core LSQ/tracker structs, `< 1 KiB` total scan state):
  - Pass 1 (`257` steps, `0.2 s` settle + `0.2 s` sample): `dsp_vpi_add()` (`< 10%` peak valley detection + quadratic vertex refinement) -> `Vpi`, and `first_valley_q = h2 * sin(h2_phase)` with `null_q_sign = -1` to disambiguate dark null (`MIN(0°)`) vs peak (`MAX(180°)`).
  - Pass 2 (`257` steps): `dsp_online_lsq_add()` accumulates `3×3` symmetric Gram matrix (`gram[6]` + `rhs[3][2]`) over `[1, sin(phi), cos(phi)]` and solves via Cholesky factorization for `o[2]` and `M[2][2]`.
- Harmonic phase observer & anti-windup PI (`ctrl_observer.c`, `ctrl_pid.c`, `ctrl_mod_mzm.c`):
  - Bessel row scaling $J_1(m_{\mathrm{now}})/J_1(m_{\mathrm{cal}})$, $J_2(m_{\mathrm{now}})/J_2(m_{\mathrm{cal}})$, determinant check (`FLT_EPSILON`), `2×2` inversion.
  - Unit-circle normalization `[obs_x, obs_y]` decoupling optical/RF power fluctuations (noting $J_0(m_{\mathrm{RF}})$ zero-crossing boundary).
  - `CH1` monitor-only invariant; $\sin^2(\phi_{\mathrm{target}})$ voltage spring + `obs_dc` fast EMA (`alpha = 0.50`, `DC_WARMUP = 5`); 5 lock criteria (`bias_ok`, `observer_ok`, `radius_ok >= 0.10`, `error_ok < 0.20/pi`, `phase_ok`) + `25`-step `hold_assist_active`.
- Phase 5 algorithms (`05-tdm-multichannel-and-ui` + `dsp_types.h`):
  - Single-MZM runtime seamless target switching (continuous optical phase `phi_now` tracking, active fringe-copy clamp update `[lo, hi]` without cross-period jumps).
  - Switchless 3-arm DPMZM fixed dual-pilot control (`VA` `f1 = 962.5 Hz`, `VC` `f2 = 1762.5 Hz`, `DSP_TDM_BLOCK_SIZE = 5120`, `VB` parent DC-only demodulated via $f_1 \pm f_2$ cross-intermodulation).
- Host physical optical simulator (`tests/host/sim/test_mzm.c`: bilinear one-pole `-82°` photoreceiver, `0.006 * dither` RAM, additive harmonic pickup, `0.63×` power step, drift injection) + thesis literature baseline comparison summary (`B1–B4` vs affine full-period error budget: `B3` worst `1.124°`, `B4` worst `2.957°`, affine frozen `0.184°`, affine + online tracking `0.044°`).

#### `docs/experiments/`
- `docs/experiments/README.md`: Master experiment ledger table (`EXP-001` Phase 1 Bring-Up, `EXP-002` Phase 2 Peripherals, `EXP-003` Phase 4 Closed-Loop Optical Verification, plus reserved `EXP-004` Long-term Drift, `EXP-005` Seamless Switching, `EXP-006` DPMZM Dual-Pilot).
- `docs/experiments/template.md`: Standard experiment report template.
- Move `docs/bringup/phase1-h503-bringup.md`, `phase2-peripherals-verify.md`, `phase4-closed-loop-verify.md` -> `docs/experiments/bringup/` and ensure relative links inside them work cleanly (note raw `.agent-bridge/*.log` references in prose as local non-tracked capture paths, NOT clickable Markdown links).

---

## 2. GitHub Actions & Verification Scripts Specification

### 2.1 `scripts/check_repo_hygiene.py`
Fast-fail Python 3.11+ CLI:
- Arguments: `--root` (default `.`), `--diff-base` (optional git ref for PR doc-sync summary), `--summary-file` (optional file path to append Markdown summary), `--strict-doc-sync` (optional flag).
- **Check 1 (`check_fpc2_and_pa8`)**: Inspect `boards/xidian/biasctrl_h503/biasctrl_h503.dts`:
  - Fail if `very-high-speed` is present.
  - Fail if `mco1` (case-insensitive) is present.
  - Fail if any of `i2c1_scl_pb8`, `i2c1_sda_pb7`, `spi1_sck_pa5`, `spi1_mosi_pa7`, `spi2_sck_pb13`, `spi2_miso_pb14`, `spi2_mosi_pb15` node blocks lacks `slew-rate = "low-speed";`.
  - Fail if any `zephyr,user` FPC2 property (`pilot-sw-gpios`, `spare-pa8-gpios`, `dac-clr-gpios`, `dac-sync-gpios`, `dac-ldac-gpios`, `adc-cs-gpios`, `adc-sync-gpios`, `adc-drdy-gpios`) lacks `STM32_GPIO_LOW_SPEED`.
- **Check 2 (`check_no_tmp`)**: Scan files under `app/`, `boards/`, `drivers/`, `include/`, `lib/`, `scripts/`, `tests/`, `.github/`, `pyproject.toml` (skipping `scripts/check_repo_hygiene.py` and `tests/host/test_check_repo_hygiene.py`) for hardcoded `/tmp` paths (`(?<![A-Za-z0-9_.-])/tmp(?:/|['"\s]|$)`).
- **Check 3 (`check_markdown_links`)**:
  - Scan `README.md`, `AGENTS.md`, `docs/**/*.md`, `openspec/**/*.md`: fail if any local `file` URI (`file` + `://`) appears anywhere.
  - In `README.md`, `AGENTS.md`, `docs/**/*.md` (skipping fenced code blocks), check every `[text](target)` link (ignoring `http://`, `https://`, `mailto:`, `sidecar://`, `#anchor`): reject leading `/` or Windows drive letters, reject targets pointing into `build/` or `.agent-bridge/`, and verify the resolved file path exists.
- **Check 4 (`report_doc_sync`, when `--diff-base` is given)**: Compute `git diff --name-only <diff_base>...HEAD` and report suggested doc checks in stdout / `--summary-file`.

### 2.2 `scripts/run_qemu_ztest.py`
Fast-fail Python 3.11+ CLI:
- Arguments: `--build-dir` (default `build/ztest_ci`), `--timeout` (default `120` s), `--pristine`, `--skip-build`.
- Prepend `$ZEPHYR_SDK_INSTALL_DIR/hosttools/usr/bin` to `PATH` if that directory exists. Verify `west` and `qemu-system-arm` are resolvable (fast-fail with explicit error if missing).
- Unless `--skip-build`, run `west build -b qemu_cortex_m3 -d <build_dir> tests/ztest` (`--pristine` if requested).
- Run `west build -d <build_dir> -t run` with `start_new_session=True`, streaming lines live. When `PROJECT EXECUTION SUCCESSFUL` is seen, terminate the entire process group (`os.killpg`) and exit `0`. Fail (exit `1`) on `PROJECT EXECUTION FAILED`, `FATAL ERROR`, timeout, or unexpected exit without success marker.

### 2.3 `.github/workflows/ci.yml` & `.github/workflows/release.yml`
- `ci.yml`:
  - Triggers: `push` on `main`, `pull_request`, `workflow_dispatch`, with `concurrency` canceling stale PR runs.
  - Jobs: `hygiene-and-docs`, `host-tests`, `zephyr-build-and-budget` (`ghcr.io/zephyrproject-rtos/ci:latest`, cache `deps` + `.west` by `manifest/west.yml`, `safe.directory`, `.agent-bridge/tmp` `TMPDIR`, `check_mem_budget.py`, step summary table, upload `zephyr.{elf,hex,bin,map}`), `zephyr-ztest` (`ghcr.io/zephyrproject-rtos/ci:latest`, same cache, `python3 scripts/run_qemu_ztest.py --build-dir build/ztest --pristine --timeout 180`).
- `release.yml`:
  - Triggers: `push.tags: ['v*']`, `workflow_dispatch`.
  - Gates on `hygiene-and-docs`, `host-tests`, `zephyr-ztest`, builds `biasctrl_h503`, verifies `check_mem_budget.py`, generates `SHA256SUMS.txt`, and creates a draft GitHub Release (`gh release create` via `GH_TOKEN: ${{ secrets.GITHUB_TOKEN }}`).
