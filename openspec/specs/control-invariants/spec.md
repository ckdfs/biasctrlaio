# Control & DSP Invariants Specification

## Purpose
Preserves the verified physical and mathematical invariants from `biascontrol_h523` while adapting memory usage to the `32 KiB` SRAM limit of `STM32H503CBT6`.

## Requirements

### Requirement: Coherent Harmonic Extraction & Phase Observer
1. **Zero-Leakage Goertzel Block**: At `64 kSPS` and `f0 = 1 kHz`, block length `N = 1280` (`20 ms`) spans an integer number of pilot cycles; any sample-rate or pilot-frequency change MUST preserve integer cycles per window.
2. **H2 Q-Component Sign**: Second harmonic observation MUST use `h2_mag * sin(h2_phase)` (Q-component), matching hardware phase lag (~82°) and avoiding near-singular affine calibration matrices.
3. **Bessel Row Scaling**: Runtime effective affine matrix `M_eff` rows MUST scale by `J1(m_now)/J1(m_cal)` and `J2(m_now)/J2(m_cal)` before inversion.
4. **Unit-Circle Normalization**: Observer vector `[obs_x, obs_y]` MUST be normalized to unit radius to decouple phase error from optical and RF power variations.
5. **Cold-Start Seeding & Spring Stabilization**: Integrator and observer MUST seed from calibrated coarse sweep bias, apply `sin^2(phi_target)` voltage spring + `obs_dc` fast EMA (`alpha = 0.50`), and satisfy all 5 lock criteria (`bias_ok`, `observer_ok`, `radius_ok`, `error_ok`, `phase_ok`) for 25 consecutive updates before activating `hold_assist_active`.
6. **Monitor-Only DC Channel**: `ADS131M02` `CH1` DC mean MUST NOT be mixed into the primary phase error loop.

#### Scenario: Invariant Harmonic Demodulation and Lock Stabilization
- **WHEN** closed-loop MZM control runs with pilot modulation and optical/RF amplitude variation
- **THEN** coherent Goertzel demodulation, Bessel row scaling, unit-circle normalization, and 5-criterion lock supervision hold steady-state tracking without `CH1` DC feedback mixing.

### Requirement: O(1)-RAM Calibration & Real-Time ISR Isolation
1. **Online Least-Squares Calibration**: Pass-2 affine fitting MUST use O(1)-RAM online normal-equation accumulation instead of multi-kilobyte static scan buffers (`< 1 KB` total scan state).
2. **64 kSPS Real-Time Isolation**: `DRDY#` (`PA11`) ISR / fast path (`15.625 µs` period) MUST NEVER call blocking I2C, UART, `k_sleep`, or mutex-guarded Zephyr slow APIs. Any SPI/DMA timeout or CRC error MUST fail fast into `FAULT`, turn off all 8 pilot switches, and clamp DAC outputs safely.

#### Scenario: Deterministic O(1)-RAM Calibration and ISR Safety
- **WHEN** two-pass calibration and `64 kSPS` streaming execute on `STM32H503CBT6`
- **THEN** scan state stays `< 1 KB`, `DRDY#` ISR performs no blocking calls, and any SPI/DMA or CRC fault transitions immediately to `FAULT` with safe output clamping.
