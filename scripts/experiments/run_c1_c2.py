"""Experiment C1 (onboard bringup checks) and C2 (step peaks, T_wait, CH1 margin, repeatability)."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from .algorithms import calibrate_affine_from_sweep
from .common import (
    DEFAULT_PILOT_MV,
    DEFAULT_SAMPLE_RATE_HZ,
    DEFAULT_STANDARD_WINDOW_SAMPLES,
    RunSession,
    capture_metadata,
    fw_to_thesis_rad,
    wrap_rad,
)
from .dc_psi import fit_dc_psi_waveform
from .metrics import cond_kappa_and_sigma_min, paired_repeatability_stats
from .sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient, pyocd_reset

ADC_FULL_SCALE_COUNTS = 8388608.0
ADC_FS_MV = 1200.0
DEFAULT_LOW_PILOT_MV = 84.9  # ~80 mV electrode peak after 0.9425 factor


def _build_uart(args: argparse.Namespace) -> UARTClient:
    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261126))
        return UARTClient(transport=SimulatedUARTTransport(plant))
    return UARTClient(port=args.port)


def _sweep_one_period(
    uart: UARTClient,
    v_start_v: float,
    v_end_v: float,
    n_points: int,
    pilot_mv: float,
    wait_blocks: int,
    blocks_per_point: int = 2,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    voltages = np.linspace(v_start_v, v_end_v, n_points)
    h1_list, h2_list, dc_list = [], [], []
    for idx, vv in enumerate(voltages):
        w = max(wait_blocks, 12) if idx == 0 else wait_blocks
        rows = uart.meas(float(vv * 1e3), pilot_mv, w, blocks_per_point)
        h1_list.append(complex(np.mean([r.h1_complex_v for r in rows])))
        h2_list.append(complex(np.mean([r.h2_complex_v for r in rows])))
        dc_list.append(float(np.mean([r.dc_mv * 1e-3 for r in rows])))
    return voltages, np.asarray(h1_list), np.asarray(h2_list), np.asarray(dc_list)


def run_c1_c2(
    uart: UARTClient,
    session: RunSession,
    *,
    dry_run: bool = False,
    skip_reset: bool = False,
    pilot_mv: float = DEFAULT_PILOT_MV,
    low_pilot_mv: float = DEFAULT_LOW_PILOT_MV,
    step_repeats: int = 30,
    repeatability_chunks: int = 16,
    repeatability_pairs: int = 15,
    c1_bias_blocks: int = 20,
) -> dict[str, Any]:
    """Execute C1 onboard hardware checks and all 4 C2 electro-optic validation items."""
    # --- C1.1: 3 cold boots + hw info ---
    if not session.is_done("c1_cold_boots"):
        boots: list[dict[str, Any]] = []
        for idx in range(3):
            boot_lines: list[str] = []
            if not dry_run and not skip_reset:
                pyocd_reset()
                boot_lines = uart.read_boot_banner(timeout_s=8.0)
            info = uart.hw_info()
            if info.sysclk_hz != 250_000_000:
                raise RuntimeError(f"Unexpected SYSCLK={info.sysclk_hz}")
            if (info.adc_id >> 8) != 0x22 or info.adc_mode != 0x1111 or info.adc_clock != 0x0322:
                raise RuntimeError(
                    f"Unexpected ADS131M02 regs: ID=0x{info.adc_id:04X} "
                    f"MODE=0x{info.adc_mode:04X} CLOCK=0x{info.adc_clock:04X}"
                )
            boots.append(
                {
                    "boot_index": idx,
                    "sysclk_hz": info.sysclk_hz,
                    "adc_id": f"0x{info.adc_id:04X}",
                    "adc_stat": f"0x{info.adc_stat:04X}",
                    "adc_mode": f"0x{info.adc_mode:04X}",
                    "adc_clock": f"0x{info.adc_clock:04X}",
                    "desc_crc": f"0x{info.desc_crc:08X}",
                    "cal_status": info.cal_status,
                    "cal_crc": f"0x{info.cal_crc:08X}",
                    "fw": info.fw,
                    "rev": info.rev,
                    "boot_banner_lines": boot_lines,
                }
            )
        session.mark_done("c1_cold_boots", boots)

    # --- C1.2: >= 1e5 DRDY cycle rate & zero-drop check ---
    if not session.is_done("c1_adc_stat"):
        stat = uart.adc_stat(100_000)
        session.mark_done(
            "c1_adc_stat",
            {
                "n": stat.n,
                "total_cyc": stat.total_cyc,
                "fs_hz": stat.fs_hz,
                "dt_min_cyc": stat.dt_min_cyc,
                "dt_max_cyc": stat.dt_max_cyc,
                "drops": stat.drops,
                "crc": stat.crc,
            },
        )

    # --- C1.3: Dark CH0 adc dump 8192 -> 0-32 kHz FFT ---
    if not session.is_done("c1_dark_dump"):
        dump = uart.adc_dump(8192)
        seg_spectra = []
        n_fft = DEFAULT_STANDARD_WINDOW_SAMPLES
        freqs = np.fft.rfftfreq(n_fft, d=1.0 / DEFAULT_SAMPLE_RATE_HZ)
        for seg in dump.segments:
            if len(seg.samples) != n_fft:
                continue
            volts = np.asarray(seg.samples, dtype=float) * (1.2 / ADC_FULL_SCALE_COUNTS)
            volts_ac = volts - np.mean(volts)
            spec = np.abs(np.fft.rfft(volts_ac)) * (2.0 / n_fft)
            seg_spectra.append(spec)
        if not seg_spectra:
            raise RuntimeError("No full 1280-sample segments in adc dump 8192")
        mean_spec = np.mean(seg_spectra, axis=0)
        bin_1k = int(np.argmin(np.abs(freqs - 1000.0)))
        bin_2k = int(np.argmin(np.abs(freqs - 2000.0)))
        bin_50 = int(np.argmin(np.abs(freqs - 50.0)))
        # Noise floor density excluding DC/50Hz/1k/2k tones.
        mask = np.ones_like(freqs, dtype=bool)
        mask[[0, bin_50, bin_1k, bin_2k]] = False
        bin_bw = DEFAULT_SAMPLE_RATE_HZ / n_fft  # 50 Hz
        rms_floor_v = float(np.median(mean_spec[mask]) / np.sqrt(2.0))
        noise_density_nv_rthz = float((rms_floor_v / np.sqrt(bin_bw)) * 1e9)
        session.mark_done(
            "c1_dark_dump",
            {
                "total_samples": dump.total_samples,
                "n_full_segments": len(seg_spectra),
                "tone_50hz_uv_pk": float(mean_spec[bin_50] * 1e6),
                "tone_1khz_uv_pk": float(mean_spec[bin_1k] * 1e6),
                "tone_2khz_uv_pk": float(mean_spec[bin_2k] * 1e6),
                "max_spur_uv_pk": float(np.max(mean_spec[1:]) * 1e6),
                "noise_floor_nv_rthz": noise_density_nv_rthz,
                "alias_192khz_note": "192 kHz = 3 * 64 kSPS folds to DC (sinc3 third notch)",
                "fft_freqs_hz": [float(f) for f in freqs],
                "fft_mag_uv_pk": [float(v * 1e6) for v in mean_spec],
            },
        )

    # --- C1.4: Bias-code pilot independence sweep + external-instrument exclusions ---
    if not session.is_done("c1_pilot_independence"):
        bias_levels_mv = [-8000.0, -4000.0, 0.0, 4000.0, 8000.0]
        records = []
        for idx, b_mv in enumerate(bias_levels_mv):
            rows = uart.meas(b_mv, pilot_mv, 12 if idx == 0 else 7, c1_bias_blocks)
            h1_v = np.mean([r.h1_complex_v for r in rows])
            h2_v = np.mean([r.h2_complex_v for r in rows])
            records.append(
                {
                    "bias_mv": b_mv,
                    "h1_mag_mv": float(abs(h1_v) * 1e3),
                    "h1_phase_rad": float(np.angle(h1_v)),
                    "h2_mag_mv": float(abs(h2_v) * 1e3),
                    "h2_phase_rad": float(np.angle(h2_v)),
                    "dc_mv": float(np.mean([r.dc_mv for r in rows])),
                }
            )
        session.mark_done(
            "c1_pilot_independence",
            {
                "bias_levels": records,
                "external_instrument_exclusions": {
                    "dc_linearity_10v": "待外部仪器补测",
                    "electrode_pilot_amp_phase_thd": "待外部仪器补测",
                    "electrode_noise_density": "待外部仪器补测",
                    "power_rail_ripple_and_ic_temp": "待外部仪器补测",
                },
            },
        )

    # --- Pre-C2 calibration sweep on optical path ---
    if not session.is_done("c2_cal_model"):
        st = uart.status()
        vpi_est_v = (st.vpi_mv * 1e-3) if st.vpi_mv > 0 else 5.382
        v_lo = max(-9.8, -vpi_est_v - 1.5)
        v_hi = min(9.8, v_lo + 2.0 * vpi_est_v)
        v_arr, h1_arr, h2_arr, dc_arr = _sweep_one_period(uart, v_lo, v_hi, 61, pilot_mv, 5, blocks_per_point=2)
        cal = calibrate_affine_from_sweep(v_arr, h1_arr, h2_arr, dc_arr, pilot_v=pilot_mv * 1e-3)
        v_quad_v = cal.target_voltage_v(0.5 * np.pi)
        v_nonspec_v = cal.target_voltage_v(0.25 * np.pi)
        session.mark_done(
            "c2_cal_model",
            {
                "vpi_v": cal.vpi_v,
                "null_v_thesis": cal.null_v_thesis,
                "null_v_fw": cal.null_v_fw,
                "v_quad_v": v_quad_v,
                "v_nonspec_45deg_v": v_nonspec_v,
                "kappa": cal.kappa,
                "sigma_min_v": cal.sigma_min,
                "B_cal": cal.B_cal.tolist(),
                "b_cal": cal.b_cal.tolist(),
                "A_hat": cal.A_hat.tolist(),
                "h1_ref_phase": cal.h1_ref_phase,
                "h2_ref_phase": cal.h2_ref_phase,
            },
        )

    cal_info = session.get_data("c2_cal_model")
    v_quad_mv = cal_info["v_quad_v"] * 1e3
    v_nonspec_mv = cal_info["v_nonspec_45deg_v"] * 1e3
    vpi_v = cal_info["vpi_v"]
    B_cal = np.asarray(cal_info["B_cal"], dtype=float)
    b_cal = np.asarray(cal_info["b_cal"], dtype=float)
    theta1 = float(cal_info["h1_ref_phase"])
    theta2 = float(cal_info["h2_ref_phase"])

    # --- C2.1: Quadrature +/-0.1/0.3/0.5 V x N step CH0 peak margin ---
    if not session.is_done("c2_step_peaks"):
        step_summary = {}
        csv_rows: list[dict[str, Any]] = []
        for dv_mv in (100.0, 300.0, 500.0):
            for sign_label, sgn in (("pos", 1.0), ("neg", -1.0)):
                v0_mv = float(np.clip(v_quad_mv - sgn * dv_mv, -9800.0, 9800.0))
                v1_mv = float(np.clip(v_quad_mv, -9800.0, 9800.0))
                max_peaks = []
                clip_count = 0
                total_samples = 0
                for rep in range(step_repeats):
                    rows = uart.step(v0_mv, v1_mv, pilot_mv, 10)
                    # Check post-50 ms window (blocks >= 3 -> t >= 60 ms) and full transient.
                    peak_all = max(max(abs(r.ch0_min_mv), abs(r.ch0_max_mv)) for r in rows)
                    peak_post50 = max(max(abs(r.ch0_min_mv), abs(r.ch0_max_mv)) for r in rows[2:])
                    max_peaks.append(peak_post50)
                    for r in rows:
                        total_samples += 1
                        if abs(r.ch0_min_mv) >= ADC_FS_MV or abs(r.ch0_max_mv) >= ADC_FS_MV:
                            clip_count += 1
                        csv_rows.append(
                            {
                                "dv_mv": dv_mv,
                                "dir": sign_label,
                                "repeat": rep,
                                "seq": r.seq,
                                "ch0_min_mv": r.ch0_min_mv,
                                "ch0_max_mv": r.ch0_max_mv,
                                "peak_all_mv": peak_all,
                            }
                        )
                step_summary[f"{int(dv_mv)}mV_{sign_label}"] = {
                    "dv_mv": dv_mv,
                    "direction": sign_label,
                    "repeats": step_repeats,
                    "post50ms_peak_mean_mv": float(np.mean(max_peaks)),
                    "post50ms_peak_max_mv": float(np.max(max_peaks)),
                    "adc_margin_mv": ADC_FS_MV - float(np.max(max_peaks)),
                    "clip_pct": 100.0 * clip_count / max(total_samples, 1),
                }
        session.append_csv("c2_step_peaks.csv", list(csv_rows[0].keys()), csv_rows)
        session.mark_done("c2_step_peaks", step_summary)

    # --- C2.2: 0-300 ms T_wait scan ---
    if not session.is_done("c2_twait_scan"):
        v0_mv = float(np.clip(v_quad_mv - 500.0, -9800.0, 9800.0))
        v1_mv = float(np.clip(v_quad_mv, -9800.0, 9800.0))
        # Capture 16 consecutive 20 ms blocks immediately after step (t = 0..300 ms start).
        all_trials = [uart.step(v0_mv, v1_mv, pilot_mv, 16) for _ in range(10)]
        # Settled reference from last 4 blocks (240..320 ms).
        settled_phis = []
        for trial in all_trials:
            h1_s = np.mean([r.h1_complex_v for r in trial[12:]])
            h2_s = np.mean([r.h2_complex_v for r in trial[12:]])
            z_s = np.array([h2_s.real * np.cos(theta2) + h2_s.imag * np.sin(theta2), h1_s.real * np.cos(theta1) + h1_s.imag * np.sin(theta1)])
            u_s = B_cal @ (z_s - b_cal)
            settled_phis.append(float(np.arctan2(u_s[1], u_s[0])))
        phi_settled = float(np.angle(np.mean(np.exp(1j * np.asarray(settled_phis)))))

        twait_rows = []
        min_twait_ms = None
        for b_idx in range(16):
            errs_deg = []
            peaks_mv = []
            for trial in all_trials:
                r = trial[b_idx]
                h1_c, h2_c = r.h1_complex_v, r.h2_complex_v
                z = np.array([h2_c.real * np.cos(theta2) + h2_c.imag * np.sin(theta2), h1_c.real * np.cos(theta1) + h1_c.imag * np.sin(theta1)])
                u = B_cal @ (z - b_cal)
                phi_b = float(np.arctan2(u[1], u[0]))
                errs_deg.append(abs(float(np.degrees(wrap_rad(phi_b - phi_settled)))))
                peaks_mv.append(max(abs(r.ch0_min_mv), abs(r.ch0_max_mv)))
            mean_err = float(np.mean(errs_deg))
            wait_ms = b_idx * 20
            if min_twait_ms is None and mean_err <= 0.2:
                min_twait_ms = wait_ms
            twait_rows.append(
                {
                    "wait_blocks": b_idx,
                    "wait_ms": wait_ms,
                    "mean_abs_phase_err_deg": mean_err,
                    "max_abs_phase_err_deg": float(np.max(errs_deg)),
                    "ch0_peak_mv": float(np.max(peaks_mv)),
                }
            )
        session.append_csv("c2_twait_scan.csv", list(twait_rows[0].keys()), twait_rows)
        session.mark_done("c2_twait_scan", {"min_twait_le_0p2deg_ms": min_twait_ms, "curve": twait_rows})

    # --- C2.3: Full-cycle CH1 DC max vs 1.2 V ADC margin ---
    if not session.is_done("c2_ch1_margin"):
        v_start = max(-9.8, cal_info["null_v_thesis"] - vpi_v)
        v_grid = np.linspace(v_start, min(9.8, v_start + 2.0 * vpi_v), 17)
        dc_vals = []
        ch1_peaks = []
        for idx, vv in enumerate(v_grid):
            rows = uart.meas(float(vv * 1e3), 0.0, 12 if idx == 0 else 7, 3)
            dc_vals.append(float(np.mean([r.dc_mv for r in rows])))
            ch1_peaks.append(max(max(abs(r.ch1_min_mv), abs(r.ch1_max_mv)) for r in rows))
        psi = fit_dc_psi_waveform(v_grid, dc_vals, v_lock_v=v_quad_mv * 1e-3, vpi_est_v=vpi_v)
        dc_max_mv = float(np.max(dc_vals))
        dc_min_mv = float(np.min(dc_vals))
        peak_mv = float(np.max(ch1_peaks))
        session.mark_done(
            "c2_ch1_margin",
            {
                "dc_min_mv": dc_min_mv,
                "dc_max_mv": dc_max_mv,
                "ch1_peak_mv": peak_mv,
                "adc_fs_mv": ADC_FS_MV,
                "fs_utilization_pct": 100.0 * peak_mv / ADC_FS_MV,
                "margin_mv": ADC_FS_MV - peak_mv,
                "dc_psi_vpi_v": psi.vpi_v,
                "dc_psi_rmse_mv": psi.fit_rmse_mv,
            },
        )

    # --- C2.4: 0.15 V and 0.08 V pilot open-loop repeatability (16 x 15 pairs = 240 pairs of 0.32 s) ---
    if not session.is_done("c2_repeatability"):
        rep_summary: dict[str, Any] = {}
        for p_label, p_mv in (("0p15V", pilot_mv), ("0p08V", low_pilot_mv)):
            # Quick sweep at p_mv to compute exact kappa(A) at this pilot level.
            v_lo = max(-9.8, cal_info["null_v_thesis"] - vpi_v)
            v_hi = min(9.8, v_lo + 2.0 * vpi_v)
            vs, h1s, h2s, dcs = _sweep_one_period(uart, v_lo, v_hi, 33, p_mv, 4, blocks_per_point=2)
            cal_p = calibrate_affine_from_sweep(vs, h1s, h2s, dcs, pilot_v=p_mv * 1e-3)
            kappa_p, sigma_min_p = cond_kappa_and_sigma_min(cal_p.A_hat)

            point_stats: dict[str, Any] = {}
            for pt_name, pt_bias_mv in (("quad_90deg", v_quad_mv), ("nonspec_45deg", v_nonspec_mv)):
                n_windows = repeatability_chunks * repeatability_pairs * 2
                # Acquire 0.32 s (16-block) windows back-to-back.
                phis_320ms = []
                h1_proj_320 = []
                h2_proj_320 = []
                for w_idx in range(n_windows):
                    rows = uart.meas(pt_bias_mv, p_mv, 12 if w_idx == 0 else 0, 16)
                    h1_m = complex(np.mean([r.h1_complex_v for r in rows]))
                    h2_m = complex(np.mean([r.h2_complex_v for r in rows]))
                    z = cal_p.project(h1_m, h2_m)
                    u = cal_p.B_cal @ (z - cal_p.b_cal)
                    phis_320ms.append(float(np.arctan2(u[1], u[0])))
                    h1_proj_320.append(float(z[1]))
                    h2_proj_320.append(float(z[0]))

                # Unwrap around mean before paired differences so +/-pi wraps don't corrupt diffs.
                phi_arr = np.asarray(phis_320ms, dtype=float)
                phi_ref = float(np.angle(np.mean(np.exp(1j * phi_arr))))
                phi_rel = np.asarray(wrap_rad(phi_arr - phi_ref), dtype=float)

                st_phi = paired_repeatability_stats(phi_rel)
                st_h1 = paired_repeatability_stats(h1_proj_320)
                st_h2 = paired_repeatability_stats(h2_proj_320)
                point_stats[pt_name] = {
                    "n_pairs": int(st_phi["n_pairs"]),
                    "sigma_320ms_rad": st_phi["sigma_single"],
                    "sigma_320ms_mrad": st_phi["sigma_single"] * 1e3,
                    "sigma_20ms_equiv_mrad": st_phi["sigma_single"] * np.sqrt(16.0) * 1e3,
                    "h1_sigma_uv": st_h1["sigma_single"] * 1e6,
                    "h2_sigma_uv": st_h2["sigma_single"] * 1e6,
                    "pair_corr": st_phi["pair_corr"],
                }
            rep_summary[p_label] = {
                "pilot_cmd_mv": p_mv,
                "kappa": kappa_p,
                "sigma_min_mv": sigma_min_p * 1e3,
                "points": point_stats,
            }
        session.mark_done("c2_repeatability", rep_summary)

    uart.stop()
    summary = {
        "c1_cold_boots": session.get_data("c1_cold_boots"),
        "c1_adc_stat": session.get_data("c1_adc_stat"),
        "c1_dark_dump": session.get_data("c1_dark_dump"),
        "c1_pilot_independence": session.get_data("c1_pilot_independence"),
        "c2_cal_model": session.get_data("c2_cal_model"),
        "c2_step_peaks": session.get_data("c2_step_peaks"),
        "c2_twait_scan": session.get_data("c2_twait_scan"),
        "c2_ch1_margin": session.get_data("c2_ch1_margin"),
        "c2_repeatability": session.get_data("c2_repeatability"),
    }
    session.write_json("summary.json", summary)
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Experiment C1 & C2")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-reset", action="store_true")
    parser.add_argument("--pilot-mv", type=float, default=DEFAULT_PILOT_MV)
    parser.add_argument("--low-pilot-mv", type=float, default=DEFAULT_LOW_PILOT_MV)
    parser.add_argument("--step-repeats", type=int, default=30)
    parser.add_argument("--repeatability-chunks", type=int, default=16)
    parser.add_argument("--repeatability-pairs", type=int, default=15)
    parser.add_argument("--c1-bias-blocks", type=int, default=20)
    args = parser.parse_args(argv)

    session = RunSession("c1_c2", run_dir=args.run_dir, resume=args.resume)
    session.write_metadata(capture_metadata("c1_c2", dry_run=args.dry_run))
    with _build_uart(args) as uart:
        run_c1_c2(
            uart,
            session,
            dry_run=args.dry_run,
            skip_reset=args.skip_reset,
            pilot_mv=args.pilot_mv,
            low_pilot_mv=args.low_pilot_mv,
            step_repeats=args.step_repeats,
            repeatability_chunks=args.repeatability_chunks,
            repeatability_pairs=args.repeatability_pairs,
            c1_bias_blocks=args.c1_bias_blocks,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
