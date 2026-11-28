"""Experiment C3: Bidirectional 181-point calibration, 5-fold CV, restarts, INL, cross-session."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from .algorithms import AffineCalModel, calibrate_affine_from_sweep
from .common import (
    DEFAULT_PILOT_MV,
    DEFAULT_WAIT_BLOCKS,
    RunSession,
    capture_metadata,
    wrap_rad,
)
from .dc_psi import compute_scan_window, fit_dc_psi_waveform
from .metrics import circular_rms, p95_abs
from .sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient


def _build_uart(args: argparse.Namespace) -> UARTClient:
    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261127))
        return UARTClient(transport=SimulatedUARTTransport(plant))
    return UARTClient(port=args.port)


def _eval_affine_angles(cal: AffineCalModel, h1_arr: np.ndarray, h2_arr: np.ndarray) -> np.ndarray:
    z = np.column_stack(
        [
            h2_arr.real * np.cos(cal.h2_ref_phase) + h2_arr.imag * np.sin(cal.h2_ref_phase),
            h1_arr.real * np.cos(cal.h1_ref_phase) + h1_arr.imag * np.sin(cal.h1_ref_phase),
        ]
    )
    u = (z - cal.b_cal) @ cal.B_cal.T
    return np.arctan2(u[:, 1], u[:, 0])


def _eval_model_errors(cal: AffineCalModel, h1_arr: np.ndarray, h2_arr: np.ndarray, phi_ref: np.ndarray) -> np.ndarray:
    return np.asarray(wrap_rad(_eval_affine_angles(cal, h1_arr, h2_arr) - phi_ref), dtype=float)


def _eval_dc_cos_rmse_mv(phi_hat: np.ndarray, dc_v: np.ndarray) -> float:
    design = np.column_stack([np.ones_like(phi_hat), np.cos(phi_hat)])
    a_dc, b_dc = (float(x) for x in np.linalg.lstsq(design, dc_v * 1e3, rcond=None)[0])
    return float(np.sqrt(np.mean((dc_v * 1e3 - (a_dc + b_dc * np.cos(phi_hat))) ** 2)))


def _fit_labeled_affine(h1_arr: np.ndarray, h2_arr: np.ndarray, phi_ref: np.ndarray, theta1: float, theta2: float) -> tuple[np.ndarray, np.ndarray]:
    s1 = h1_arr.real * np.cos(theta1) + h1_arr.imag * np.sin(theta1)
    s2 = h2_arr.real * np.cos(theta2) + h2_arr.imag * np.sin(theta2)
    z = np.column_stack([s2, s1])
    design = np.column_stack([np.cos(phi_ref), np.sin(phi_ref), np.ones_like(phi_ref)])
    c = np.linalg.lstsq(design, z, rcond=None)[0]
    a_fwd = c[:2].T
    b_off = c[2]
    return np.linalg.inv(a_fwd), b_off


def _run_5fold_blocked_cv(
    voltages: np.ndarray,
    h1_arr: np.ndarray,
    h2_arr: np.ndarray,
    dc_v: np.ndarray,
    phi_ref: np.ndarray,
    pilot_v: float,
) -> dict[str, float]:
    n = voltages.size
    fold_bounds = np.linspace(0, n, 6, dtype=int)
    err_labeled, err_unsupervised, err_diag = [], [], []
    for k in range(5):
        lo, hi = int(fold_bounds[k]), int(fold_bounds[k + 1])
        mask_test = np.zeros(n, dtype=bool)
        mask_test[lo:hi] = True
        mask_train = ~mask_test

        cal_unsup = calibrate_affine_from_sweep(
            voltages[mask_train], h1_arr[mask_train], h2_arr[mask_train], dc_v[mask_train], pilot_v=pilot_v
        )
        err_unsupervised.append(_eval_model_errors(cal_unsup, h1_arr[mask_test], h2_arr[mask_test], phi_ref[mask_test]))

        # Labeled 2D affine fit using external/DC-PSI phase labels on training folds.
        b_lbl, off_lbl = _fit_labeled_affine(
            h1_arr[mask_train], h2_arr[mask_train], phi_ref[mask_train], cal_unsup.h1_ref_phase, cal_unsup.h2_ref_phase
        )
        z_test = np.column_stack(
            [
                h2_arr[mask_test].real * np.cos(cal_unsup.h2_ref_phase) + h2_arr[mask_test].imag * np.sin(cal_unsup.h2_ref_phase),
                h1_arr[mask_test].real * np.cos(cal_unsup.h1_ref_phase) + h1_arr[mask_test].imag * np.sin(cal_unsup.h1_ref_phase),
            ]
        )
        u_lbl = (z_test - off_lbl) @ b_lbl.T
        err_labeled.append(np.asarray(wrap_rad(np.arctan2(u_lbl[:, 1], u_lbl[:, 0]) - phi_ref[mask_test]), dtype=float))

        # Diagonal 2D baseline (zero off-diagonals and zero center offset).
        b_diag = np.diag([1.0 / cal_unsup.A_hat[0, 0], 1.0 / cal_unsup.A_hat[1, 1]])
        u_diag = z_test @ b_diag.T
        err_diag.append(np.asarray(wrap_rad(np.arctan2(u_diag[:, 1], u_diag[:, 0]) - phi_ref[mask_test]), dtype=float))

    return {
        "labeled_cv_rms_mrad": circular_rms(np.concatenate(err_labeled)) * 1e3,
        "ellipse_dc_cv_rms_mrad": circular_rms(np.concatenate(err_unsupervised)) * 1e3,
        "diagonal_cv_rms_mrad": circular_rms(np.concatenate(err_diag)) * 1e3,
    }


def _collect_sweep(
    uart: UARTClient,
    voltages_v: np.ndarray,
    pilot_mv: float,
    t_wait_blocks: int,
    blocks_per_point: int,
    direction: str,
    session_idx: int,
    csv_rows: list[dict[str, Any]],
) -> dict[str, np.ndarray]:
    h1_list, h2_list, dc_list, code_list = [], [], [], []
    for idx, vv in enumerate(voltages_v):
        # 2.0 s initial pre-soak settles LiNbO3 relaxation after the large jump to sweep start.
        w = max(t_wait_blocks, 100) if idx == 0 else t_wait_blocks
        rows = uart.meas(float(vv * 1e3), pilot_mv, w, blocks_per_point)
        h1_m = complex(np.mean([r.h1_complex_v for r in rows]))
        h2_m = complex(np.mean([r.h2_complex_v for r in rows]))
        dc_mv = float(np.mean([r.dc_mv for r in rows]))
        code_val = rows[-1].code
        h1_list.append(h1_m)
        h2_list.append(h2_m)
        dc_list.append(dc_mv * 1e-3)
        code_list.append(code_val)
        csv_rows.append(
            {
                "session": session_idx,
                "direction": direction,
                "point_idx": idx,
                "bias_mv": float(vv * 1e3),
                "code": code_val,
                "h1_i_uv": int(round(h1_m.real * 1e6)),
                "h1_q_uv": int(round(h1_m.imag * 1e6)),
                "h2_i_uv": int(round(h2_m.real * 1e6)),
                "h2_q_uv": int(round(h2_m.imag * 1e6)),
                "dc_mv": dc_mv,
            }
        )
    return {
        "voltages_v": voltages_v.copy(),
        "h1": np.asarray(h1_list, dtype=complex),
        "h2": np.asarray(h2_list, dtype=complex),
        "dc_v": np.asarray(dc_list, dtype=float),
        "codes": np.asarray(code_list, dtype=int),
    }


def run_c3(
    uart: UARTClient,
    session: RunSession,
    *,
    sessions: int = 3,
    n_points: int = 181,
    blocks_per_point: int = 3,
    t_wait_blocks: int = DEFAULT_WAIT_BLOCKS,
    pilot_mv: float = DEFAULT_PILOT_MV,
    vpi_est_mv: float = 5382.0,
    v_lock_mv: float = -340.0,
    n_restarts: int = 10,
) -> dict[str, Any]:
    """Execute C3 bidirectional full-cycle calibration, CV, restarts, INL, and cross-session retest."""
    v_start, _ = compute_scan_window(v_lock_mv * 1e-3, vpi_est_mv * 1e-3)
    v_fwd = np.linspace(v_start, v_start + 2.0 * (vpi_est_mv * 1e-3), n_points)
    v_rev = v_fwd[::-1].copy()

    session_models: list[AffineCalModel] = []
    session_sweeps: list[dict[str, Any]] = []
    session_summaries: list[dict[str, Any]] = []

    for s_idx in range(sessions):
        key = f"session_{s_idx}"
        if not session.is_done(key):
            csv_rows: list[dict[str, Any]] = []
            fwd = _collect_sweep(uart, v_fwd, pilot_mv, t_wait_blocks, blocks_per_point, "fwd", s_idx, csv_rows)
            rev = _collect_sweep(uart, v_rev, pilot_mv, t_wait_blocks, blocks_per_point, "rev", s_idx, csv_rows)
            session.append_csv(f"c3_session_{s_idx}.csv", list(csv_rows[0].keys()), csv_rows)

            # High-density synchronous CH1 DC fit gives reference phase labels across 181 points.
            psi_fwd = fit_dc_psi_waveform(fwd["voltages_v"], fwd["dc_v"] * 1e3, v_lock_v=v_lock_mv * 1e-3, vpi_est_v=vpi_est_mv * 1e-3)
            psi_rev = fit_dc_psi_waveform(rev["voltages_v"], rev["dc_v"] * 1e3, v_lock_v=v_lock_mv * 1e-3, vpi_est_v=vpi_est_mv * 1e-3)
            phi_ref_fwd = np.asarray(wrap_rad(np.pi * (fwd["voltages_v"] - v_lock_mv * 1e-3) / psi_fwd.vpi_v + psi_fwd.phi_true_thesis_rad), dtype=float)
            phi_ref_rev = np.asarray(wrap_rad(np.pi * (rev["voltages_v"] - v_lock_mv * 1e-3) / psi_rev.vpi_v + psi_rev.phi_true_thesis_rad), dtype=float)

            cal_fwd = calibrate_affine_from_sweep(fwd["voltages_v"], fwd["h1"], fwd["h2"], fwd["dc_v"], pilot_v=pilot_mv * 1e-3)
            cal_rev = calibrate_affine_from_sweep(rev["voltages_v"], rev["h1"], rev["h2"], rev["dc_v"], pilot_v=pilot_mv * 1e-3)
            cal_comb = calibrate_affine_from_sweep(
                np.concatenate([fwd["voltages_v"], rev["voltages_v"]]),
                np.concatenate([fwd["h1"], rev["h1"]]),
                np.concatenate([fwd["h2"], rev["h2"]]),
                np.concatenate([fwd["dc_v"], rev["dc_v"]]),
                pilot_v=pilot_mv * 1e-3,
            )

            e_in_fwd = _eval_model_errors(cal_fwd, fwd["h1"], fwd["h2"], phi_ref_fwd)
            e_in_rev = _eval_model_errors(cal_rev, rev["h1"], rev["h2"], phi_ref_rev)
            cv_res = _run_5fold_blocked_cv(fwd["voltages_v"], fwd["h1"], fwd["h2"], fwd["dc_v"], phi_ref_fwd, pilot_mv * 1e-3)
            origin_diff_mv = abs(cal_fwd.null_v_thesis - cal_rev.null_v_thesis) * 1e3
            origin_diff_deg = abs(origin_diff_mv * 180.0 / (cal_comb.vpi_v * 1e3))

            # 10 reference-restart consistency trials at v_lock_mv.
            restart_phis = []
            for _ in range(n_restarts):
                uart.stop()
                rows_r = uart.meas(v_lock_mv, pilot_mv, max(t_wait_blocks, 12), blocks_per_point)
                h1_r = complex(np.mean([r.h1_complex_v for r in rows_r]))
                h2_r = complex(np.mean([r.h2_complex_v for r in rows_r]))
                u_r = cal_comb.B_cal @ (cal_comb.project(h1_r, h2_r) - cal_comb.b_cal)
                restart_phis.append(float(np.arctan2(u_r[1], u_r[0])))
            phi_r_arr = np.asarray(restart_phis, dtype=float)
            phi_r_mean = float(np.angle(np.mean(np.exp(1j * phi_r_arr))))
            restart_err = np.asarray(wrap_rad(phi_r_arr - phi_r_mean), dtype=float)

            # 512-code DAC INL spur check on H2 residuals along code % 512.
            res_u_fwd = _eval_model_errors(cal_comb, fwd["h1"], fwd["h2"], phi_ref_fwd)
            mod512 = fwd["codes"] % 512
            inl_corr = float(np.abs(np.mean(res_u_fwd * np.exp(-1j * 2.0 * np.pi * mod512 / 512.0))) * 2.0 * 1e3)
            dc_aff_rmse = _eval_dc_cos_rmse_mv(_eval_affine_angles(cal_fwd, fwd["h1"], fwd["h2"]), fwd["dc_v"])

            payload = {
                "session": s_idx,
                "in_fit_median_mrad": float(np.median(np.abs(np.concatenate([e_in_fwd, e_in_rev])))) * 1e3,
                "dc_vs_affine_rmse_mv": dc_aff_rmse,
                "kappa": cal_comb.kappa,
                "sigma_min_mv": cal_comb.sigma_min * 1e3,
                "vpi_v": cal_comb.vpi_v,
                "null_v_thesis": cal_comb.null_v_thesis,
                "null_v_fw": cal_comb.null_v_fw,
                "bidirectional_origin_diff_mv": origin_diff_mv,
                "bidirectional_origin_diff_deg": origin_diff_deg,
                "restart_consistency_rms_mrad": circular_rms(restart_err) * 1e3,
                "restart_consistency_max_mrad": float(np.max(np.abs(restart_err))) * 1e3,
                "dac_512code_inl_spur_mrad": inl_corr,
                **cv_res,
                "model": {
                    "B_cal": cal_comb.B_cal.tolist(),
                    "b_cal": cal_comb.b_cal.tolist(),
                    "A_hat": cal_comb.A_hat.tolist(),
                    "h1_ref_phase": cal_comb.h1_ref_phase,
                    "h2_ref_phase": cal_comb.h2_ref_phase,
                    "h1_mean_real": cal_comb.h1_mean_complex.real,
                    "h1_mean_imag": cal_comb.h1_mean_complex.imag,
                    "h2_mean_real": cal_comb.h2_mean_complex.real,
                    "h2_mean_imag": cal_comb.h2_mean_complex.imag,
                    "a1_cal": cal_comb.a1_cal,
                    "a2_cal": cal_comb.a2_cal,
                    "a1_signed": cal_comb.a1_signed,
                    "dc_a_v": cal_comb.dc_a_v,
                    "dc_b_v": cal_comb.dc_b_v,
                    "vpi_v": cal_comb.vpi_v,
                    "null_v_thesis": cal_comb.null_v_thesis,
                    "null_v_fw": cal_comb.null_v_fw,
                    "pilot_v": cal_comb.pilot_v,
                    "kappa": cal_comb.kappa,
                    "sigma_min": cal_comb.sigma_min,
                    "winding_sign": cal_comb.winding_sign,
                },
                "fwd_h1_re": fwd["h1"].real.tolist(),
                "fwd_h1_im": fwd["h1"].imag.tolist(),
                "fwd_h2_re": fwd["h2"].real.tolist(),
                "fwd_h2_im": fwd["h2"].imag.tolist(),
                "fwd_dc_v": fwd["dc_v"].tolist(),
                "phi_ref_fwd": phi_ref_fwd.tolist(),
            }
            session.mark_done(key, payload)

        saved = session.get_data(key)
        m_d = saved["model"]
        cal_obj = AffineCalModel(
            B_cal=np.asarray(m_d["B_cal"], dtype=float),
            b_cal=np.asarray(m_d["b_cal"], dtype=float),
            A_hat=np.asarray(m_d["A_hat"], dtype=float),
            h1_ref_phase=float(m_d["h1_ref_phase"]),
            h2_ref_phase=float(m_d["h2_ref_phase"]),
            h1_mean_complex=complex(m_d["h1_mean_real"], m_d["h1_mean_imag"]),
            h2_mean_complex=complex(m_d["h2_mean_real"], m_d["h2_mean_imag"]),
            a1_cal=float(m_d["a1_cal"]),
            a2_cal=float(m_d["a2_cal"]),
            a1_signed=float(m_d["a1_signed"]),
            dc_a_v=float(m_d["dc_a_v"]),
            dc_b_v=float(m_d["dc_b_v"]),
            vpi_v=float(m_d["vpi_v"]),
            null_v_thesis=float(m_d["null_v_thesis"]),
            null_v_fw=float(m_d["null_v_fw"]),
            pilot_v=float(m_d["pilot_v"]),
            kappa=float(m_d["kappa"]),
            sigma_min=float(m_d["sigma_min"]),
            winding_sign=int(m_d["winding_sign"]),
        )
        session_models.append(cal_obj)
        session_sweeps.append(saved)
        session_summaries.append(
            {
                k: v
                for k, v in saved.items()
                if k not in ("model", "fwd_h1_re", "fwd_h1_im", "fwd_h2_re", "fwd_h2_im", "fwd_dc_v", "phi_ref_fwd")
            }
        )

    # Cross-session frozen retest using Session 0 model evaluated on subsequent sessions.
    cross_errors = []
    cross_model_diffs = []
    cross_dc_rmses = []
    base_model = session_models[0]
    for s_obj, saved in zip(session_models[1:], session_sweeps[1:]):
        h1_s = np.asarray(saved["fwd_h1_re"]) + 1j * np.asarray(saved["fwd_h1_im"])
        h2_s = np.asarray(saved["fwd_h2_re"]) + 1j * np.asarray(saved["fwd_h2_im"])
        phi_s = np.asarray(saved["phi_ref_fwd"], dtype=float)
        cross_errors.append(_eval_model_errors(base_model, h1_s, h2_s, phi_s))
        phi_base = _eval_affine_angles(base_model, h1_s, h2_s)
        phi_cur = _eval_affine_angles(s_obj, h1_s, h2_s)
        cross_model_diffs.append(np.asarray(wrap_rad(phi_base - phi_cur), dtype=float))
        dc_s = np.asarray(saved["fwd_dc_v"], dtype=float)
        cross_dc_rmses.append(_eval_dc_cos_rmse_mv(phi_base, dc_s))

    cross_retest = {}
    if cross_errors:
        pooled = np.concatenate(cross_errors)
        pooled_model = np.concatenate(cross_model_diffs)
        cross_retest = {
            "cross_session_frozen_rms_mrad": circular_rms(pooled) * 1e3,
            "cross_session_frozen_p95_deg": float(np.degrees(p95_abs(pooled))),
            "cross_session_model_rms_mrad": circular_rms(pooled_model) * 1e3,
            "cross_session_model_p95_deg": float(np.degrees(p95_abs(pooled_model))),
            "cross_session_dc_rmse_mv": float(np.mean(cross_dc_rmses)),
            "pass_p95_le_1deg": bool(np.degrees(p95_abs(pooled)) <= 1.0),
        }

    uart.stop()
    summary = {
        "sessions": session_summaries,
        "cross_session_frozen_retest": cross_retest,
        "aggregate": {
            "median_in_fit_mrad": float(np.median([s["in_fit_median_mrad"] for s in session_summaries])),
            "mean_labeled_cv_rms_mrad": float(np.mean([s["labeled_cv_rms_mrad"] for s in session_summaries])),
            "mean_ellipse_dc_cv_rms_mrad": float(np.mean([s["ellipse_dc_cv_rms_mrad"] for s in session_summaries])),
            "mean_diagonal_cv_rms_mrad": float(np.mean([s["diagonal_cv_rms_mrad"] for s in session_summaries])),
            "mean_kappa": float(np.mean([s["kappa"] for s in session_summaries])),
            "mean_origin_diff_mv": float(np.mean([s["bidirectional_origin_diff_mv"] for s in session_summaries])),
        },
    }
    session.write_json("summary.json", summary)
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Experiment C3 Bidirectional Calibration")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--sessions", type=int, default=3)
    parser.add_argument("--n-points", type=int, default=181)
    parser.add_argument("--blocks-per-point", type=int, default=3)
    parser.add_argument("--t-wait-blocks", type=int, default=DEFAULT_WAIT_BLOCKS)
    parser.add_argument("--pilot-mv", type=float, default=DEFAULT_PILOT_MV)
    parser.add_argument("--vpi-est-mv", type=float, default=5382.0)
    parser.add_argument("--v-lock-mv", type=float, default=-340.0)
    parser.add_argument("--n-restarts", type=int, default=10)
    args = parser.parse_args(argv)

    session = RunSession("c3", run_dir=args.run_dir, resume=args.resume)
    session.write_metadata(capture_metadata("c3", dry_run=args.dry_run))
    with _build_uart(args) as uart:
        run_c3(
            uart,
            session,
            sessions=args.sessions,
            n_points=args.n_points,
            blocks_per_point=args.blocks_per_point,
            t_wait_blocks=args.t_wait_blocks,
            pilot_mv=args.pilot_mv,
            vpi_est_mv=args.vpi_est_mv,
            v_lock_mv=args.v_lock_mv,
            n_restarts=args.n_restarts,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
