"""Experiment C4: Gain sweep + 16-target/dense 5-method head-to-head closed-loop comparison."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from .algorithms import (
    AffineCalModel,
    calibrate_affine_from_sweep,
    make_controller,
    run_closed_loop_episode,
)
from .common import (
    DEFAULT_PILOT_MV,
    DEFAULT_WAIT_BLOCKS,
    RunSession,
    capture_metadata,
    fw_to_thesis_rad,
    thesis_to_fw_rad,
    wrap_rad,
)
from .dc_psi import compute_scan_window, measure_dc_psi
from .metrics import circular_rms, summarize_target_runs, two_period_half_amp
from .sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient

DEFAULT_16_TARGETS_DEG = tuple(np.arange(0.0, 360.0, 22.5).tolist())
DEFAULT_DENSE_OFFSETS_DEG = (5.0, 10.0, 80.0, 85.0, 95.0, 100.0, 170.0, 175.0, 185.0, 190.0, 260.0, 265.0, 275.0, 280.0, 350.0, 355.0)
DEFAULT_METHODS = ("B1", "B2", "B3", "B4", "AffineFrozen")


def _build_uart(args: argparse.Namespace) -> UARTClient:
    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261128))
        return UARTClient(transport=SimulatedUARTTransport(plant))
    return UARTClient(port=args.port)


def _calibrate_session(
    uart: UARTClient,
    v_lock_mv: float,
    vpi_est_mv: float,
    pilot_mv: float,
    t_wait_blocks: int,
    n_cal_points: int,
) -> AffineCalModel:
    v_start, _ = compute_scan_window(v_lock_mv * 1e-3, vpi_est_mv * 1e-3)
    vs = np.linspace(v_start, v_start + 2.0 * (vpi_est_mv * 1e-3), n_cal_points)
    h1s, h2s, dcs = [], [], []
    for idx, vv in enumerate(vs):
        rows = uart.meas(float(vv * 1e3), pilot_mv, max(t_wait_blocks, 12) if idx == 0 else t_wait_blocks, 2)
        h1s.append(complex(np.mean([r.h1_complex_v for r in rows])))
        h2s.append(complex(np.mean([r.h2_complex_v for r in rows])))
        dcs.append(float(np.mean([r.dc_mv * 1e-3 for r in rows])))
    return calibrate_affine_from_sweep(vs, h1s, h2s, dcs, pilot_v=pilot_mv * 1e-3)


def run_c4(
    uart: UARTClient,
    session: RunSession,
    *,
    sessions: int = 3,
    t_wait_blocks: int = DEFAULT_WAIT_BLOCKS,
    pilot_mv: float = DEFAULT_PILOT_MV,
    gain: float = 0.25,
    n_steps: int = 60,
    tail_steps: int = 20,
    methods: tuple[str, ...] = DEFAULT_METHODS,
    include_dense: bool = True,
    skip_gain_sweep: bool = False,
    cal_points: int = 61,
    vpi_est_mv: float = 5382.0,
    v_lock_mv: float = -340.0,
) -> dict[str, Any]:
    """Execute C4 gain pre-check, multi-target 5-method head-to-head comparison, and onboard spot check."""
    if tail_steps > n_steps:
        raise ValueError("tail_steps cannot exceed n_steps")

    # Initial calibration for gain sweep.
    cal_init = _calibrate_session(uart, v_lock_mv, vpi_est_mv, pilot_mv, t_wait_blocks, cal_points)

    # --- 1. Gain pre-check: 4 pts x 5 gains ---
    if not skip_gain_sweep and not session.is_done("c4_gain_sweep"):
        gain_targets_deg = (0.0, 45.0, 90.0, 180.0)
        gains = (0.05, 0.10, 0.15, 0.20, 0.30)
        gain_results = []
        for g_val in gains:
            a2_vals = []
            tail_rms_vals = []
            for tgt_deg in gain_targets_deg:
                tgt_rad = float(np.radians(tgt_deg))
                seed_v = cal_init.target_voltage_v(tgt_rad) + 0.6 * (cal_init.vpi_v / np.pi)
                ctrl = make_controller("AffineFrozen", cal_init, tgt_rad, gain=g_val, pilot_mv=pilot_mv, wait_blocks=t_wait_blocks)
                hist = run_closed_loop_episode(uart, ctrl, seed_v, n_steps)
                tail_errs = np.asarray(wrap_rad([h.phi_hat_thesis_rad - tgt_rad for h in hist[-tail_steps:]]), dtype=float)
                a2_vals.append(two_period_half_amp(tail_errs) * 1e3)
                tail_rms_vals.append(circular_rms(tail_errs) * 1e3)
            gain_results.append(
                {
                    "gain": g_val,
                    "max_two_period_a2_mrad": float(np.max(a2_vals)),
                    "mean_tail_rms_mrad": float(np.mean(tail_rms_vals)),
                }
            )
        session.mark_done("c4_gain_sweep", gain_results)

    # --- 2. 16 equidistant + optional dense targets x +/-1 rad seeds x 3 sessions x 5 methods ---
    targets_deg = list(DEFAULT_16_TARGETS_DEG)
    if include_dense:
        targets_deg.extend(DEFAULT_DENSE_OFFSETS_DEG)

    trace_csv_fields = [
        "session",
        "method",
        "target_thesis_deg",
        "target_fw_deg",
        "seed_rad",
        "step_idx",
        "bias_mv",
        "phi_hat_thesis_rad",
        "phi_hat_fw_rad",
        "err_est_rad",
        "radius",
        "dc_mv",
        "drops",
        "crc",
    ]
    psi_csv_fields = [
        "session",
        "method",
        "target_thesis_deg",
        "target_fw_deg",
        "seed_rad",
        "final_bias_mv",
        "phi_true_thesis_deg",
        "phi_true_fw_deg",
        "e_ctl_deg",
        "e_est_deg",
        "vpi_psi_v",
        "fit_rmse_mv",
        "sigma_phi_deg",
        "drift_mv",
    ]

    all_method_episodes: dict[str, list[dict[str, Any]]] = {m: [] for m in methods}
    for s_idx in range(sessions):
        cal_s = cal_init if s_idx == 0 else _calibrate_session(uart, v_lock_mv, vpi_est_mv, pilot_mv, t_wait_blocks, cal_points)
        for m_name in methods:
            for tgt_deg in targets_deg:
                for seed_rad in (-1.0, 1.0):
                    ep_key = f"s{s_idx}_{m_name}_t{tgt_deg:.1f}_seed{seed_rad:+.1f}"
                    if not session.is_done(ep_key):
                        tgt_rad = float(wrap_rad(np.radians(tgt_deg)))
                        tgt_fw_deg = float(np.degrees(thesis_to_fw_rad(tgt_rad)))
                        v_tgt = cal_s.target_voltage_v(tgt_rad)
                        v_seed = float(np.clip(v_tgt + seed_rad * (cal_s.vpi_v / np.pi), -9.8, 9.8))
                        ctrl = make_controller(
                            m_name,
                            cal_s,
                            tgt_rad,
                            gain=gain,
                            pilot_mv=pilot_mv,
                            wait_blocks=t_wait_blocks,
                        )
                        hist = run_closed_loop_episode(uart, ctrl, v_seed, n_steps)
                        final_bias_v = hist[-1].next_bias_mv * 1e-3
                        # Validate physical optical phase via 17-point In-Situ DC PSI.
                        psi = measure_dc_psi(uart, final_bias_v * 1e3, cal_s.vpi_v * 1e3, pre_wait_blocks=12)
                        e_ctl_rad = float(wrap_rad(psi.phi_true_thesis_rad - tgt_rad))
                        e_est_rad = float(wrap_rad(hist[-1].phi_hat_thesis_rad - psi.phi_true_thesis_rad))

                        # Reconstruct tail external error series anchored by DC PSI + local voltage delta.
                        tail_biases_v = np.asarray([h.bias_mv * 1e-3 for h in hist[-tail_steps:]], dtype=float)
                        tail_phi_ref = np.asarray(
                            wrap_rad(psi.phi_true_thesis_rad + np.pi * (tail_biases_v - final_bias_v) / psi.vpi_v),
                            dtype=float,
                        )
                        tail_e_ctl = np.asarray(wrap_rad(tail_phi_ref - tgt_rad), dtype=float)

                        session.append_csv(
                            "c4_traces.csv",
                            trace_csv_fields,
                            [
                                {
                                    "session": s_idx,
                                    "method": m_name,
                                    "target_thesis_deg": tgt_deg,
                                    "target_fw_deg": tgt_fw_deg,
                                    "seed_rad": seed_rad,
                                    "step_idx": h.step_idx,
                                    "bias_mv": h.bias_mv,
                                    "phi_hat_thesis_rad": h.phi_hat_thesis_rad,
                                    "phi_hat_fw_rad": h.phi_hat_fw_rad,
                                    "err_est_rad": h.err_est_rad,
                                    "radius": h.radius,
                                    "dc_mv": h.dc_mv,
                                    "drops": h.drops,
                                    "crc": h.crc,
                                }
                                for h in hist
                            ],
                        )
                        psi_rec = {
                            "session": s_idx,
                            "method": m_name,
                            "target_thesis_deg": tgt_deg,
                            "target_fw_deg": tgt_fw_deg,
                            "seed_rad": seed_rad,
                            "final_bias_mv": final_bias_v * 1e3,
                            "phi_true_thesis_deg": psi.phi_true_thesis_deg,
                            "phi_true_fw_deg": psi.phi_true_fw_deg,
                            "e_ctl_deg": float(np.degrees(e_ctl_rad)),
                            "e_est_deg": float(np.degrees(e_est_rad)),
                            "vpi_psi_v": psi.vpi_v,
                            "fit_rmse_mv": psi.fit_rmse_mv,
                            "sigma_phi_deg": psi.sigma_phi_deg,
                            "drift_mv": psi.drift_mv,
                            "tail_e_ctl_rad": tail_e_ctl.tolist(),
                        }
                        session.append_csv(
                            "c4_dc_psi_anchors.csv",
                            psi_csv_fields,
                            [{k: v for k, v in psi_rec.items() if k != "tail_e_ctl_rad"}],
                        )
                        session.mark_done(ep_key, psi_rec)

                    all_method_episodes[m_name].append(session.get_data(ep_key))

    # --- 3. Onboard autonomous firmware spot-check at QUAD (90 deg fw -> -90 deg thesis) ---
    if not session.is_done("c4_onboard_spot_check"):
        uart.stop()
        uart.target("quad")
        uart.telemetry(True)
        uart.start()
        tel_rows = uart.read_telemetry_rows(2.0, max_rows=20)
        uart.telemetry(False)
        uart.stop()
        if tel_rows:
            last_tel = tel_rows[-1]
            psi_spot = measure_dc_psi(uart, last_tel.bias_mv, cal_init.vpi_v * 1e3, pre_wait_blocks=12)
            spot_e_ctl_deg = float(np.degrees(wrap_rad(psi_spot.phi_true_fw_rad - np.radians(90.0))))
        else:
            psi_spot = None
            spot_e_ctl_deg = float("nan")
        uart.stop()
        session.mark_done(
            "c4_onboard_spot_check",
            {
                "n_tel_rows": len(tel_rows),
                "final_bias_mv": tel_rows[-1].bias_mv if tel_rows else None,
                "phi_true_thesis_deg": psi_spot.phi_true_thesis_deg if psi_spot else None,
                "phi_true_fw_deg": psi_spot.phi_true_fw_deg if psi_spot else None,
                "e_ctl_deg": spot_e_ctl_deg,
            },
        )

    uart.stop()

    method_summaries: dict[str, Any] = {}
    for m_name, eps in all_method_episodes.items():
        # Group pooled tail errors per target across sessions/seeds, and also evaluate all episodes.
        by_target: dict[float, list[float]] = {t: [] for t in targets_deg}
        for ep in eps:
            by_target[ep["target_thesis_deg"]].extend(ep["tail_e_ctl_rad"])
        summary_16 = summarize_target_runs([by_target[t] for t in DEFAULT_16_TARGETS_DEG])
        summary_all = summarize_target_runs([by_target[t] for t in targets_deg])
        method_summaries[m_name] = {
            "equidistant_16": summary_16,
            "all_targets": summary_all,
            "n_episodes": len(eps),
        }

    out = {
        "gain_sweep": session.get_data("c4_gain_sweep"),
        "method_summaries": method_summaries,
        "onboard_spot_check": session.get_data("c4_onboard_spot_check"),
    }
    session.write_json("summary.json", out)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Experiment C4 16-Target 5-Method Comparison")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--sessions", type=int, default=3)
    parser.add_argument("--t-wait-blocks", type=int, default=DEFAULT_WAIT_BLOCKS)
    parser.add_argument("--pilot-mv", type=float, default=DEFAULT_PILOT_MV)
    parser.add_argument("--gain", type=float, default=0.25)
    parser.add_argument("--n-steps", type=int, default=60)
    parser.add_argument("--tail-steps", type=int, default=20)
    parser.add_argument("--cal-points", type=int, default=61)
    parser.add_argument("--methods", default=",".join(DEFAULT_METHODS))
    parser.add_argument("--no-dense", action="store_true")
    parser.add_argument("--skip-gain-sweep", action="store_true")
    args = parser.parse_args(argv)

    session = RunSession("c4", run_dir=args.run_dir, resume=args.resume)
    session.write_metadata(capture_metadata("c4", dry_run=args.dry_run))
    methods = tuple(m.strip() for m in args.methods.split(",") if m.strip())
    with _build_uart(args) as uart:
        run_c4(
            uart,
            session,
            sessions=args.sessions,
            t_wait_blocks=args.t_wait_blocks,
            pilot_mv=args.pilot_mv,
            gain=args.gain,
            n_steps=args.n_steps,
            tail_steps=args.tail_steps,
            methods=methods,
            include_dense=not args.no_dense,
            skip_gain_sweep=args.skip_gain_sweep,
            cal_points=args.cal_points,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
