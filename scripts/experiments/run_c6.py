"""Experiment C6: 20/25 MHz RF loading robustness (Off/0/3/6/9/13 dBm) & J0 zero-crossing freeze."""

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
    thesis_to_fw_rad,
    wrap_rad,
)
from .dc_psi import compute_scan_window, measure_dc_psi
from .dg922_client import DEFAULT_HOST as DEFAULT_DG922_HOST
from .dg922_client import DG922Client
from .metrics import cond_kappa_and_sigma_min, summarize_target_runs
from .sim_plant import MZMPlantConfig, SimulatedDG922Socket, SimulatedMZMPlant, SimulatedUARTTransport
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient

DEFAULT_RF_TARGETS_DEG = tuple(np.arange(0.0, 360.0, 45.0).tolist())
DEFAULT_RF_POWERS_DBM = (0.0, 3.0, 6.0, 9.0, 13.0)
DEFAULT_C6_METHODS = ("B2", "B3", "B4", "AffineFrozen", "AffineRecal", "AffineS3Plus")


def _build_clients(args: argparse.Namespace) -> tuple[UARTClient, DG922Client]:
    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261129, vpi_drift_rel=0.005))
        uart = UARTClient(transport=SimulatedUARTTransport(plant))
        dg = DG922Client(
            host=args.dg922_host,
            cmd_settle_s=0.0,
            socket_factory=lambda *a, **kw: SimulatedDG922Socket(plant),
        )
        return uart, dg
    if not args.confirm_rf:
        raise PermissionError(
            "Physical DG922 Pro RF output requires explicit --confirm-rf confirmation (or use --dry-run)."
        )
    return UARTClient(port=args.port), DG922Client(host=args.dg922_host)


def _calibrate_state(
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


def run_c6(
    uart: UARTClient,
    dg: DG922Client,
    session: RunSession,
    *,
    rf_freq_hz: float = 20e6,
    rf_powers_dbm: tuple[float, ...] = DEFAULT_RF_POWERS_DBM,
    sessions: int = 3,
    t_wait_blocks: int = DEFAULT_WAIT_BLOCKS,
    pilot_mv: float = DEFAULT_PILOT_MV,
    gain: float = 0.25,
    n_steps: int = 35,
    tail_steps: int = 15,
    cal_points: int = 49,
    methods: tuple[str, ...] = DEFAULT_C6_METHODS,
    targets_deg: tuple[float, ...] = DEFAULT_RF_TARGETS_DEG,
    run_j0_sweep: bool = False,
    j0_powers_dbm: tuple[float, ...] = (13.0, 14.5, 15.5, 16.2, 16.6, 17.2),
    vpi_est_mv: float = 5382.0,
    v_lock_mv: float = -340.0,
) -> dict[str, Any]:
    """Execute C6 RF loading benchmark across power states and optional J0 null freeze sweep."""
    if tail_steps > n_steps:
        raise ValueError("tail_steps cannot exceed n_steps")

    # Baseline calibration with RF explicitly OFF.
    dg.output_off(channel=1, verify=True)
    cal_off = _calibrate_state(uart, v_lock_mv, vpi_est_mv, pilot_mv, t_wait_blocks, cal_points)
    v_quad_mv = cal_off.target_voltage_v(0.5 * np.pi) * 1e3
    h1_ref_rows = uart.meas(v_quad_mv, pilot_mv, max(t_wait_blocks, 12), 4)
    h1_off_amp_v = float(abs(np.mean([r.h1_complex_v for r in h1_ref_rows])))

    rf_states: list[tuple[str, float | None]] = [("Off", None)]
    for p_dbm in rf_powers_dbm:
        rf_states.append((f"{p_dbm:g}dBm", float(p_dbm)))
    rf_states.append(("Off_return", None))

    state_results: dict[str, Any] = {}
    csv_rows: list[dict[str, Any]] = []

    try:
        for state_label, p_dbm in rf_states:
            if p_dbm is None:
                dg.output_off(channel=1, verify=True)
            else:
                dg.configure_rf_dbm(rf_freq_hz, p_dbm, channel=1)
                dg.output_on(channel=1)

            state_key = f"state_{state_label}"
            if not session.is_done(state_key):
                h1_probe_rows = uart.meas(v_quad_mv, pilot_mv, max(t_wait_blocks, 12), 4)
                h1_state_amp_v = float(abs(np.mean([r.h1_complex_v for r in h1_probe_rows])))
                h1_rel = h1_state_amp_v / max(h1_off_amp_v, 1e-9)

                cal_state = cal_off if p_dbm is None else _calibrate_state(uart, v_lock_mv, vpi_est_mv, pilot_mv, t_wait_blocks, cal_points)
                kappa_s, sigma_min_s = cond_kappa_and_sigma_min(cal_state.A_hat)

                method_tails: dict[str, dict[float, list[float]]] = {m: {t: [] for t in targets_deg} for m in methods}
                method_failed: dict[str, dict[float, list[bool]]] = {m: {t: [] for t in targets_deg} for m in methods}

                for s_idx in range(sessions):
                    for m_name in methods:
                        cal_use = cal_state if m_name == "AffineRecal" else cal_off
                        for tgt_deg in targets_deg:
                            tgt_rad = float(wrap_rad(np.radians(tgt_deg)))
                            v_seed = float(np.clip(cal_use.target_voltage_v(tgt_rad) + 0.35 * (cal_use.vpi_v / np.pi), -9.8, 9.8))
                            ctrl = make_controller(
                                m_name,
                                cal_use,
                                tgt_rad,
                                gain=gain,
                                pilot_mv=pilot_mv,
                                wait_blocks=t_wait_blocks,
                            )
                            hist = run_closed_loop_episode(uart, ctrl, v_seed, n_steps)
                            final_v = hist[-1].next_bias_mv * 1e-3
                            psi = measure_dc_psi(uart, final_v * 1e3, cal_use.vpi_v * 1e3, pre_wait_blocks=12)
                            tail_biases_v = np.asarray([h.bias_mv * 1e-3 for h in hist[-tail_steps:]], dtype=float)
                            tail_phi_ref = np.asarray(
                                wrap_rad(psi.phi_true_thesis_rad + np.pi * (tail_biases_v - final_v) / psi.vpi_v),
                                dtype=float,
                            )
                            tail_e_ctl = np.asarray(wrap_rad(tail_phi_ref - tgt_rad), dtype=float)
                            method_tails[m_name][tgt_deg].extend(tail_e_ctl.tolist())
                            method_failed[m_name][tgt_deg].append(bool(hist[-1].frozen))

                            csv_rows.append(
                                {
                                    "rf_state": state_label,
                                    "rf_power_dbm": "" if p_dbm is None else p_dbm,
                                    "session": s_idx,
                                    "method": m_name,
                                    "target_thesis_deg": tgt_deg,
                                    "target_fw_deg": float(np.degrees(thesis_to_fw_rad(tgt_rad))),
                                    "final_bias_mv": final_v * 1e3,
                                    "phi_true_thesis_deg": psi.phi_true_thesis_deg,
                                    "phi_true_fw_deg": psi.phi_true_fw_deg,
                                    "e_ctl_deg": float(np.degrees(wrap_rad(psi.phi_true_thesis_rad - tgt_rad))),
                                    "sigma_phi_deg": psi.sigma_phi_deg,
                                    "frozen_last": int(hist[-1].frozen),
                                    "k_axis_last": hist[-1].k_axis,
                                }
                            )

                if csv_rows:
                    session.append_csv("c6_rf_runs.csv", list(csv_rows[0].keys()), csv_rows)
                    csv_rows.clear()

                per_method_summary = {}
                for m_name in methods:
                    tails_list = [method_tails[m_name][t] for t in targets_deg]
                    fail_list = [any(method_failed[m_name][t]) for t in targets_deg]
                    per_method_summary[m_name] = summarize_target_runs(tails_list, failed_mask=fail_list)

                session.mark_done(
                    state_key,
                    {
                        "rf_state": state_label,
                        "rf_power_dbm": p_dbm,
                        "h1_rel_amp": h1_rel,
                        "kappa": kappa_s,
                        "sigma_min_mv": sigma_min_s * 1e3,
                        "methods": per_method_summary,
                    },
                )
            state_results[state_label] = session.get_data(state_key)

        # Optional J0(m_RF)=0 null-crossing freeze verification.
        j0_records: list[dict[str, Any]] = []
        if run_j0_sweep and not session.is_done("j0_null_freeze"):
            tgt_rad = float(np.deg2rad(45.0))
            ctrl = make_controller("AffineS3Plus", cal_off, tgt_rad, gain=gain, pilot_mv=pilot_mv, wait_blocks=t_wait_blocks)
            cur_v = cal_off.target_voltage_v(tgt_rad)
            for p_dbm in j0_powers_dbm:
                dg.configure_rf_dbm(rf_freq_hz, float(p_dbm), channel=1)
                dg.output_on(channel=1)
                hist = run_closed_loop_episode(uart, ctrl, cur_v, 10)
                cur_v = hist[-1].next_bias_mv * 1e-3
                j0_records.append(
                    {
                        "power_dbm": float(p_dbm),
                        "min_radius": float(min(h.radius for h in hist)),
                        "frozen_steps": int(sum(1 for h in hist if h.frozen)),
                        "start_bias_mv": hist[0].bias_mv,
                        "end_bias_mv": hist[-1].next_bias_mv,
                        "bias_delta_mv": hist[-1].next_bias_mv - hist[0].bias_mv,
                    }
                )
            dg.output_off(channel=1, verify=True)
            session.mark_done("j0_null_freeze", j0_records)
    finally:
        dg.output_off(channel=1, verify=False)
        uart.stop()

    summary = {
        "rf_freq_hz": rf_freq_hz,
        "states": state_results,
        "j0_null_freeze": session.get_data("j0_null_freeze"),
        "weak_light_10db_status": "待 VOA 补测",
    }
    session.write_json("summary.json", summary)
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Experiment C6 RF Loading & J0 Null Freeze")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--dg922-host", default=DEFAULT_DG922_HOST)
    parser.add_argument("--rf-freq-hz", type=float, default=20e6)
    parser.add_argument("--rf-powers-dbm", default="0,3,6,9,13")
    parser.add_argument("--confirm-rf", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--sessions", type=int, default=3)
    parser.add_argument("--t-wait-blocks", type=int, default=DEFAULT_WAIT_BLOCKS)
    parser.add_argument("--pilot-mv", type=float, default=DEFAULT_PILOT_MV)
    parser.add_argument("--gain", type=float, default=0.25)
    parser.add_argument("--n-steps", type=int, default=35)
    parser.add_argument("--tail-steps", type=int, default=15)
    parser.add_argument("--cal-points", type=int, default=49)
    parser.add_argument("--methods", default=",".join(DEFAULT_C6_METHODS))
    parser.add_argument("--run-j0-sweep", action="store_true")
    parser.add_argument("--j0-powers-dbm", default="13,14.5,15.5,16.2,16.6,17.2")
    args = parser.parse_args(argv)

    powers = tuple(float(x.strip()) for x in args.rf_powers_dbm.split(",") if x.strip())
    j0_powers = tuple(float(x.strip()) for x in args.j0_powers_dbm.split(",") if x.strip())
    methods = tuple(m.strip() for m in args.methods.split(",") if m.strip())

    session = RunSession("c6", run_dir=args.run_dir, resume=args.resume)
    session.write_metadata(capture_metadata("c6", rf_freq_hz=args.rf_freq_hz, dry_run=args.dry_run))
    uart, dg = _build_clients(args)
    with uart, dg:
        run_c6(
            uart,
            dg,
            session,
            rf_freq_hz=args.rf_freq_hz,
            rf_powers_dbm=powers,
            sessions=args.sessions,
            t_wait_blocks=args.t_wait_blocks,
            pilot_mv=args.pilot_mv,
            gain=args.gain,
            n_steps=args.n_steps,
            tail_steps=args.tail_steps,
            cal_points=args.cal_points,
            methods=methods,
            run_j0_sweep=args.run_j0_sweep,
            j0_powers_dbm=j0_powers,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
