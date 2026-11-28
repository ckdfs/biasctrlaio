"""Experiment C5: Long-term autonomous 3h stability, step settling, and disturbance recovery."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from pathlib import Path
import time
from typing import Any

import numpy as np

from .algorithms import AffineCalModel, AffineRecal, calibrate_affine_from_sweep, run_closed_loop_episode
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
from .dg922_client import DEFAULT_HOST as DEFAULT_DG922_HOST
from .dg922_client import DG922Client
from .metrics import (
    DEFAULT_BAND_1DEG_RAD,
    DEFAULT_BAND_20MRAD,
    p95_abs,
    settling_time_steps,
    temporal_rms,
    unwrapped_p2p_and_slope,
)
from .sim_plant import (
    MZMPlantConfig,
    SimulatedDG922Socket,
    SimulatedMZMPlant,
    SimulatedUARTTransport,
)
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient


def _build_clients(args: argparse.Namespace) -> tuple[UARTClient, DG922Client | None, SimulatedMZMPlant | None]:
    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261130))
        uart = UARTClient(transport=SimulatedUARTTransport(plant))
        dg = DG922Client(
            host=args.dg922_host,
            cmd_settle_s=0.0,
            socket_factory=lambda *a, **kw: SimulatedDG922Socket(plant),
        )
        return uart, dg, plant
    if args.disturbance_trials > 0 and not args.confirm_rf:
        raise PermissionError(
            "Physical C5 disturbance recovery uses DG922 Pro RF; pass --confirm-rf (or --disturbance-trials 0 / --dry-run)."
        )
    dg = DG922Client(host=args.dg922_host) if args.confirm_rf else None
    return UARTClient(port=args.port), dg, None


def _calibrate_quick(
    uart: UARTClient,
    v_lock_mv: float,
    vpi_est_mv: float,
    pilot_mv: float,
    t_wait_blocks: int,
    n_cal_points: int = 49,
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


def _set_firmware_target_thesis(uart: UARTClient, target_thesis_deg: float) -> float:
    tgt_fw_deg = float(np.degrees(thesis_to_fw_rad(np.radians(target_thesis_deg))))
    if abs(tgt_fw_deg - 90.0) < 1e-3:
        uart.target("quad")
    elif abs(tgt_fw_deg) < 1e-3:
        uart.target("min")
    elif abs(abs(tgt_fw_deg) - 180.0) < 1e-3:
        uart.target("max")
    else:
        uart.target("custom", tgt_fw_deg)
    return tgt_fw_deg


def run_c5(
    uart: UARTClient,
    session: RunSession,
    *,
    dry_run: bool = False,
    dg: DG922Client | None = None,
    plant: SimulatedMZMPlant | None = None,
    targets_thesis_deg: tuple[float, ...] = (90.0, 45.0),
    runs_per_target: int = 3,
    duration_s: float = 10800.0,
    anchor_interval_s: float = 900.0,
    step_trials: int = 10,
    disturbance_trials: int = 5,
    disturbance_rf_freq_hz: float = 20e6,
    disturbance_rf_dbm: float = 13.0,
    t_wait_blocks: int = DEFAULT_WAIT_BLOCKS,
    pilot_mv: float = DEFAULT_PILOT_MV,
    gain: float = 0.25,
    vpi_est_mv: float = 5382.0,
    v_lock_mv: float = -340.0,
) -> dict[str, Any]:
    """Execute C5 autonomous long-term stability, step settling, and recalibration recovery."""
    if dg is not None:
        # Ensure RF is OFF before calibrating/long-term runs without keeping TCP open idle for hours.
        with (nullcontext(dg) if dg._sock is not None else dg):
            dg.output_off(channel=1, verify=True)

    cal_base = _calibrate_quick(uart, v_lock_mv, vpi_est_mv, pilot_mv, t_wait_blocks)
    # Exit EXP_HOLD -> IDLE before ctrl cfg and onboard calibration checks.
    uart.stop()

    if not dry_run:
        st_cal = uart.status(fail_on_fault=True)
        if not st_cal.calibrated:
            uart.cal_bias()
            uart.save()

    # Configure onboard loop timing parameters on Phase 7 firmware.
    for cfg_key, cfg_val in (("pilot", pilot_mv), ("wait", t_wait_blocks), ("gain", gain)):
        uart.ctrl_cfg(cfg_key, cfg_val)

    # --- 1. Long-term autonomous closed-loop runs (default 3 x 3 h at 90 deg and 45 deg) ---
    long_term_summary: dict[str, list[dict[str, Any]]] = {}
    for tgt_thesis_deg in targets_thesis_deg:
        tgt_label = f"thesis_{tgt_thesis_deg:g}deg"
        long_term_summary[tgt_label] = []
        tgt_rad = float(wrap_rad(np.radians(tgt_thesis_deg)))

        for run_idx in range(runs_per_target):
            key = f"longterm_{tgt_label}_run{run_idx}"
            if not session.is_done(key):
                tgt_fw_deg = _set_firmware_target_thesis(uart, tgt_thesis_deg)
                tel_csv = f"c5_tel_{tgt_label}_run{run_idx}.csv"
                psi_csv = f"c5_psi_{tgt_label}_run{run_idx}.csv"

                tel_fields = [
                    "elapsed_s",
                    "k",
                    "t_ms",
                    "bias_mv",
                    "phi_hat_thesis_rad",
                    "phi_hat_fw_rad",
                    "err_rad",
                    "radius",
                    "dc_mv",
                    "locked",
                    "frozen",
                    "recal_count",
                ]
                psi_fields = [
                    "elapsed_s",
                    "v_lock_mv",
                    "phi_true_thesis_deg",
                    "phi_true_fw_deg",
                    "e_ctl_deg",
                    "sigma_phi_deg",
                    "drift_mv",
                ]

                t_start = time.monotonic()
                n_tel_samples = 0
                tel_t0_ms: int | None = None
                tel_base_elapsed = 0.0
                prev_t_ms: int | None = None
                prev_row_elapsed = -1e-9
                steady_tel_t: list[float] = []
                steady_tel_bias_mv: list[float] = []
                steady_tel_err: list[float] = []
                all_psi_t, all_psi_e_ctl = [], []
                recal_count_total = 0

                while True:
                    elapsed = time.monotonic() - t_start
                    if elapsed >= duration_s and len(all_psi_t) > 0:
                        break
                    seg_dur = min(anchor_interval_s, max(duration_s - elapsed, 0.05))
                    seg_t0 = time.monotonic() - t_start
                    uart.telemetry(True)
                    uart.start()
                    rows = uart.read_telemetry_rows(
                        seg_dur,
                        max_rows=30 if dry_run else None,
                    )
                    uart.telemetry(False)
                    uart.stop()
                    if uart.telemetry_backlog:
                        rows.extend(uart.telemetry_backlog)
                        uart.telemetry_backlog.clear()

                    cur_elapsed = time.monotonic() - t_start
                    if rows:
                        if tel_t0_ms is None:
                            tel_t0_ms = rows[0].t_ms
                            tel_base_elapsed = seg_t0
                        recal_count_total += max(r.recal_count for r in rows)
                        n_tel_samples += len(rows)
                        csv_batch = []
                        row_times: list[float] = []
                        for r in rows:
                            if prev_t_ms is not None and r.t_ms < prev_t_ms:
                                tel_base_elapsed = max(seg_t0, prev_row_elapsed + 0.020)
                                tel_t0_ms = r.t_ms
                            row_elapsed = tel_base_elapsed + (r.t_ms - tel_t0_ms) * 1e-3
                            prev_t_ms = r.t_ms
                            prev_row_elapsed = row_elapsed
                            row_times.append(row_elapsed)
                            csv_batch.append(
                                {
                                    "elapsed_s": row_elapsed,
                                    "k": r.k,
                                    "t_ms": r.t_ms,
                                    "bias_mv": r.bias_mv,
                                    "phi_hat_thesis_rad": r.phi_hat_thesis_rad,
                                    "phi_hat_fw_rad": r.phi_hat_fw_rad,
                                    "err_rad": r.err_rad,
                                    "radius": r.radius,
                                    "dc_mv": r.dc_mv,
                                    "locked": int(r.locked),
                                    "frozen": int(r.frozen),
                                    "recal_count": r.recal_count,
                                }
                            )
                        session.append_csv(tel_csv, tel_fields, csv_batch)
                        skip = min(25, len(rows) // 4) if len(rows) > 40 else 0
                        steady_tel_t.extend(row_times[skip:])
                        steady_tel_bias_mv.extend(r.bias_mv for r in rows[skip:])
                        steady_tel_err.extend(r.err_rad for r in rows[skip:])
                        anchor_v_mv = rows[-1].bias_mv
                    else:
                        anchor_v_mv = cal_base.target_voltage_v(tgt_rad) * 1e3

                    psi = measure_dc_psi(uart, anchor_v_mv, cal_base.vpi_v * 1e3, pre_wait_blocks=12)
                    uart.stop()
                    e_ctl = float(wrap_rad(psi.phi_true_thesis_rad - tgt_rad))
                    all_psi_t.append(cur_elapsed)
                    all_psi_e_ctl.append(e_ctl)
                    session.append_csv(
                        psi_csv,
                        psi_fields,
                        [
                            {
                                "elapsed_s": cur_elapsed,
                                "v_lock_mv": anchor_v_mv,
                                "phi_true_thesis_deg": psi.phi_true_thesis_deg,
                                "phi_true_fw_deg": psi.phi_true_fw_deg,
                                "e_ctl_deg": float(np.degrees(e_ctl)),
                                "sigma_phi_deg": psi.sigma_phi_deg,
                                "drift_mv": psi.drift_mv,
                            }
                        ],
                    )
                    if time.monotonic() - t_start >= duration_s:
                        break

                psi_arr = np.asarray(all_psi_e_ctl, dtype=float)
                bias_stats = (
                    unwrapped_p2p_and_slope(steady_tel_t, steady_tel_bias_mv)
                    if steady_tel_t
                    else {"net_change": 0.0, "peak_to_peak": 0.0, "slope_per_hour": 0.0}
                )
                run_rec = {
                    "run_idx": run_idx,
                    "target_thesis_deg": tgt_thesis_deg,
                    "target_fw_deg": tgt_fw_deg,
                    "n_tel_samples": n_tel_samples,
                    "n_psi_anchors": len(all_psi_e_ctl),
                    "ext_time_rms_mrad": temporal_rms(psi_arr) * 1e3,
                    "ext_p95_deg": float(np.degrees(p95_abs(psi_arr))),
                    "ext_max_abs_deg": float(np.degrees(np.max(np.abs(psi_arr)))),
                    "int_loop_rms_mrad": temporal_rms(steady_tel_err) * 1e3 if steady_tel_err else float("nan"),
                    "bias_net_change_mv": bias_stats["net_change"],
                    "bias_p2p_mv": bias_stats["peak_to_peak"],
                    "bias_slope_mv_per_hr": bias_stats["slope_per_hour"],
                    "recal_count": recal_count_total,
                    "recal_recovery_status": "本次未触发" if recal_count_total == 0 else "triggered",
                }
                session.mark_done(key, run_rec)
            long_term_summary[tgt_label].append(session.get_data(key))

    # --- 2. Step settling: 5 targets x 10 forward/reverse trials ---
    if not session.is_done("c5_step_settling"):
        transitions = [(0.0, 90.0), (90.0, 180.0), (180.0, 270.0), (270.0, 45.0), (45.0, 0.0)]
        dt_update_s = (t_wait_blocks + 1) * 0.020
        settling_rows = []
        steps_1deg_all, steps_20mrad_all = [], []
        time_1deg_all, time_20mrad_all = [], []

        for t0_deg, t1_deg in transitions:
            for dir_name, (start_deg, end_deg) in (("fwd", (t0_deg, t1_deg)), ("rev", (t1_deg, t0_deg))):
                start_rad = float(np.radians(start_deg))
                end_rad = float(np.radians(end_deg))
                v_start_bias = cal_base.target_voltage_v(start_rad)
                for trial in range(step_trials):
                    ctrl = AffineRecal(cal_base, end_rad, gain=gain, pilot_mv=pilot_mv, wait_blocks=t_wait_blocks)
                    hist = run_closed_loop_episode(uart, ctrl, v_start_bias, 50)
                    errs = np.asarray([h.err_est_rad for h in hist], dtype=float)
                    s_1d, t_1d = settling_time_steps(errs, threshold_rad=DEFAULT_BAND_1DEG_RAD, sustain_steps=10, dt_s=dt_update_s)
                    s_20m, t_20m = settling_time_steps(errs, threshold_rad=DEFAULT_BAND_20MRAD, sustain_steps=10, dt_s=dt_update_s)
                    if s_1d is not None and t_1d is not None:
                        steps_1deg_all.append(s_1d)
                        time_1deg_all.append(t_1d)
                    if s_20m is not None and t_20m is not None:
                        steps_20mrad_all.append(s_20m)
                        time_20mrad_all.append(t_20m)
                    settling_rows.append(
                        {
                            "start_thesis_deg": start_deg,
                            "end_thesis_deg": end_deg,
                            "direction": dir_name,
                            "trial": trial,
                            "steps_1deg": s_1d,
                            "time_1deg_s": t_1d,
                            "steps_20mrad": s_20m,
                            "time_20mrad_s": t_20m,
                        }
                    )
        session.append_csv("c5_step_settling.csv", list(settling_rows[0].keys()), settling_rows)
        session.mark_done(
            "c5_step_settling",
            {
                "dt_update_s": dt_update_s,
                "n_trials_total": len(settling_rows),
                "settled_1deg_count": len(steps_1deg_all),
                "mean_steps_1deg": float(np.mean(steps_1deg_all)) if steps_1deg_all else None,
                "mean_time_1deg_s": float(np.mean(time_1deg_all)) if time_1deg_all else None,
                "settled_20mrad_count": len(steps_20mrad_all),
                "mean_steps_20mrad": float(np.mean(steps_20mrad_all)) if steps_20mrad_all else None,
                "mean_time_20mrad_s": float(np.mean(time_20mrad_all)) if time_20mrad_all else None,
            },
        )

    # --- 3. Disturbance-triggered recalibration recovery (5 trials) ---
    if not session.is_done("c5_disturbance_recovery"):
        rec_trials = []
        dt_update_s = (t_wait_blocks + 1) * 0.020
        tgt_rad = float(np.deg2rad(90.0))
        dg_ctx = nullcontext(dg) if (dg is None or dg._sock is not None) else dg
        with dg_ctx:
            for tr_idx in range(disturbance_trials):
                if dg is not None:
                    dg.output_off(channel=1, verify=True)
                ctrl = AffineRecal(
                    cal_base,
                    tgt_rad,
                    gain=gain,
                    pilot_mv=pilot_mv,
                    wait_blocks=t_wait_blocks,
                    rho_thresh=0.18,
                    hold_ticks=4,
                )
                cur_v = cal_base.target_voltage_v(tgt_rad)
                pre_hist = run_closed_loop_episode(uart, ctrl, cur_v, 8)
                cur_v = pre_hist[-1].next_bias_mv * 1e-3

                recal_step = None
                post_errs = []
                try:
                    if dg is not None:
                        dg.configure_rf_dbm(disturbance_rf_freq_hz, disturbance_rf_dbm, channel=1)
                        dg.output_on(channel=1)
                    elif plant is not None:
                        plant.cfg.optical_power_scale = 0.72
                    else:
                        raise RuntimeError("Real-hardware disturbance recovery requires DG922Client (--confirm-rf)")

                    for s in range(40):
                        obs = ctrl.step(uart, cur_v)
                        cur_v = obs.next_bias_mv * 1e-3
                        post_errs.append(obs.err_est_rad)
                        if obs.recal_triggered and recal_step is None:
                            recal_step = s
                            new_cal = _calibrate_quick(uart, v_lock_mv, vpi_est_mv, pilot_mv, t_wait_blocks, n_cal_points=33)
                            ctrl.recalibrate(new_cal)
                finally:
                    if dg is not None:
                        dg.output_off(channel=1, verify=False)
                    if plant is not None:
                        plant.cfg.optical_power_scale = 1.0

                psi_post = measure_dc_psi(uart, cur_v * 1e3, cal_base.vpi_v * 1e3, pre_wait_blocks=12)
                post_e_ctl_deg = float(np.degrees(wrap_rad(psi_post.phi_true_thesis_rad - tgt_rad)))

                s_rec, t_rec = settling_time_steps(
                    post_errs,
                    threshold_rad=DEFAULT_BAND_1DEG_RAD,
                    sustain_steps=10,
                    dt_s=dt_update_s,
                )
                rec_trials.append(
                    {
                        "trial": tr_idx,
                        "recal_triggered_step": recal_step,
                        "recovery_steps_1deg": s_rec,
                        "recovery_time_1deg_s": t_rec,
                        "post_recovery_e_ctl_deg": post_e_ctl_deg,
                    }
                )
        session.mark_done("c5_disturbance_recovery", rec_trials)

    uart.stop()
    summary = {
        "long_term": long_term_summary,
        "step_settling": session.get_data("c5_step_settling"),
        "disturbance_recovery": session.get_data("c5_disturbance_recovery"),
    }
    session.write_json("summary.json", summary)
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Experiment C5 Long-Term Stability & Recovery")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--dg922-host", default=DEFAULT_DG922_HOST)
    parser.add_argument("--confirm-rf", action="store_true")
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--duration-s", type=float, default=10800.0)
    parser.add_argument("--anchor-interval-s", type=float, default=900.0)
    parser.add_argument("--targets-thesis-deg", default="90,45")
    parser.add_argument("--step-trials", type=int, default=10)
    parser.add_argument("--disturbance-trials", type=int, default=5)
    parser.add_argument("--disturbance-rf-freq-hz", type=float, default=20e6)
    parser.add_argument("--disturbance-rf-dbm", type=float, default=13.0)
    parser.add_argument("--t-wait-blocks", type=int, default=DEFAULT_WAIT_BLOCKS)
    parser.add_argument("--pilot-mv", type=float, default=DEFAULT_PILOT_MV)
    parser.add_argument("--gain", type=float, default=0.25)
    args = parser.parse_args(argv)

    tgts = tuple(float(x.strip()) for x in args.targets_thesis_deg.split(",") if x.strip())
    session = RunSession("c5", run_dir=args.run_dir, resume=args.resume)
    session.write_metadata(capture_metadata("c5", dry_run=args.dry_run))
    uart, dg, plant = _build_clients(args)
    with uart:
        run_c5(
            uart,
            session,
            dry_run=args.dry_run,
            dg=dg,
            plant=plant,
            targets_thesis_deg=tgts,
            runs_per_target=args.runs,
            duration_s=args.duration_s,
            anchor_interval_s=args.anchor_interval_s,
            step_trials=args.step_trials,
            disturbance_trials=args.disturbance_trials,
            disturbance_rf_freq_hz=args.disturbance_rf_freq_hz,
            disturbance_rf_dbm=args.disturbance_rf_dbm,
            t_wait_blocks=args.t_wait_blocks,
            pilot_mv=args.pilot_mv,
            gain=args.gain,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
