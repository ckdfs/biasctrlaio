"""Experiment C1 external DMM verification: -10 to +10 V DC linearity and single-code step DNL."""

from __future__ import annotations

import argparse
from collections import deque
import csv
import math
import socket
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .common import RunSession, atomic_write_json, capture_metadata, repo_root
from .dm858e_client import DEFAULT_HOST as DEFAULT_DMM_HOST, DEFAULT_PORT as DEFAULT_DMM_PORT, DM858EClient
from .sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient

DAC_FULL_SCALE_COUNTS = 65535
NOMINAL_MIN_V = -10.0
NOMINAL_MAX_V = 10.0
NOMINAL_GAIN_VS_5V = 4.0
NOMINAL_LSB_UV = (NOMINAL_MAX_V - NOMINAL_MIN_V) * 1e6 / DAC_FULL_SCALE_COUNTS
THESIS_NOMINAL_VPI_V = 5.2776
DEFAULT_PHASE7_VPI_V = 5.378


def code_to_cmd_mv(code: int) -> float:
    """Return exact 3-decimal mV command string rounded float mapping uniquely 1-to-1 to DAC code."""
    if not (0 <= code <= DAC_FULL_SCALE_COUNTS):
        raise ValueError(f"DAC code out of [0, 65535]: {code}")
    mv = -10000.0 + (code / float(DAC_FULL_SCALE_COUNTS)) * 20000.0
    mv_clamped = min(max(round(mv, 3), -10000.0), 10000.0)
    return float(mv_clamped)


def default_full_sweep_codes() -> list[int]:
    """Return 129 codes covering every 512-code boundary [0, 512, ..., 65024, 65535]."""
    return [i * 512 for i in range(128)] + [DAC_FULL_SCALE_COUNTS]


def default_single_code_window() -> list[int]:
    """Return 65 consecutive codes centered on the 32767 -> 32768 MSB carry."""
    return list(range(32736, 32801))


def compute_linearity_metrics(
    rows: list[dict[str, Any]],
    *,
    vpi_v: float = DEFAULT_PHASE7_VPI_V,
) -> dict[str, Any]:
    """Compute bidirectional DC transfer slope, gain, zero offset, INL, and hysteresis."""
    if not rows:
        raise ValueError("Cannot compute DC linearity metrics on empty sweep")

    codes = np.asarray([float(r["code"]) for r in rows], dtype=float)
    meas_v = np.asarray([float(r["dmm_v"]) for r in rows], dtype=float)
    cmd_v = np.asarray([float(r["cmd_mv"]) * 1e-3 for r in rows], dtype=float)
    if np.any(~np.isfinite(meas_v)):
        raise ValueError("Non-finite DMM voltage in sweep rows")

    # Best-fit straight line V_meas = slope * code + intercept
    slope, intercept = np.polyfit(codes, meas_v, 1)
    fit_v = slope * codes + intercept
    inl_v = meas_v - fit_v
    lsb_v = float(slope)
    lsb_uv = lsb_v * 1e6

    fwd_rows = [r for r in rows if r["direction"] == "fwd"]
    rev_rows = [r for r in rows if r["direction"] == "rev"]
    fwd_codes = np.asarray([float(r["code"]) for r in fwd_rows], dtype=float)
    fwd_meas = np.asarray([float(r["dmm_v"]) for r in fwd_rows], dtype=float)
    rev_codes = np.asarray([float(r["code"]) for r in rev_rows], dtype=float)
    rev_meas = np.asarray([float(r["dmm_v"]) for r in rev_rows], dtype=float)

    slope_fwd, _ = np.polyfit(fwd_codes, fwd_meas, 1) if len(fwd_codes) >= 2 else (slope, intercept)
    slope_rev, _ = np.polyfit(rev_codes, rev_meas, 1) if len(rev_codes) >= 2 else (slope, intercept)

    # Endpoint line using pooled mean at min code and max code
    c_min = float(np.min(codes))
    c_max = float(np.max(codes))
    v_at_min = float(np.mean(meas_v[codes == c_min]))
    v_at_max = float(np.mean(meas_v[codes == c_max]))
    endpt_slope = (v_at_max - v_at_min) / (c_max - c_min)
    endpt_fit_v = v_at_min + endpt_slope * (codes - c_min)
    endpt_inl_v = meas_v - endpt_fit_v

    # Forward minus reverse hysteresis at matched codes
    fwd_by_code = {int(r["code"]): float(r["dmm_v"]) for r in fwd_rows}
    rev_by_code = {int(r["code"]): float(r["dmm_v"]) for r in rev_rows}
    matched_codes = sorted(set(fwd_by_code) & set(rev_by_code))
    if not matched_codes:
        raise ValueError("No matched codes between fwd and rev sweeps")
    hyst_uv = np.asarray([(fwd_by_code[c] - rev_by_code[c]) * 1e6 for c in matched_codes], dtype=float)

    v_span_v = v_at_max - v_at_min
    code_span_ratio = (c_max - c_min) / float(DAC_FULL_SCALE_COUNTS)
    full_span_v = float(slope * DAC_FULL_SCALE_COUNTS)
    full_range_nominal_v = NOMINAL_MAX_V - NOMINAL_MIN_V

    c32768_vals = meas_v[codes == 32768.0]
    midscale_32768_mv = float(np.mean(c32768_vals) * 1e3) if len(c32768_vals) else float((slope * 32768.0 + intercept) * 1e3)
    zero_cmd_offset_mv = float((slope * 32767.5 + intercept) * 1e3)
    max_abs_inl_mv = float(np.max(np.abs(inl_v)) * 1e3)
    rms_inl_mv = float(np.sqrt(np.mean(inl_v ** 2)) * 1e3)
    cmd_err_mv = (meas_v - cmd_v) * 1e3

    return {
        "n_rows": len(rows),
        "code_min": int(c_min),
        "code_max": int(c_max),
        "v_min_v": v_at_min,
        "v_max_v": v_at_max,
        "measured_span_v": float(v_span_v),
        "nominal_span_v": float(full_range_nominal_v * code_span_ratio),
        "dc_gain_vs_5v_endpoint": float(v_at_max - v_at_min) / (5.0 * code_span_ratio),
        "dc_gain_vs_5v_fit": float(full_span_v / 5.0),
        "gain_error_pct": float((full_span_v / 5.0 - NOMINAL_GAIN_VS_5V) / NOMINAL_GAIN_VS_5V * 100.0),
        "lsb_uv": lsb_uv,
        "lsb_fwd_uv": float(slope_fwd * 1e6),
        "lsb_rev_uv": float(slope_rev * 1e6),
        "nominal_lsb_uv": float(NOMINAL_LSB_UV),
        "phase_step_mrad_onboard_vpi": float((lsb_v * math.pi / vpi_v) * 1000.0),
        "phase_step_mrad_thesis_vpi": float((lsb_v * math.pi / THESIS_NOMINAL_VPI_V) * 1000.0),
        "midscale_code_32768_mv": midscale_32768_mv,
        "zero_cmd_offset_mv": zero_cmd_offset_mv,
        "max_abs_cmd_error_mv": float(np.max(np.abs(cmd_err_mv))),
        "rms_cmd_error_mv": float(np.sqrt(np.mean(cmd_err_mv ** 2))),
        "best_fit_inl_max_abs_mv": max_abs_inl_mv,
        "best_fit_inl_rms_mv": rms_inl_mv,
        "best_fit_inl_max_abs_lsb": float(max_abs_inl_mv / (lsb_uv * 1e-3)),
        "best_fit_inl_rms_lsb": float(rms_inl_mv / (lsb_uv * 1e-3)),
        "best_fit_inl_max_abs_ppm_fs": float(max_abs_inl_mv * 1e-3 / full_span_v * 1e6),
        "best_fit_inl_max_abs_mrad": float((max_abs_inl_mv * 1e-3) * math.pi / vpi_v * 1000.0),
        "endpoint_inl_max_abs_mv": float(np.max(np.abs(endpt_inl_v)) * 1e3),
        "endpoint_inl_max_abs_lsb": float(np.max(np.abs(endpt_inl_v)) / lsb_v),
        "hysteresis_max_abs_uv": float(np.max(np.abs(hyst_uv))),
        "hysteresis_rms_uv": float(np.sqrt(np.mean(hyst_uv ** 2))),
        "hysteresis_mean_signed_uv": float(np.mean(hyst_uv)),
        "best_fit_slope_v_per_code": float(slope),
        "best_fit_intercept_v": float(intercept),
    }


def compute_single_code_dnl_metrics(
    rows: list[dict[str, Any]],
    *,
    global_lsb_uv: float,
    vpi_v: float = DEFAULT_PHASE7_VPI_V,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Compute consecutive single-code steps and DNL (including 32767 -> 32768 MSB carry)."""
    if len(rows) < 2:
        raise ValueError("Need at least 2 consecutive codes to compute single-code steps")
    ordered = sorted(rows, key=lambda r: int(r["code"]))
    trans_rows: list[dict[str, Any]] = []
    dv_uv_list: list[float] = []
    for a, b in zip(ordered[:-1], ordered[1:]):
        c0 = int(a["code"])
        c1 = int(b["code"])
        if c1 != c0 + 1:
            raise ValueError(f"Non-consecutive codes in single-code sweep: {c0} -> {c1}")
        dv_uv = (float(b["dmm_v"]) - float(a["dmm_v"])) * 1e6
        dv_uv_list.append(dv_uv)
        trans_rows.append(
            {
                "code_from": c0,
                "code_to": c1,
                "cmd_from_mv": float(a["cmd_mv"]),
                "cmd_to_mv": float(b["cmd_mv"]),
                "v_from_v": float(a["dmm_v"]),
                "v_to_v": float(b["dmm_v"]),
                "step_uv": dv_uv,
                "dnl_global_lsb": (dv_uv - global_lsb_uv) / global_lsb_uv,
            }
        )

    dv_arr = np.asarray(dv_uv_list, dtype=float)
    local_lsb_uv = float(np.mean(dv_arr))
    for item in trans_rows:
        item["dnl_local_lsb"] = float((item["step_uv"] - local_lsb_uv) / local_lsb_uv)

    dnl_global = (dv_arr - global_lsb_uv) / global_lsb_uv
    dnl_local = (dv_arr - local_lsb_uv) / local_lsb_uv
    msb = next((t for t in trans_rows if t["code_from"] == 32767 and t["code_to"] == 32768), None)

    summary = {
        "code_start": int(ordered[0]["code"]),
        "code_end": int(ordered[-1]["code"]),
        "n_transitions": len(trans_rows),
        "global_lsb_uv": float(global_lsb_uv),
        "step_mean_uv": local_lsb_uv,
        "step_std_uv": float(np.std(dv_arr, ddof=1)) if len(dv_arr) > 1 else 0.0,
        "step_min_uv": float(np.min(dv_arr)),
        "step_max_uv": float(np.max(dv_arr)),
        "step_phase_mean_mrad": float((local_lsb_uv * 1e-6) * math.pi / vpi_v * 1000.0),
        "monotonic": bool(np.all(dv_arr > 0.0)),
        "dnl_global_min_lsb": float(np.min(dnl_global)),
        "dnl_global_max_lsb": float(np.max(dnl_global)),
        "dnl_global_max_abs_lsb": float(np.max(np.abs(dnl_global))),
        "dnl_global_rms_lsb": float(np.sqrt(np.mean(dnl_global ** 2))),
        "dnl_local_min_lsb": float(np.min(dnl_local)),
        "dnl_local_max_lsb": float(np.max(dnl_local)),
        "dnl_local_max_abs_lsb": float(np.max(np.abs(dnl_local))),
        "msb_carry_transition": "32767->32768" if msb else None,
        "msb_carry_step_uv": float(msb["step_uv"]) if msb else None,
        "msb_carry_dnl_global_lsb": float(msb["dnl_global_lsb"]) if msb else None,
        "msb_carry_dnl_local_lsb": float(msb["dnl_local_lsb"]) if msb else None,
    }
    return summary, trans_rows


class SimulatedDM858ESocket:
    """Mock TCP socket modeling RIGOL DM858E SCPI queries coupled to SimulatedMZMPlant."""

    def __init__(self, plant: SimulatedMZMPlant, *, seed: int = 20261128) -> None:
        self.plant = plant
        self.rng = np.random.default_rng(seed)
        self.vrange = 1000.0
        self.nplc = 20.0
        self.errors: list[str] = []
        self._rx_buf: deque[bytes] = deque()
        self.closed = False

    def settimeout(self, timeout: float) -> None:
        self.timeout = timeout

    def connect(self, addr) -> None:
        self.addr = addr

    def close(self) -> None:
        self.closed = True

    def _sim_va_voltage(self) -> float:
        # Simulated DAC8568 + OPA4192 transfer: tiny gain error (+0.025%), +4.1 mV offset,
        # ~0.4 mV smooth curvature, and a small 5 uV step at MSB carry >= 32768.
        v_ideal = float(self.plant.bias_v)
        code = int(round(min(max((v_ideal + 10.0) / 20.0, 0.0), 1.0) * 65535.0))
        inl = 0.00035 * math.sin(math.pi * (code / 65535.0)) + (0.000005 if code >= 32768 else 0.0)
        noise = float(self.rng.normal(0.0, 4e-7 if self.nplc >= 15 else 2e-6))
        return 1.00025 * v_ideal + 0.0041 + inl + noise

    def _push(self, reply: str) -> None:
        self._rx_buf.append((reply + "\n").encode("ascii"))

    def sendall(self, data: bytes) -> None:
        cmd = data.decode("ascii").strip()
        upper = cmd.upper()
        if upper == "*CLS":
            self.errors.clear()
        elif upper == "*IDN?":
            self._push("Rigol Technologies,DM858E,DM8E_SIM,00.01.00.00.22")
        elif upper == ":FUNCTION?":
            self._push('"VOLT"')
        elif upper == ":SYSTEM:ERROR?":
            if self.errors:
                self._push(self.errors.pop(0))
            else:
                self._push('0,"No error"')
        elif upper == ":SENSE:VOLTAGE:DC:RANGE?":
            self._push(f"{self.vrange:.8E}")
        elif upper == ":SENSE:VOLTAGE:DC:NPLC?":
            self._push(f"{self.nplc:.8E}")
        elif upper.startswith("CONF:VOLT:DC"):
            arg = upper.replace("CONF:VOLT:DC", "").strip()
            self.vrange = 1000.0 if arg in ("", "AUTO") else float(arg)
        elif upper.startswith(":SENSE:VOLTAGE:DC:NPLC") or upper.startswith(":VOLTAGE:DC:NPLC"):
            val = float(cmd.split()[-1])
            self.nplc = 5.0 if val == 10.0 else val
        elif upper in ("READ?", "MEAS:VOLT:DC?", "FETCH?"):
            v = self._sim_va_voltage()
            # Realistic slight 10V range ceiling around 10.004 V to exercise overload switching.
            if self.vrange <= 0.11 and abs(v) > 0.12:
                self._push("9.90000000E+37")
            elif self.vrange <= 10.01 and abs(v) > 10.005:
                self._push("9.90000000E+37")
            else:
                self._push(f"{v:.11G}")
        else:
            self.errors.append(f'-113,"Undefined header: {cmd}"')

    def recv(self, nbytes: int) -> bytes:
        if not self._rx_buf:
            return b""
        return self._rx_buf.popleft()


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _read_dmm_with_range_guard(
    dmm: DM858EClient,
    nominal_vrange: float,
    fallback_vrange: float,
    nplc: float,
) -> tuple[float, float]:
    """Read DMM once; on range overload sentinel switch cleanly to fallback range and back."""
    val = dmm.read()
    used_range = nominal_vrange
    if math.isinf(val):
        used_range, _ = dmm.configure_dc_voltage(vrange=fallback_vrange, nplc=nplc)
        val = dmm.read_finite()
        dmm.configure_dc_voltage(vrange=nominal_vrange, nplc=nplc)
    elif not math.isfinite(val):
        raise RuntimeError(f"Non-finite DMM reading: {val}")
    return val, used_range


def generate_c1_dc_linearity_plot(summary: dict[str, Any], out_png: Path) -> Path:
    """Render 3-panel C1 DC linearity, best-fit INL, and single-code DNL figure."""
    sweep_rows = summary["full_sweep_rows"]
    trans_rows = summary["single_code_transitions"]
    fit_m = summary["full_range_metrics"]
    dnl_m = summary["single_code_metrics"]

    fwd = [r for r in sweep_rows if r["direction"] == "fwd"]
    rev = [r for r in sweep_rows if r["direction"] == "rev"]
    fwd_c = np.asarray([r["code"] for r in fwd], dtype=float)
    fwd_v = np.asarray([r["dmm_v"] for r in fwd], dtype=float)
    rev_c = np.asarray([r["code"] for r in rev], dtype=float)
    rev_v = np.asarray([r["dmm_v"] for r in rev], dtype=float)

    slope = fit_m["best_fit_slope_v_per_code"]
    intercept = fit_m["best_fit_intercept_v"]
    fwd_inl_mv = (fwd_v - (slope * fwd_c + intercept)) * 1e3
    rev_inl_mv = (rev_v - (slope * rev_c + intercept)) * 1e3

    t_codes = np.asarray([r["code_to"] for r in trans_rows], dtype=float)
    t_steps = np.asarray([r["step_uv"] for r in trans_rows], dtype=float)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0), constrained_layout=True)

    # Panel A: Full-range transfer curve
    axes[0].plot(fwd_c / 1e3, fwd_v, color="#1f4e78", lw=1.4, label="Forward (0 -> 65535)")
    axes[0].plot(rev_c / 1e3, rev_v, color="#d95f02", ls="--", lw=1.2, label="Reverse (65535 -> 0)")
    axes[0].set_xlabel("DAC8568 Code (x10^3)")
    axes[0].set_ylabel("VA Output Voltage (V)")
    axes[0].set_title(
        f"Full-Range Span {fit_m['measured_span_v']:.5f} V\n"
        f"(Gain={fit_m['dc_gain_vs_5v_fit']:.5f}, LSB={fit_m['lsb_uv']:.2f} uV)"
    )
    axes[0].legend(loc="upper left", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Panel B: Best-fit INL
    lsb_mv = fit_m["lsb_uv"] * 1e-3
    axes[1].plot(fwd_c / 1e3, fwd_inl_mv, "o-", ms=2.5, lw=1.0, color="#1f4e78", label="Forward INL")
    axes[1].plot(rev_c / 1e3, rev_inl_mv, "s-", ms=2.5, lw=1.0, color="#d95f02", alpha=0.85, label="Reverse INL")
    axes[1].axhline(lsb_mv, color="#666666", ls=":", lw=1.0, label=f"+/-1 LSB (+/-{lsb_mv:.3f} mV)")
    axes[1].axhline(-lsb_mv, color="#666666", ls=":", lw=1.0)
    axes[1].set_xlabel("DAC8568 Code (x10^3)")
    axes[1].set_ylabel("Best-Fit INL (mV)")
    axes[1].set_title(
        f"Full-Range Best-Fit INL\n"
        f"(Max={fit_m['best_fit_inl_max_abs_mv']:.3f} mV = {fit_m['best_fit_inl_max_abs_lsb']:.2f} LSB)"
    )
    axes[1].legend(loc="best", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # Panel C: Single-code step & MSB carry DNL
    axes[2].plot(t_codes, t_steps, "o-", ms=3.0, lw=1.1, color="#1b7837", label="Single-Code Step")
    axes[2].axvline(32768, color="#b2182b", ls="--", lw=1.1, label="MSB Carry (32767 -> 32768)")
    axes[2].axhline(fit_m["lsb_uv"], color="#666666", ls=":", lw=1.0, label=f"Global LSB ({fit_m['lsb_uv']:.2f} uV)")
    axes[2].set_xlabel("Destination DAC Code")
    axes[2].set_ylabel("Code Step dV (uV)")
    axes[2].set_title(
        f"64 Consecutive Steps Around Midscale\n"
        f"(Mean={dnl_m['step_mean_uv']:.2f} uV, DNL=[{dnl_m['dnl_global_min_lsb']:+.2f}, {dnl_m['dnl_global_max_lsb']:+.2f}] LSB)"
    )
    axes[2].legend(loc="best", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    return out_png


def run_c1_dc_linearity(
    uart: UARTClient,
    dmm: DM858EClient,
    session: RunSession,
    out_dir: Path,
    *,
    dry_run: bool = False,
    vpi_v: float = DEFAULT_PHASE7_VPI_V,
) -> dict[str, Any]:
    """Execute C1 VA full-range bidirectional DC linearity and single-code step verification."""
    ident = dmm.identify()
    dmm.clear_status()
    st0 = uart.status()
    if st0.state != "IDLE" or st0.drops != 0 or st0.crc_errs != 0:
        raise RuntimeError(f"Board preflight failed: {st0}")
    hw = uart.hw_info()

    full_codes = default_full_sweep_codes()
    single_codes = default_single_code_window()
    sweep_rows: list[dict[str, Any]] = []
    single_rows: list[dict[str, Any]] = []
    static_readings_v: list[float] = []

    try:
        # Part 1: Full-range bidirectional sweep on 10 V range, 5 NPLC
        range_p1, nplc_p1 = dmm.configure_dc_voltage(vrange=10.0, nplc=5.0)
        sequence = [("fwd", c) for c in full_codes] + [("rev", c) for c in reversed(full_codes)]
        for idx, (direction, code) in enumerate(sequence):
            cmd_mv = code_to_cmd_mv(code)
            wait_blocks = 20 if idx == 0 else 12
            meas_rows = uart.meas(cmd_mv, 0.0, wait_blocks, 1)
            row_fw = meas_rows[0]
            if row_fw.code != code:
                raise RuntimeError(f"Firmware quantized code {row_fw.code} != target {code} for cmd_mv={cmd_mv}")
            dmm_v, used_rng = _read_dmm_with_range_guard(dmm, 10.0, 100.0, 5.0)
            sweep_rows.append(
                {
                    "idx": idx,
                    "direction": direction,
                    "code": code,
                    "cmd_mv": cmd_mv,
                    "fw_bias_mv": row_fw.bias_mv,
                    "dmm_v": dmm_v,
                    "dmm_range_v": used_rng,
                    "seq": row_fw.seq,
                    "t_ms": row_fw.t_ms,
                }
            )

        # Part 2: Consecutive single-code sweep around midscale (32736..32800) on 0.1 V range, 20 NPLC
        first_single_mv = code_to_cmd_mv(single_codes[0])
        meas_init = uart.meas(first_single_mv, 0.0, 20, 1)
        if meas_init[0].code != single_codes[0]:
            raise RuntimeError(f"Firmware code {meas_init[0].code} != {single_codes[0]}")

        nominal_single_range = 0.1
        range_p2, nplc_p2 = dmm.configure_dc_voltage(vrange=nominal_single_range, nplc=20.0)
        test_v = dmm.read()
        if math.isinf(test_v):
            nominal_single_range = 1.0
            range_p2, nplc_p2 = dmm.configure_dc_voltage(vrange=nominal_single_range, nplc=20.0)

        for idx, code in enumerate(single_codes):
            cmd_mv = code_to_cmd_mv(code)
            wait_blocks = 12 if idx == 0 else 10
            meas_rows = uart.meas(cmd_mv, 0.0, wait_blocks, 1)
            row_fw = meas_rows[0]
            if row_fw.code != code:
                raise RuntimeError(f"Firmware code {row_fw.code} != target {code}")
            dmm_v, used_rng = _read_dmm_with_range_guard(dmm, nominal_single_range, 10.0, 20.0)
            single_rows.append(
                {
                    "idx": idx,
                    "code": code,
                    "cmd_mv": cmd_mv,
                    "fw_bias_mv": row_fw.bias_mv,
                    "dmm_v": dmm_v,
                    "dmm_range_v": used_rng,
                    "seq": row_fw.seq,
                    "t_ms": row_fw.t_ms,
                }
            )

        # Part 3: 20 static readings at code 32768
        mid_mv = code_to_cmd_mv(32768)
        meas_mid = uart.meas(mid_mv, 0.0, 12, 1)
        if meas_mid[0].code != 32768:
            raise RuntimeError(f"Firmware code {meas_mid[0].code} != 32768")
        for _ in range(20):
            dmm_v, _ = _read_dmm_with_range_guard(dmm, nominal_single_range, 10.0, 20.0)
            static_readings_v.append(dmm_v)
    finally:
        uart.stop()
        st_end = uart.status()
        if st_end.state != "IDLE" or st_end.bias_mv != 0 or st_end.drops != 0 or st_end.crc_errs != 0:
            raise RuntimeError(f"Board did not return safely to IDLE zero bias: {st_end}")

    full_metrics = compute_linearity_metrics(sweep_rows, vpi_v=vpi_v)
    working_rows = [r for r in sweep_rows if -9800.0 <= float(r["cmd_mv"]) <= 9800.0]
    working_metrics = compute_linearity_metrics(working_rows, vpi_v=vpi_v)
    single_metrics, trans_rows = compute_single_code_dnl_metrics(
        single_rows,
        global_lsb_uv=full_metrics["lsb_uv"],
        vpi_v=vpi_v,
    )
    stat_arr = np.asarray(static_readings_v, dtype=float)
    static_metrics = {
        "code": 32768,
        "n_samples": len(static_readings_v),
        "dmm_range_v": range_p2,
        "dmm_nplc": nplc_p2,
        "mean_mv": float(np.mean(stat_arr) * 1e3),
        "std_uv": float(np.std(stat_arr, ddof=1) * 1e6),
        "pk_pk_uv": float((np.max(stat_arr) - np.min(stat_arr)) * 1e6),
        "readings_v": static_readings_v,
    }

    metadata = capture_metadata(
        "c1_dc_linearity",
        fw_info={
            "sysclk_hz": hw.sysclk_hz,
            "adc_id": f"0x{hw.adc_id:04X}",
            "adc_clock": f"0x{hw.adc_clock:04X}",
            "desc_crc": f"0x{hw.desc_crc:08X}",
            "cal_crc": f"0x{hw.cal_crc:08X}",
            "fw": hw.fw,
            "rev": hw.rev,
        },
        dry_run=dry_run,
        notes="MZM disconnected; RIGOL DM858E connected to CN1 VA output",
    )
    summary: dict[str, Any] = {
        "metadata": metadata,
        "instrument": {
            "dmm_idn": ident,
            "part1_range_v": range_p1,
            "part1_nplc": nplc_p1,
            "part2_range_v": range_p2,
            "part2_nplc": nplc_p2,
        },
        "vpi_reference_v": vpi_v,
        "full_range_metrics": full_metrics,
        "working_9800mv_metrics": working_metrics,
        "single_code_metrics": single_metrics,
        "midscale_static_metrics": static_metrics,
        "full_sweep_rows": sweep_rows,
        "single_code_rows": single_rows,
        "single_code_transitions": trans_rows,
    }

    # Save raw artifacts under run_dir
    session.write_metadata(metadata)
    session.write_json("summary.json", summary)
    _write_csv(session.run_dir / "c1_dc_full_sweep.csv", list(sweep_rows[0].keys()), sweep_rows)
    _write_csv(session.run_dir / "c1_dc_single_code_dnl.csv", list(trans_rows[0].keys()), trans_rows)

    # Archive concise deliverables into out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    archive_summary = {k: v for k, v in summary.items() if k not in ("full_sweep_rows", "single_code_rows", "single_code_transitions")}
    atomic_write_json(out_dir / "c1_dc_linearity_summary.json", archive_summary)
    _write_csv(out_dir / "c1_dc_full_sweep.csv", list(sweep_rows[0].keys()), sweep_rows)
    _write_csv(out_dir / "c1_dc_single_code_dnl.csv", list(trans_rows[0].keys()), trans_rows)
    generate_c1_dc_linearity_plot(summary, out_dir / "c1_dc_linearity.png")
    return archive_summary


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    p.add_argument("--dmm-host", default=DEFAULT_DMM_HOST)
    p.add_argument("--dmm-port", type=int, default=DEFAULT_DMM_PORT)
    p.add_argument("--run-dir", default=str(repo_root() / ".agent-bridge" / "exp_runs" / "c1_dc_linearity_live"))
    p.add_argument("--out-dir", default=str(repo_root() / "docs" / "experiments" / "data" / "c1_c2"))
    p.add_argument("--vpi-v", type=float, default=DEFAULT_PHASE7_VPI_V)
    p.add_argument("--dry-run", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    session = RunSession("c1_dc_linearity", run_dir=Path(args.run_dir))

    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261128))
        uart_ctx = UARTClient(transport=SimulatedUARTTransport(plant))
        dmm_ctx = DM858EClient(
            host=args.dmm_host,
            port=args.dmm_port,
            connect_settle_s=0.0,
            cmd_settle_s=0.0,
            socket_factory=lambda *a, **kw: SimulatedDM858ESocket(plant),
        )
    else:
        uart_ctx = UARTClient(port=args.port)
        dmm_ctx = DM858EClient(host=args.dmm_host, port=args.dmm_port)

    with uart_ctx as uart, dmm_ctx as dmm:
        run_c1_dc_linearity(uart, dmm, session, Path(args.out_dir), dry_run=args.dry_run, vpi_v=args.vpi_v)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
