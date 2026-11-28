"""Experiment C1 external oscilloscope verification on VA: 1 kHz pilot gain/THD, noise floor, and tau_LP."""

from __future__ import annotations

import argparse
from collections import deque
import csv
import math
from pathlib import Path
import time
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

from .common import RunSession, atomic_write_json, capture_metadata, repo_root
from .sds824x_client import (
    DEFAULT_HOST as DEFAULT_SCOPE_HOST,
    DEFAULT_PORT as DEFAULT_SCOPE_PORT,
    SDS824XClient,
    build_wavedesc,
)
from .sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient

DESIGN_PILOT_INJ_GAIN = 0.09425
DESIGN_TAU_LP_S = 0.01632
DESIGN_FC_HZ = 1.0 / (2.0 * math.pi * DESIGN_TAU_LP_S)
NOISE_AVG_SETTLE_S = 0.45


def analyze_pilot_harmonics(
    times: np.ndarray,
    volts: np.ndarray,
    *,
    nominal_f0_hz: float = 1000.0,
    max_harmonic: int = 10,
) -> dict[str, Any]:
    """Perform high-accuracy coherent least-squares harmonic decomposition (H1..H10 + THD)."""
    if len(times) != len(volts) or len(times) < 64:
        raise ValueError("Invalid waveform length for harmonic analysis")
    t = np.asarray(times, dtype=np.float64) - float(times[0])
    y = np.asarray(volts, dtype=np.float64)
    y = y - float(np.mean(y))
    dt = float(t[1] - t[0])

    # Fine-tune fundamental frequency around nominal_f0_hz using Hann FFT + quadratic grid search.
    win = np.hanning(len(y))
    freqs_fft = np.fft.rfftfreq(len(y), d=dt)
    fft_mag = np.abs(np.fft.rfft(y * win))
    mask = (freqs_fft >= nominal_f0_hz - 40.0) & (freqs_fft <= nominal_f0_hz + 40.0)
    if np.any(mask):
        idx_local = int(np.argmax(fft_mag[mask]))
        f_coarse = float(freqs_fft[mask][idx_local])
    else:
        f_coarse = nominal_f0_hz

    best_f0 = f_coarse
    best_p1 = -1.0
    for f_cand in np.linspace(f_coarse - 1.5, f_coarse + 1.5, 121):
        ph = 2.0 * math.pi * f_cand * t
        c1 = 2.0 * float(np.mean(y * np.cos(ph)))
        s1 = 2.0 * float(np.mean(y * np.sin(ph)))
        p1 = c1 * c1 + s1 * s1
        if p1 > best_p1:
            best_p1 = p1
            best_f0 = float(f_cand)

    # Joint multi-harmonic linear least-squares fit eliminates slight non-integer window leakage.
    cols = [np.ones_like(t)]
    for k in range(1, max_harmonic + 1):
        ph = 2.0 * math.pi * (k * best_f0) * t
        cols.append(np.cos(ph))
        cols.append(np.sin(ph))
    design = np.column_stack(cols)
    beta, _, _, _ = np.linalg.lstsq(design, y, rcond=None)

    amps_v: list[float] = []
    for k in range(1, max_harmonic + 1):
        ak = float(beta[1 + 2 * (k - 1)])
        bk = float(beta[2 + 2 * (k - 1)])
        amps_v.append(math.hypot(ak, bk))

    h1_pk_v = amps_v[0]
    if h1_pk_v <= 0.0:
        raise RuntimeError("Fundamental pilot amplitude collapsed to zero")

    dbc = [20.0 * math.log10(max(a, 1e-12) / h1_pk_v) for a in amps_v]
    thd_2_5 = math.sqrt(sum(a * a for a in amps_v[1:5])) / h1_pk_v
    thd_2_10 = math.sqrt(sum(a * a for a in amps_v[1:max_harmonic])) / h1_pk_v

    return {
        "f0_hz": best_f0,
        "h1_pk_mv": h1_pk_v * 1e3,
        "h1_rms_mv": h1_pk_v * 1e3 / math.sqrt(2.0),
        "h1_pp_mv": 2.0 * h1_pk_v * 1e3,
        "harmonics_mv_pk": [a * 1e3 for a in amps_v],
        "harmonics_dbc": dbc,
        "h2_dbc": dbc[1],
        "h3_dbc": dbc[2],
        "h4_dbc": dbc[3],
        "h5_dbc": dbc[4],
        "thd_2_5_pct": thd_2_5 * 100.0,
        "thd_2_10_pct": thd_2_10 * 100.0,
        "thd_2_10_db": 20.0 * math.log10(max(thd_2_10, 1e-12)),
    }


def analyze_quiet_noise_welch(
    frames: list[np.ndarray],
    dt_s: float,
) -> dict[str, Any]:
    """Compute Welch averaged ASD (nV/rtHz) and low-frequency/switching spurs from quiet frames."""
    if not frames:
        raise ValueError("No noise frames to analyze")
    fs = 1.0 / dt_s
    n = len(frames[0])
    win = np.hanning(n)
    norm_psd = 2.0 / (fs * float(np.sum(win ** 2)))
    norm_pk = 2.0 / float(np.sum(win))

    psd_acc = np.zeros(n // 2 + 1, dtype=np.float64)
    rms_list = []
    for frame in frames:
        x = np.asarray(frame, dtype=np.float64)
        x = x - float(np.mean(x))
        rms_list.append(float(np.sqrt(np.mean(x ** 2))))
        spec = np.fft.rfft(x * win)
        psd_acc += norm_psd * (np.abs(spec) ** 2)

    psd = psd_acc / float(len(frames))
    asd_nv = np.sqrt(psd) * 1e9
    freqs = np.fft.rfftfreq(n, d=dt_s)
    # Effective single-bin pk amplitude from averaged power: E[|X|^2] -> sqrt(PSD * ENBW_bin * 2)
    enbw_hz = fs * float(np.sum(win ** 2)) / (float(np.sum(win)) ** 2)
    pk_uv = np.sqrt(psd * enbw_hz * 2.0) * 1e6

    i1k = int(np.argmin(np.abs(freqs - 1000.0)))
    i2k = int(np.argmin(np.abs(freqs - 2000.0)))
    band_1k_mask = (freqs >= 700.0) & (freqs <= 1300.0) & (np.abs(freqs - 1000.0) > 40.0)
    band_2k_mask = (freqs >= 1700.0) & (freqs <= 2300.0) & (np.abs(freqs - 2000.0) > 40.0)
    band_sw_mask = (freqs >= 450e3) & (freqs <= 550e3)

    sw_idx = int(np.argmax(pk_uv[band_sw_mask])) if np.any(band_sw_mask) else 0
    sw_freq = float(freqs[band_sw_mask][sw_idx]) if np.any(band_sw_mask) else 0.0
    sw_pk_uv = float(pk_uv[band_sw_mask][sw_idx]) if np.any(band_sw_mask) else 0.0

    def _bin_val(arr: np.ndarray, target_hz: float) -> float:
        idx = int(np.argmin(np.abs(freqs - target_hz)))
        return float(arr[idx])

    return {
        "n_frames": len(frames),
        "n_samples_per_frame": n,
        "fs_hz": fs,
        "df_hz": float(freqs[1] - freqs[0]),
        "enbw_hz": enbw_hz,
        "broadband_rms_uv": float(np.mean(rms_list) * 1e6),
        "asd_1k_bin_nv_rthz": float(asd_nv[i1k]),
        "asd_1k_band_median_nv_rthz": float(np.median(asd_nv[band_1k_mask])),
        "asd_2k_bin_nv_rthz": float(asd_nv[i2k]),
        "asd_2k_band_median_nv_rthz": float(np.median(asd_nv[band_2k_mask])),
        "spur_50hz_uv_pk": _bin_val(pk_uv, 50.0),
        "spur_100hz_uv_pk": _bin_val(pk_uv, 100.0),
        "spur_200hz_uv_pk": _bin_val(pk_uv, 200.0),
        "spur_1khz_uv_pk": float(pk_uv[i1k]),
        "spur_2khz_uv_pk": float(pk_uv[i2k]),
        "spur_500khz_freq_hz": sw_freq,
        "spur_500khz_uv_pk": sw_pk_uv,
        "freqs_hz": freqs[freqs <= 5000.0].tolist(),
        "asd_nv_rthz_0_5khz": asd_nv[freqs <= 5000.0].tolist(),
    }


def fit_step_tau_lp(times: np.ndarray, volts: np.ndarray, *, rising: bool) -> dict[str, Any]:
    """Fit single-pole RC step response to extract tau_LP and cutoff fc."""
    t = np.asarray(times, dtype=np.float64)
    v = np.asarray(volts, dtype=np.float64)
    n = len(t)
    v_pre = float(np.mean(v[: max(n // 20, 10)]))
    v_post = float(np.mean(v[-max(n // 10, 10) :]))
    span = v_post - v_pre
    if (rising and span <= 0.15) or ((not rising) and span >= -0.15):
        raise RuntimeError(f"Step transition span too small for fit: v_pre={v_pre:.4f}, v_post={v_post:.4f}")

    # Find 10% and 90% crossing timestamps using edge-padded sliding average (no zero-pad boundary artifact).
    win = max(min(n // 200, 101), 5)
    pad_left = win // 2
    pad_right = win - 1 - pad_left
    v_padded = np.pad(v, (pad_left, pad_right), mode="edge")
    v_smooth = np.convolve(v_padded, np.ones(win) / win, mode="valid")
    frac = (v_smooth - v_pre) / span
    idx_10 = int(np.argmax(frac >= 0.10))
    idx_90 = int(np.argmax(frac >= 0.90))
    if idx_90 <= idx_10:
        raise RuntimeError(f"Invalid 10%/90% crossings: idx_10={idx_10}, idx_90={idx_90}")
    t_10_90_ms = float((t[idx_90] - t[idx_10]) * 1e3)

    # Fit exponential from 10% crossing out to +120 ms (or end of record).
    end_idx = min(n, idx_10 + int(0.120 / float(t[1] - t[0])))
    t_seg = t[idx_10:end_idx] - float(t[idx_10])
    v_seg = v[idx_10:end_idx]
    v_at_10 = v_pre + 0.10 * span

    def _model(tt: np.ndarray, vf: float, amp: float, tau_s: float) -> np.ndarray:
        return vf + amp * np.exp(-tt / tau_s)

    p0 = [v_post, v_at_10 - v_post, DESIGN_TAU_LP_S]
    bounds = ([v_post - 0.1, -2.0, 1e-4], [v_post + 0.1, 2.0, 0.5])
    popt, _ = curve_fit(_model, t_seg, v_seg, p0=p0, bounds=bounds, maxfev=10000)
    vf_fit, amp_fit, tau_s_fit = float(popt[0]), float(popt[1]), float(popt[2])
    pred = _model(t_seg, vf_fit, amp_fit, tau_s_fit)
    rmse_mv = float(np.sqrt(np.mean((v_seg - pred) ** 2)) * 1e3)

    if rising:
        overshoot_pct = max(0.0, (float(np.max(v_smooth[idx_10:])) - v_post) / abs(span) * 100.0)
    else:
        overshoot_pct = max(0.0, (v_post - float(np.min(v_smooth[idx_10:]))) / abs(span) * 100.0)

    return {
        "direction": "rising" if rising else "falling",
        "v_pre_v": v_pre,
        "v_post_v": v_post,
        "step_span_v": span,
        "t_10_90_ms": t_10_90_ms,
        "tau_lp_ms": tau_s_fit * 1e3,
        "tau_err_pct": (tau_s_fit - DESIGN_TAU_LP_S) / DESIGN_TAU_LP_S * 100.0,
        "fc_hz": 1.0 / (2.0 * math.pi * tau_s_fit),
        "overshoot_pct": overshoot_pct,
        "fit_rmse_mv": rmse_mv,
        "fit_amp_v": amp_fit,
        "t_rel_ms": ((t[max(0, idx_10 - int(0.015 / float(t[1] - t[0]))):end_idx] - float(t[idx_10])) * 1e3).tolist()[::10],
        "v_trace_v": v[max(0, idx_10 - int(0.015 / float(t[1] - t[0]))):end_idx].tolist()[::10],
    }


class SimulatedSDS824XSocket:
    """Mock TCP socket modeling Siglent SDS824X HD SCPI and binary WAVEDESC/WORD blocks."""

    def __init__(self, plant: SimulatedMZMPlant, *, seed: int = 20261128) -> None:
        self.plant = plant
        self.rng = np.random.default_rng(seed)
        self.vdiv = 0.05
        self.offset = 0.0
        self.tdiv = 2e-3
        self.delay = 0.0
        self.mdepth_pts = 100000
        self.cpl = "A1M"
        self.trig_mode = "AUTO"
        self.trig_status = "Auto"
        self.slope = "RISing"
        self.trig_level = 0.0
        self.armed_bias_v: float | None = None
        self.pending_step_v: tuple[float, float] | None = None
        self._rx_buf: deque[bytes] = deque()
        self.closed = False

    def settimeout(self, timeout: float) -> None:
        self.timeout = timeout

    def gettimeout(self) -> float:
        return getattr(self, "timeout", 5.0)

    def connect(self, addr) -> None:
        self.addr = addr

    def close(self) -> None:
        self.closed = True

    def _push_text(self, s: str) -> None:
        self._rx_buf.append((s + "\n").encode("ascii"))

    def _push_binary(self, payload: bytes) -> None:
        hdr = f"#9{len(payload):09d}".encode("ascii")
        self._rx_buf.append(hdr + payload + b"\n")

    def _sample_rate(self) -> float:
        total_t = self.tdiv * 10.0
        return float(self.mdepth_pts) / total_t

    def _generate_waveform(self) -> tuple[bytes, bytes]:
        n = self.mdepth_pts
        total_t = self.tdiv * 10.0
        dt = total_t / float(n)
        t0 = -(total_t / 2.0) + self.delay
        t = t0 + np.arange(n, dtype=np.float64) * dt

        if self.pending_step_v is not None:
            v0, v1 = self.pending_step_v
            v = np.where(t < 0.0, v0, v1 + (v0 - v1) * np.exp(-np.maximum(t, 0.0) / DESIGN_TAU_LP_S))
            v = v + self.rng.normal(0.0, 2e-4, size=n)
            self.pending_step_v = None
        else:
            p_cmd_v = self.plant.pilot_v_cmd
            v_va_pk = (p_cmd_v * 10.0) * DESIGN_PILOT_INJ_GAIN
            if self.cpl == "A1M":
                v_dc = 0.0
            else:
                v_dc = self.plant.bias_v
            # Synthetic 1 kHz pilot with small H2 (-70 dBc) and H3 (-75 dBc) harmonics + quiet noise.
            v = v_dc + v_va_pk * (
                np.sin(2.0 * np.pi * 1000.0 * t + 0.3)
                + 10 ** (-70.0 / 20.0) * np.sin(2.0 * np.pi * 2000.0 * t + 0.7)
                + 10 ** (-75.0 / 20.0) * np.sin(2.0 * np.pi * 3000.0 * t + 1.1)
            )
            fs = 1.0 / dt
            sigma_noise = 60e-9 * math.sqrt(fs / 2.0) if p_cmd_v == 0 else 5e-5
            v = v + self.rng.normal(0.0, sigma_noise, size=n)

        code_per_div = 7680.0
        codes = np.clip(np.round((v + self.offset) / self.vdiv * code_per_div), -32768, 32767).astype("<i2")
        desc = build_wavedesc(
            point_num=n,
            vdiv=self.vdiv,
            offset=self.offset,
            code_per_div=code_per_div,
            adc_bit=16,
            interval=dt,
            delay=self.delay,
            tdiv_s=self.tdiv,
        )
        return desc, codes.tobytes()

    def sendall(self, data: bytes) -> None:
        cmd = data.decode("ascii").strip()
        upper = cmd.upper()
        if upper == "*IDN?":
            self._push_text("Siglent Technologies,SDS824X HD,SDS_SIM,4.8.12.1.1.6.5")
        elif upper == ":TRIGGER:STATUS?":
            if (
                self.trig_mode == "SINGLE"
                and self.trig_status == "Ready"
                and self.armed_bias_v is not None
                and abs(self.plant.bias_v - self.armed_bias_v) > 0.1
            ):
                self.pending_step_v = (self.armed_bias_v, float(self.plant.bias_v))
                self.armed_bias_v = None
                self.trig_status = "Stop"
            self._push_text(self.trig_status)
        elif upper == "TRMD AUTO":
            self.trig_mode = "AUTO"
            self.trig_status = "Auto"
            self.armed_bias_v = None
        elif upper == "TRMD SINGLE":
            self.trig_mode = "SINGLE"
            if self.trig_status != "Stop":
                self.trig_status = "Ready"
                self.armed_bias_v = float(self.plant.bias_v)
            self.pending_step_v = None
        elif upper == ":TRIGGER:RUN":
            if self.trig_mode == "SINGLE":
                self.trig_status = "Ready"
                self.armed_bias_v = float(self.plant.bias_v)
            else:
                self.trig_status = "Auto"
                self.armed_bias_v = None
        elif upper in (":STOP", ":TRIGGER:STOP"):
            self.trig_status = "Stop"
        elif upper.startswith("C1:CPL "):
            self.cpl = upper.split()[-1]
        elif upper.startswith("C1:VDIV "):
            self.vdiv = float(cmd.split()[-1])
        elif upper.startswith("C1:OFST "):
            self.offset = float(cmd.split()[-1])
        elif upper.startswith("TDIV "):
            self.tdiv = float(cmd.split()[-1])
        elif upper.startswith("TRDL "):
            self.delay = float(cmd.split()[-1])
        elif upper.startswith(":ACQUIRE:MDEPTH "):
            tok = upper.split()[-1]
            self.mdepth_pts = int(float(tok[:-1]) * 1000) if tok.endswith("K") else int(tok)
        elif upper == ":ACQUIRE:SRATE?":
            self._push_text(f"{self._sample_rate():.6E}")
        elif upper.startswith(":TRIGGER:EDGE:SLOPE "):
            self.slope = cmd.split()[-1]
        elif upper.startswith(":TRIGGER:EDGE:LEVEL "):
            self.trig_level = float(cmd.split()[-1])
        elif upper.startswith("C1:PAVA? "):
            item = upper.split()[-1]
            p_pk = (self.plant.pilot_v_cmd * 10.0) * DESIGN_PILOT_INJ_GAIN
            if item == "PKPK":
                self._push_text(f"C1:PAVA PKPK,{2.0 * p_pk:.4E}V")
            elif item == "RMS":
                self._push_text(f"C1:PAVA RMS,{p_pk / math.sqrt(2.0):.4E}V")
            elif item == "FREQ":
                self._push_text("C1:PAVA FREQ,1.0000E+03Hz" if p_cmd_nz(self.plant.pilot_v_cmd) else "C1:PAVA FREQ,****Hz")
            else:
                self._push_text(f"C1:PAVA {item},0.0E+00V")
        elif upper == ":WAVEFORM:PREAMBLE?":
            desc, raw = self._generate_waveform()
            self._cached_raw = raw
            self._push_binary(desc)
        elif upper == ":WAVEFORM:DATA?":
            if not hasattr(self, "_cached_raw"):
                _, raw = self._generate_waveform()
                self._cached_raw = raw
            hdr = f"#9{len(self._cached_raw):09d}".encode("ascii")
            # Model real Siglent double-LF binary termination.
            self._rx_buf.append(hdr + self._cached_raw + b"\n\n")

    def recv(self, nbytes: int) -> bytes:
        if not self._rx_buf:
            return b""
        chunk = self._rx_buf[0]
        if len(chunk) <= nbytes:
            self._rx_buf.popleft()
            return chunk
        self._rx_buf[0] = chunk[nbytes:]
        return chunk[:nbytes]


def p_cmd_nz(val: float) -> bool:
    return abs(val) > 1e-6


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def generate_c1_va_scope_plot(summary: dict[str, Any], out_png: Path) -> Path:
    """Render 3-panel plot: (a) gain vs bias, (b) 0-10 kHz spectrum, (c) tau_LP steps."""
    sweep = summary["part_a_pilot_rows"]
    s159 = [r for r in sweep if abs(r["pilot_mv"] - 159.2) < 0.5]
    s150 = [r for r in sweep if abs(r["pilot_mv"] - 150.0) < 0.5]
    ref_row = next((r for r in s159 if abs(r["bias_mv"]) < 1.0), s159[0])
    step_r = summary["part_c_steps"]["rising"]
    step_f = summary["part_c_steps"]["falling"]

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0), constrained_layout=True)

    # Panel A: Injection gain vs bias
    axes[0].plot(
        [r["bias_mv"] * 1e-3 for r in s159],
        [r["inj_gain_mean"] for r in s159],
        "o-",
        color="#1f4e78",
        label="1.592 V src (159.2 mV cmd)",
    )
    axes[0].plot(
        [r["bias_mv"] * 1e-3 for r in s150],
        [r["inj_gain_mean"] for r in s150],
        "s--",
        color="#d95f02",
        label="1.500 V src (150.0 mV cmd)",
    )
    axes[0].axhline(DESIGN_PILOT_INJ_GAIN, color="#666666", ls=":", lw=1.2, label=f"Design ({DESIGN_PILOT_INJ_GAIN:.5f})")
    axes[0].set_xlabel("DC Bias VA (V)")
    axes[0].set_ylabel("Measured 1 kHz Gain (V_VA / V_src)")
    axes[0].set_title(
        f"Pilot Gain Flatness Across Bias\n"
        f"(Mean Gain={ref_row['inj_gain_mean']:.5f}, err={ref_row['gain_err_pct']:+.2f}%)"
    )
    axes[0].legend(loc="best", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Panel B: Harmonic levels at 0 V / 159.2 mV
    orders = np.arange(1, len(ref_row["harmonics_dbc_mean"]) + 1)
    axes[1].bar(orders, ref_row["harmonics_dbc_mean"], color="#1b7837", width=0.55)
    axes[1].set_xticks(orders, [f"H{k}" for k in orders])
    axes[1].set_ylabel("Amplitude Relative to H1 (dBc)")
    axes[1].set_ylim(-100, 5)
    axes[1].set_title(
        f"1 kHz VA Harmonics & THD (0 V, 1.592 V src)\n"
        f"(H2={ref_row['h2_dbc_mean']:.1f} dBc, H3={ref_row['h3_dbc_mean']:.1f} dBc, THD={ref_row['thd_2_10_pct_mean']:.4f}%)"
    )
    axes[1].grid(True, axis="y", alpha=0.3)

    # Panel C: 0 <-> +500 mV step responses
    axes[2].plot(step_r["t_rel_ms"], [v * 1e3 for v in step_r["v_trace_v"]], color="#1f4e78", lw=1.3, label=f"Rise tau={step_r['tau_lp_ms']:.2f} ms")
    axes[2].plot(step_f["t_rel_ms"], [v * 1e3 for v in step_f["v_trace_v"]], color="#d95f02", lw=1.3, label=f"Fall tau={step_f['tau_lp_ms']:.2f} ms")
    axes[2].set_xlabel("Time Since 10% Crossing (ms)")
    axes[2].set_ylabel("VA Step Voltage (mV)")
    axes[2].set_title(
        f"Direct 0 <-> 500 mV VA Step Response\n"
        f"(Design tau_LP=16.32 ms, fc=9.75 Hz)"
    )
    axes[2].legend(loc="best", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    return out_png


def run_c1_va_scope(
    uart: UARTClient,
    scope: SDS824XClient,
    session: RunSession,
    out_dir: Path,
    *,
    dry_run: bool = False,
    n_harm_frames: int = 6,
    n_noise_frames: int = 16,
) -> dict[str, Any]:
    """Run C1 VA oscilloscope measurements: pilot gain/THD, quiet noise floor, and tau_LP step."""
    ident = scope.identify()
    st0 = uart.status()
    if st0.state != "IDLE" or st0.drops != 0 or st0.crc_errs != 0:
        raise RuntimeError(f"Board preflight failed: {st0}")
    hw = uart.hw_info()

    sweep_points: list[tuple[float, float, float]] = [
        (-8000.0, 159.2, 0.05),
        (-4000.0, 159.2, 0.05),
        (0.0, 159.2, 0.05),
        (4000.0, 159.2, 0.05),
        (8000.0, 159.2, 0.05),
        (-8000.0, 150.0, 0.05),
        (-4000.0, 150.0, 0.05),
        (0.0, 150.0, 0.05),
        (4000.0, 150.0, 0.05),
        (8000.0, 150.0, 0.05),
        (0.0, 84.9, 0.05),
        (0.0, 130.0, 0.05),
    ]

    pilot_rows: list[dict[str, Any]] = []
    noise_results: dict[str, Any] = {}
    step_results: dict[str, Any] = {}

    try:
        # Part A: Pilot gain, bias flatness, and THD across [-8..+8 V] and [84.9..159.2 mV]
        scope.set_mdepth("100K")
        scope.setup_timebase(2e-3, delay_s=0.0)
        scope.setup_edge_trigger(source="C1", slope="RISing", level=0.0, coupling="AC")

        prev_vdiv = None
        for idx, (bias_mv, pilot_mv, vdiv) in enumerate(sweep_points):
            if prev_vdiv != vdiv:
                scope.setup_channel(1, display=True, coupling="AC", scale=vdiv, offset=0.0, attn=1, bwl=True)
                prev_vdiv = vdiv
            uart.meas(bias_mv, pilot_mv, 20 if idx == 0 else 15, 1)

            frame_stats = []
            for _ in range(n_harm_frames):
                t, v, _meta = scope.capture_auto_waveform(1, settle_s=0.35 if dry_run else 0.40)
                frame_stats.append(analyze_pilot_harmonics(t, v))

            if not dry_run:
                time.sleep(0.25)
            pava_pkpk_mv = scope.measure("PKPK", 1) * 1e3
            pava_rms_mv = scope.measure("RMS", 1) * 1e3
            pava_freq_hz = scope.measure("FREQ", 1)

            h1_vals = [f["h1_pk_mv"] for f in frame_stats]
            v_src_mv = 10.0 * pilot_mv
            gains = [h / v_src_mv for h in h1_vals]
            mean_gain = float(np.mean(gains))
            harm_dbc_mean = np.mean([f["harmonics_dbc"] for f in frame_stats], axis=0).tolist()

            pilot_rows.append(
                {
                    "bias_mv": bias_mv,
                    "pilot_mv": pilot_mv,
                    "v_src_mv_pk": v_src_mv,
                    "expected_va_mv_pk": v_src_mv * DESIGN_PILOT_INJ_GAIN,
                    "vdiv_v": vdiv,
                    "n_frames": n_harm_frames,
                    "f0_hz_mean": float(np.mean([f["f0_hz"] for f in frame_stats])),
                    "h1_mv_pk_mean": float(np.mean(h1_vals)),
                    "h1_mv_pk_std": float(np.std(h1_vals, ddof=1)) if len(h1_vals) > 1 else 0.0,
                    "h1_mv_rms_mean": float(np.mean([f["h1_rms_mv"] for f in frame_stats])),
                    "inj_gain_mean": mean_gain,
                    "gain_err_pct": (mean_gain - DESIGN_PILOT_INJ_GAIN) / DESIGN_PILOT_INJ_GAIN * 100.0,
                    "h2_dbc_mean": float(np.mean([f["h2_dbc"] for f in frame_stats])),
                    "h3_dbc_mean": float(np.mean([f["h3_dbc"] for f in frame_stats])),
                    "h4_dbc_mean": float(np.mean([f["h4_dbc"] for f in frame_stats])),
                    "h5_dbc_mean": float(np.mean([f["h5_dbc"] for f in frame_stats])),
                    "thd_2_5_pct_mean": float(np.mean([f["thd_2_5_pct"] for f in frame_stats])),
                    "thd_2_10_pct_mean": float(np.mean([f["thd_2_10_pct"] for f in frame_stats])),
                    "thd_2_10_db_mean": float(np.mean([f["thd_2_10_db"] for f in frame_stats])),
                    "pava_pkpk_mv": pava_pkpk_mv,
                    "pava_rms_mv": pava_rms_mv,
                    "pava_freq_hz": pava_freq_hz,
                    "harmonics_dbc_mean": harm_dbc_mean,
                }
            )

        # Part B: Quiet VA noise floor at 0 V and QTP+ (4912 mV), pilot=0, 2 mV/div, 50 ms
        scope.setup_channel(1, display=True, coupling="AC", scale=0.002, offset=0.0, attn=1, bwl=True)
        scope.setup_timebase(5e-3, delay_s=0.0)
        scope.set_mdepth("100K")
        scope.setup_edge_trigger(source="C1", slope="RISing", level=0.0, coupling="DC")

        for label, bias_mv in (("0V", 0.0), ("QTP_4912mV", 4912.0)):
            uart.meas(bias_mv, 0.0, 20, 1)
            frames = []
            dt_s = 0.0
            for _ in range(n_noise_frames):
                t, v, meta = scope.capture_auto_waveform(1, settle_s=0.35 if dry_run else 0.45)
                frames.append(v)
                dt_s = float(meta["interval"])
            noise_results[label] = analyze_quiet_noise_welch(frames, dt_s)

        # Part C: Direct DC step response 0 <-> +500 mV to measure tau_LP
        scope.setup_channel(1, display=True, coupling="DC", scale=0.2, offset=0.0, attn=1, bwl=True)
        scope.setup_timebase(20e-3, delay_s=0.0)
        scope.set_mdepth("100K")

        # Rising 0 -> +500 mV
        uart.meas(0.0, 0.0, 15, 1)
        scope.setup_edge_trigger(source="C1", slope="RISing", level=0.15, coupling="DC")
        scope.arm_single()
        scope.wait_trigger_status(("Ready",), timeout_s=4.0)
        uart.meas(500.0, 0.0, 15, 1)
        scope.wait_trigger_status(("Stop",), timeout_s=5.0)
        t_rise, v_rise, _ = scope.read_stopped_waveform(1)
        step_results["rising"] = fit_step_tau_lp(t_rise, v_rise, rising=True)

        # Falling +500 -> 0 mV
        uart.meas(500.0, 0.0, 15, 1)
        scope.setup_edge_trigger(source="C1", slope="FALLing", level=0.35, coupling="DC")
        scope.arm_single()
        scope.wait_trigger_status(("Ready",), timeout_s=4.0)
        uart.meas(0.0, 0.0, 15, 1)
        scope.wait_trigger_status(("Stop",), timeout_s=5.0)
        t_fall, v_fall, _ = scope.read_stopped_waveform(1)
        step_results["falling"] = fit_step_tau_lp(t_fall, v_fall, rising=False)
    finally:
        uart.stop()
        st_end = uart.status()
        if st_end.state != "IDLE" or st_end.bias_mv != 0 or st_end.drops != 0 or st_end.crc_errs != 0:
            raise RuntimeError(f"Board did not return safely to IDLE zero bias: {st_end}")

    def _flatness(pilot_target: float) -> dict[str, float]:
        sub = [r["h1_mv_pk_mean"] for r in pilot_rows if abs(r["pilot_mv"] - pilot_target) < 0.5]
        gains = [r["inj_gain_mean"] for r in pilot_rows if abs(r["pilot_mv"] - pilot_target) < 0.5]
        mean_h1 = float(np.mean(sub))
        return {
            "pilot_mv": pilot_target,
            "h1_mean_mv_pk": mean_h1,
            "gain_mean": float(np.mean(gains)),
            "gain_err_pct": (float(np.mean(gains)) - DESIGN_PILOT_INJ_GAIN) / DESIGN_PILOT_INJ_GAIN * 100.0,
            "bias_ptp_pct": (float(np.max(sub)) - float(np.min(sub))) / mean_h1 * 100.0,
            "bias_rel_std_pct": float(np.std(sub, ddof=1)) / mean_h1 * 100.0 if len(sub) > 1 else 0.0,
        }

    metadata = capture_metadata(
        "c1_va_scope",
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
        notes="MZM disconnected; SDS824X HD C1 (1x probe, 20 MHz BWL) connected to CN1 VA",
    )

    csv_fields = [
        "bias_mv", "pilot_mv", "v_src_mv_pk", "expected_va_mv_pk", "vdiv_v",
        "f0_hz_mean", "h1_mv_pk_mean", "h1_mv_pk_std", "h1_mv_rms_mean",
        "inj_gain_mean", "gain_err_pct", "h2_dbc_mean", "h3_dbc_mean",
        "h4_dbc_mean", "h5_dbc_mean", "thd_2_5_pct_mean", "thd_2_10_pct_mean",
        "thd_2_10_db_mean", "pava_pkpk_mv", "pava_rms_mv", "pava_freq_hz",
    ]
    csv_rows = [{k: row[k] for k in csv_fields} for row in pilot_rows]

    # Omit large trace arrays from concise out_dir archive summary; keep full data in run_dir.
    noise_concise = {
        k: {kk: vv for kk, vv in v.items() if kk not in ("freqs_hz", "asd_nv_rthz_0_5khz")}
        for k, v in noise_results.items()
    }
    step_concise = {
        k: {kk: vv for kk, vv in v.items() if kk not in ("t_rel_ms", "v_trace_v")}
        for k, v in step_results.items()
    }

    summary = {
        "metadata": metadata,
        "scope_idn": ident,
        "design_references": {
            "pilot_inj_gain_1khz": DESIGN_PILOT_INJ_GAIN,
            "tau_lp_ms": DESIGN_TAU_LP_S * 1e3,
            "fc_hz": DESIGN_FC_HZ,
        },
        "part_a_flatness": {
            "pilot_159_2mv": _flatness(159.2),
            "pilot_150_0mv": _flatness(150.0),
        },
        "part_a_pilot_rows": pilot_rows,
        "part_b_noise": noise_results,
        "part_c_steps": step_results,
    }
    archive_summary = {
        "metadata": metadata,
        "scope_idn": ident,
        "design_references": summary["design_references"],
        "part_a_flatness": summary["part_a_flatness"],
        "part_a_pilot_rows": csv_rows,
        "part_b_noise": noise_concise,
        "part_c_steps": step_concise,
    }

    session.write_metadata(metadata)
    session.write_json("summary.json", summary)
    _write_csv(session.run_dir / "c1_va_pilot_sweep.csv", csv_fields, csv_rows)

    out_dir.mkdir(parents=True, exist_ok=True)
    atomic_write_json(out_dir / "c1_va_scope_summary.json", archive_summary)
    _write_csv(out_dir / "c1_va_pilot_sweep.csv", csv_fields, csv_rows)
    generate_c1_va_scope_plot(summary, out_dir / "c1_va_scope_char.png")
    return archive_summary


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    p.add_argument("--scope-host", default=DEFAULT_SCOPE_HOST)
    p.add_argument("--scope-port", type=int, default=DEFAULT_SCOPE_PORT)
    p.add_argument("--run-dir", default=str(repo_root() / ".agent-bridge" / "exp_runs" / "c1_va_scope_live"))
    p.add_argument("--out-dir", default=str(repo_root() / "docs" / "experiments" / "data" / "c1_c2"))
    p.add_argument("--harm-frames", type=int, default=6)
    p.add_argument("--noise-frames", type=int, default=16)
    p.add_argument("--dry-run", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    session = RunSession("c1_va_scope", run_dir=Path(args.run_dir))

    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261128))
        uart_ctx = UARTClient(transport=SimulatedUARTTransport(plant))
        scope_ctx = SDS824XClient(
            host=args.scope_host,
            port=args.scope_port,
            cmd_sleep_s=0.0,
            waveform_sleep_s=0.0,
            settle_scale=0.0,
            socket_factory=lambda *a, **kw: SimulatedSDS824XSocket(plant),
        )
    else:
        uart_ctx = UARTClient(port=args.port)
        scope_ctx = SDS824XClient(host=args.scope_host, port=args.scope_port)

    with uart_ctx as uart, scope_ctx as scope:
        run_c1_va_scope(
            uart,
            scope,
            session,
            Path(args.out_dir),
            dry_run=args.dry_run,
            n_harm_frames=args.harm_frames,
            n_noise_frames=args.noise_frames,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
