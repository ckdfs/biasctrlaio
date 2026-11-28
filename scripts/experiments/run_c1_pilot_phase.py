"""Experiment C1 dual-channel oscilloscope verification: C48.1 (DITHER1) -> VA 1 kHz pilot transfer & phase."""

from __future__ import annotations

import argparse
import cmath
import math
from pathlib import Path
import time
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize_scalar

from .common import RunSession, atomic_write_json, capture_metadata, repo_root
from .run_c1_va_scope import (
    DESIGN_PILOT_INJ_GAIN,
    SimulatedSDS824XSocket,
    _write_csv,
    p_cmd_nz,
)
from .sds824x_client import (
    DEFAULT_HOST as DEFAULT_SCOPE_HOST,
    DEFAULT_PORT as DEFAULT_SCOPE_PORT,
    SDS824XClient,
    build_wavedesc,
)
from .sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from .uart_client import DEFAULT_SERIAL_PORT, UARTClient

R_INJ_OHM = 120_000.0
C_INJ_F = 100e-9
R_FB_OHM = 12_000.0
C_FB_F = 4.7e-9


def design_pilot_transfer(freq_hz: float = 1000.0) -> complex:
    """Exact theoretical injection transfer T_i(jw) = V_VA(jw) / V_C48.1(jw)."""
    s = 1j * 2.0 * math.pi * float(freq_hz)
    hpf = (s * R_INJ_OHM * C_INJ_F) / (1.0 + s * R_INJ_OHM * C_INJ_F)
    lpf = 1.0 / (1.0 + s * R_FB_OHM * C_FB_F)
    return -0.1 * hpf * lpf


def analyze_dual_channel_transfer(
    t_va: np.ndarray,
    v_va: np.ndarray,
    t_src: np.ndarray,
    v_src: np.ndarray,
    *,
    nominal_f0_hz: float = 1000.0,
    max_harmonic: int = 10,
) -> dict[str, Any]:
    """Coherent joint harmonic decomposition and complex phasor transfer Z_va / Z_src."""
    if len(t_va) != len(v_va) or len(t_src) != len(v_src) or len(t_va) != len(t_src) or len(t_va) < 64:
        raise ValueError("Invalid or mismatched dual-channel waveform lengths")
    t1 = np.asarray(t_va, dtype=np.float64)
    t2 = np.asarray(t_src, dtype=np.float64)
    dt = float(t2[1] - t2[0])
    dt1 = float(t1[1] - t1[0])
    if dt <= 0.0 or abs(dt - dt1) > 1e-12 or abs(float(t1[0]) - float(t2[0])) > 1e-9:
        raise ValueError("Mismatched dual-channel time axes")

    t = t2 - float(t2[0])
    y_va = np.asarray(v_va, dtype=np.float64)
    y_src = np.asarray(v_src, dtype=np.float64)
    src_ac = y_src - float(np.mean(y_src))

    # Estimate common fundamental frequency from clean large-swing C48.1 source waveform
    # by maximizing single-tone least-squares explained variance (immune to image leakage).
    win = np.hanning(len(src_ac))
    freqs_fft = np.fft.rfftfreq(len(src_ac), d=dt)
    df_fft = float(freqs_fft[1] - freqs_fft[0])
    fft_mag = np.abs(np.fft.rfft(src_ac * win))
    mask = (freqs_fft >= nominal_f0_hz - 40.0) & (freqs_fft <= nominal_f0_hz + 40.0)
    f_coarse = float(freqs_fft[mask][int(np.argmax(fft_mag[mask]))]) if np.any(mask) else float(nominal_f0_hz)

    ones = np.ones_like(t)

    def _neg_explained_energy(f_cand: float) -> float:
        ph = (2.0 * math.pi * f_cand) * t
        x_tone = np.column_stack((ones, np.cos(ph), np.sin(ph)))
        xtx = x_tone.T @ x_tone
        xty = x_tone.T @ src_ac
        beta_tone = np.linalg.solve(xtx, xty)
        return -float(beta_tone @ xty)

    coarse_grid = np.linspace(max(1.0, f_coarse - 0.6 * df_fft), f_coarse + 0.6 * df_fft, 13)
    f_peak = float(min(coarse_grid, key=_neg_explained_energy))
    half_win = max(1.5, 0.25 * df_fft)
    opt = minimize_scalar(
        _neg_explained_energy,
        bounds=(max(1.0, f_peak - half_win), f_peak + half_win),
        method="bounded",
        options={"xatol": 1e-4},
    )
    best_f0 = float(opt.x)

    cols = [np.ones_like(t)]
    for k in range(1, max_harmonic + 1):
        ph = 2.0 * math.pi * (k * best_f0) * t
        cols.append(np.cos(ph))
        cols.append(np.sin(ph))
    design = np.column_stack(cols)
    targets = np.column_stack([y_va, y_src])
    beta, _, _, _ = np.linalg.lstsq(design, targets, rcond=None)

    def _channel_metrics(col_idx: int) -> tuple[complex, float, float, float]:
        amps: list[float] = []
        a1 = float(beta[1, col_idx])
        b1 = float(beta[2, col_idx])
        z1 = complex(a1, -b1)  # cos(wt + phi) = cos(phi)cos(wt) - sin(phi)sin(wt)
        for k in range(1, max_harmonic + 1):
            ak = float(beta[1 + 2 * (k - 1), col_idx])
            bk = float(beta[2 + 2 * (k - 1), col_idx])
            amps.append(math.hypot(ak, bk))
        h1_pk = amps[0]
        if h1_pk <= 0.0:
            raise RuntimeError(f"Zero fundamental detected on channel column {col_idx}")
        thd_2_10 = math.sqrt(sum(a * a for a in amps[1:max_harmonic])) / h1_pk
        h2_dbc = 20.0 * math.log10(max(amps[1], 1e-12) / h1_pk)
        return z1, h1_pk, thd_2_10 * 100.0, h2_dbc

    z_va, va_pk_v, ch1_thd_pct, ch1_h2_dbc = _channel_metrics(0)
    z_src, src_pk_v, ch2_thd_pct, ch2_h2_dbc = _channel_metrics(1)

    h_meas = z_va / z_src
    gain_direct = abs(h_meas)
    total_phase_deg = math.degrees(cmath.phase(h_meas))
    extra_phase_deg = math.degrees(cmath.phase(-h_meas))
    extra_delay_us = (-extra_phase_deg / 360.0) / best_f0 * 1e6

    h_design = design_pilot_transfer(best_f0)
    design_gain = abs(h_design)
    design_extra_deg = math.degrees(cmath.phase(-h_design))

    return {
        "f0_hz": best_f0,
        "ch1_va_pk_mv": va_pk_v * 1e3,
        "ch2_src_pk_mv": src_pk_v * 1e3,
        "ch1_dc_mean_v": float(np.mean(y_va)),
        "ch2_dc_mean_v": float(np.mean(y_src)),
        "gain_direct": gain_direct,
        "gain_err_vs_design_pct": (gain_direct - design_gain) / design_gain * 100.0,
        "total_phase_deg": total_phase_deg,
        "extra_phase_deg": extra_phase_deg,
        "phase_err_vs_design_deg": extra_phase_deg - design_extra_deg,
        "extra_delay_us": extra_delay_us,
        "ch1_thd_2_10_pct": ch1_thd_pct,
        "ch2_thd_2_10_pct": ch2_thd_pct,
        "ch1_h2_dbc": ch1_h2_dbc,
        "ch2_h2_dbc": ch2_h2_dbc,
    }


class SimulatedDualSDS824XSocket(SimulatedSDS824XSocket):
    """Mock two-channel SDS824X HD modeling synchronized C1 (VA) and C2 (C48.1)."""

    def __init__(self, plant: SimulatedMZMPlant, *, seed: int = 20261128) -> None:
        super().__init__(plant, seed=seed)
        self.wav_source = 1
        self.ch_cfg: dict[int, dict[str, Any]] = {
            1: {"tra": True, "cpl": "A1M", "vdiv": 0.05, "offset": 0.0, "attn": 1.0, "bwl": True},
            2: {"tra": True, "cpl": "D1M", "vdiv": 1.00, "offset": 0.0, "attn": 10.0, "bwl": True},
        }
        self._frame_phase = 0.25
        self._cached_raw_by_ch: dict[int, bytes] = {}

    def _generate_channel_waveform(self, ch: int) -> tuple[bytes, bytes]:
        cfg = self.ch_cfg[ch]
        n = self.mdepth_pts
        total_t = self.tdiv * 10.0
        dt = total_t / float(n)
        t0 = -(total_t / 2.0) + self.delay
        t = t0 + np.arange(n, dtype=np.float64) * dt

        p_cmd_v = float(self.plant.pilot_v_cmd)
        v_src_pk = p_cmd_v * 10.0
        omega = 2.0 * math.pi * 1000.0
        phi0 = self._frame_phase
        h_des = design_pilot_transfer(1000.0)
        gain_des = abs(h_des)
        phase_des = cmath.phase(h_des)

        if ch == 2:
            v_dc = 2.5 if cfg["cpl"] == "D1M" else 0.0
            v = v_dc + v_src_pk * (
                np.cos(omega * t + phi0)
                + 10 ** (-68.0 / 20.0) * np.cos(2.0 * omega * t + 0.4)
                + 10 ** (-74.0 / 20.0) * np.cos(3.0 * omega * t + 0.9)
            )
            noise_std = 2e-4
        else:
            v_dc = float(self.plant.bias_v) if cfg["cpl"] == "D1M" else 0.0
            v_va_pk = v_src_pk * gain_des
            v = v_dc + v_va_pk * (
                np.cos(omega * t + phi0 + phase_des)
                + 10 ** (-56.0 / 20.0) * np.cos(2.0 * omega * t + 0.6)
                + 10 ** (-68.0 / 20.0) * np.cos(3.0 * omega * t + 1.1)
            )
            noise_std = 5e-5

        v = v + self.rng.normal(0.0, noise_std, size=n)
        code_per_div = 7680.0
        codes = np.clip(
            np.round((v + float(cfg["offset"])) / float(cfg["vdiv"]) * code_per_div),
            -32768,
            32767,
        ).astype("<i2")
        desc = build_wavedesc(
            point_num=n,
            vdiv=float(cfg["vdiv"]),
            offset=float(cfg["offset"]),
            code_per_div=code_per_div,
            adc_bit=16,
            interval=dt,
            delay=self.delay,
            tdiv_s=self.tdiv,
            probe=float(cfg["attn"]),
        )
        return desc, codes.tobytes()

    def sendall(self, data: bytes) -> None:
        cmd = data.decode("ascii").strip()
        upper = cmd.upper()
        if upper == ":SYSTEM:ERROR?":
            self._push_text('0,"No error"')
            return
        if upper == "TRMD AUTO" or upper == ":TRIGGER:RUN":
            super().sendall(data)
            self._frame_phase = float(self.rng.uniform(-math.pi, math.pi))
            return
        if upper.startswith("C1:") or upper.startswith("C2:"):
            ch = int(upper[1])
            body = upper[3:]
            if body.startswith("TRA "):
                self.ch_cfg[ch]["tra"] = body.split()[-1] == "ON"
                return
            if body.startswith("CPL "):
                self.ch_cfg[ch]["cpl"] = body.split()[-1]
                return
            if body.startswith("ATTN "):
                self.ch_cfg[ch]["attn"] = float(cmd.split()[-1])
                return
            if body.startswith("VDIV "):
                self.ch_cfg[ch]["vdiv"] = float(cmd.split()[-1])
                return
            if body.startswith("OFST "):
                self.ch_cfg[ch]["offset"] = float(cmd.split()[-1])
                return
        if upper.startswith("BWL "):
            parts = upper.split()[-1].split(",")
            ch = int(parts[0][1:])
            self.ch_cfg[ch]["bwl"] = parts[1] == "ON"
            return
        if upper.startswith(":WAVEFORM:SOURCE "):
            src = upper.split()[-1]
            self.wav_source = int(src[1:])
            return
        if upper == ":WAVEFORM:PREAMBLE?":
            desc, raw = self._generate_channel_waveform(self.wav_source)
            self._cached_raw_by_ch[self.wav_source] = raw
            self._push_binary(desc)
            return
        if upper == ":WAVEFORM:DATA?":
            if self.wav_source not in self._cached_raw_by_ch:
                _, raw = self._generate_channel_waveform(self.wav_source)
                self._cached_raw_by_ch[self.wav_source] = raw
            raw = self._cached_raw_by_ch[self.wav_source]
            hdr = f"#9{len(raw):09d}".encode("ascii")
            self._rx_buf.append(hdr + raw + b"\n\n")
            return
        super().sendall(data)


def generate_c1_pilot_phase_plot(summary: dict[str, Any], out_png: Path) -> Path:
    """Render 3-panel plot: (a) 2-cycle waveform phase lag, (b) phase/gain vs bias, (c) phase/THD vs amplitude."""
    trace = summary["representative_trace"]
    rows = summary["sweep_rows"]
    ac_bias_rows = [r for r in rows if r["group"] == "ac_bias_sweep"]
    dc_159_row = next(r for r in rows if r["group"] == "dc_ref" and abs(r["pilot_mv"] - 159.2) < 0.5)
    amp_rows = sorted(
        [
            next(r for r in rows if r["group"] == "ac_bias_sweep" and abs(r["bias_mv"]) < 1.0)
            if abs(p - 159.2) < 0.5
            else next(r for r in rows if r["group"] == "ac_amp_sweep" and abs(r["pilot_mv"] - p) < 0.5)
            for p in (84.9, 130.0, 150.0, 159.2)
        ],
        key=lambda r: r["pilot_mv"],
    )
    h_des = design_pilot_transfer(1000.0)
    des_phase_deg = math.degrees(cmath.phase(-h_des))

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(13.8, 4.1), constrained_layout=True)

    # Panel A: 2-cycle normalized -V_C48.1 vs V_VA (0 V, 159.2 mV, DC-coupled)
    t_ms = np.asarray(trace["t_ms"], dtype=np.float64)
    va_mv = np.asarray(trace["va_mv"], dtype=np.float64)
    src_ac_v = np.asarray(trace["src_v"], dtype=np.float64) - float(dc_159_row["ch2_dc_mean_v"])
    va_norm = (va_mv - float(np.mean(va_mv))) / max(float(dc_159_row["ch1_va_pk_mv"]), 1e-6)
    src_inv_norm = -src_ac_v / max(float(dc_159_row["ch2_src_pk_mv"]) * 1e-3, 1e-6)
    axes[0].plot(t_ms, src_inv_norm, color="#555555", ls="--", lw=1.4, label="-V(C48.1) AC norm")
    axes[0].plot(t_ms, va_norm, color="#1f4e78", lw=1.5, label="V(VA) norm")
    axes[0].set_xlabel("Time (ms)")
    axes[0].set_ylabel("Normalized AC Amplitude")
    axes[0].set_ylim(-1.25, 1.25)
    axes[0].set_title(
        f"2-Cycle DC-Coupled Overlay (0 V, 1.592 V src)\n"
        f"(Phase={dc_159_row['extra_phase_deg_mean']:+.2f} deg, Lag={dc_159_row['extra_delay_us_mean']:.2f} us)"
    )
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Panel B: Extra phase shift and direct gain across -8..+8 V DC bias
    biases_v = [r["bias_mv"] * 1e-3 for r in ac_bias_rows]
    ax1 = axes[1]
    ax1_r = ax1.twinx()
    l1 = ax1.errorbar(
        biases_v,
        [r["extra_phase_deg_mean"] for r in ac_bias_rows],
        yerr=[r["extra_phase_deg_std"] for r in ac_bias_rows],
        fmt="o-",
        capsize=3,
        color="#1f4e78",
        label="AC extra phase (deg)",
    )
    l2 = ax1.plot(
        [0.0],
        [dc_159_row["extra_phase_deg_mean"]],
        "*",
        ms=10,
        color="#b2182b",
        label="DC ref 0 V (deg)",
    )
    l3 = ax1.axhline(des_phase_deg, color="#1f4e78", ls=":", lw=1.1, label=f"Design ({des_phase_deg:.2f} deg)")
    l4 = ax1_r.plot(
        biases_v,
        [r["gain_direct_mean"] for r in ac_bias_rows],
        "s--",
        color="#d95f02",
        label="Direct |V_VA/V_C48.1|",
    )
    ax1.set_xlabel("DC Bias VA (V)")
    ax1.set_ylabel("Extra Phase Relative to -1 (deg)", color="#1f4e78")
    ax1_r.set_ylabel("Direct Gain |V_VA / V_C48.1|", color="#d95f02")
    ax1_r.set_ylim(0.088, 0.100)
    ax1.set_title("Phase & Gain Flatness vs DC Bias (159.2 mV)")
    handles = [l1, l2[0], l3, l4[0]]
    labels = [h.get_label() for h in handles]
    ax1.legend(handles, labels, loc="lower left", fontsize=7.5)
    ax1.grid(True, alpha=0.3)

    # Panel C: Extra phase & THD vs pilot command
    pilots = [r["pilot_mv"] for r in amp_rows]
    ax2 = axes[2]
    ax2_r = ax2.twinx()
    p1 = ax2.errorbar(
        pilots,
        [r["extra_phase_deg_mean"] for r in amp_rows],
        yerr=[r["extra_phase_deg_std"] for r in amp_rows],
        fmt="o-",
        capsize=3,
        color="#1b7837",
        label="Extra phase (deg)",
    )
    p2 = ax2_r.plot(pilots, [r["ch1_thd_2_10_pct_mean"] for r in amp_rows], "d--", color="#7570b3", label="VA THD (%)")
    p3 = ax2_r.plot(pilots, [r["ch2_thd_2_10_pct_mean"] for r in amp_rows], "^:", color="#666666", label="C48.1 THD (%)")
    ax2.set_xlabel("Pilot Command (mV)")
    ax2.set_ylabel("Extra Phase Relative to -1 (deg)", color="#1b7837")
    ax2_r.set_ylabel("THD (2..10, %)", color="#7570b3")
    ax2.set_title("Phase Linearity & Source vs VA THD (0 V, AC)")
    h2 = [p1, p2[0], p3[0]]
    ax2.legend(h2, [h.get_label() for h in h2], loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    return out_png


def run_c1_pilot_phase(
    uart: UARTClient,
    scope: SDS824XClient,
    session: RunSession,
    out_dir: Path,
    *,
    dry_run: bool = False,
    n_frames: int = 6,
    ch1_attn: int = 1,
    ch2_attn: int = 10,
) -> dict[str, Any]:
    """Run C1 dual-channel C48.1 -> VA 1 kHz pilot transfer & phase measurement."""
    ident = scope.identify()
    st0 = uart.status()
    if st0.state != "IDLE" or st0.drops != 0 or st0.crc_errs != 0:
        raise RuntimeError(f"Board preflight failed: {st0}")
    hw = uart.hw_info()

    sweep_plan: list[tuple[str, str, float, float]] = [
        # Group 1: DC-coupled zero-AC-skew reference
        ("dc_ref", "DC", 0.0, 159.2),
        ("dc_ref", "DC", 0.0, 150.0),
        # Group 2: AC-coupled cross-bias sweep
        ("ac_bias_sweep", "AC", -8000.0, 159.2),
        ("ac_bias_sweep", "AC", -4000.0, 159.2),
        ("ac_bias_sweep", "AC", 0.0, 159.2),
        ("ac_bias_sweep", "AC", 4000.0, 159.2),
        ("ac_bias_sweep", "AC", 8000.0, 159.2),
        # Group 3: AC-coupled amplitude sweep at 0 V
        ("ac_amp_sweep", "AC", 0.0, 84.9),
        ("ac_amp_sweep", "AC", 0.0, 130.0),
        ("ac_amp_sweep", "AC", 0.0, 150.0),
    ]

    sweep_rows: list[dict[str, Any]] = []
    representative_trace: dict[str, list[float]] | None = None

    try:
        scope.set_mdepth("100K")
        scope.setup_timebase(2e-3, delay_s=0.0)
        scope.setup_edge_trigger(source="C2", slope="RISing", level=0.0, coupling="AC")

        prev_cpl: str | None = None
        for idx, (group, coupling, bias_mv, pilot_mv) in enumerate(sweep_plan):
            if prev_cpl != coupling:
                c2_offset = -2.5 if coupling == "DC" else 0.0
                scope.setup_channel(1, display=True, coupling=coupling, scale=0.05, offset=0.0, attn=ch1_attn, bwl=True)
                scope.setup_channel(2, display=True, coupling=coupling, scale=0.5, offset=c2_offset, attn=ch2_attn, bwl=True)
                prev_cpl = coupling

            uart.meas(bias_mv, pilot_mv, 20 if idx == 0 else 15, 1)

            frame_stats: list[dict[str, Any]] = []
            for frame_idx in range(n_frames):
                (t1, v1, _m1), (t2, v2, _m2) = scope.capture_dual_auto_waveforms(
                    1, 2, settle_s=0.35 if dry_run else 0.40
                )
                stat = analyze_dual_channel_transfer(t1, v1, t2, v2)
                if idx == 0 and frame_idx == 0:
                    if not (2.0 <= stat["ch2_dc_mean_v"] <= 3.0) or not (1350.0 <= stat["ch2_src_pk_mv"] <= 1850.0):
                        raise RuntimeError(
                            f"DC C48.1 sanity check failed (check C2 10x probe/ATTN=10): "
                            f"ch2_dc_mean_v={stat['ch2_dc_mean_v']:.3f} V, ch2_src_pk_mv={stat['ch2_src_pk_mv']:.1f} mV"
                        )
                    # Capture 2 ms representative window decimated 10x (1000 points) for Panel A.
                    dt_s = float(t1[1] - t1[0])
                    step = max(1, int(round(2e-6 / dt_s)))
                    n_win = min(len(t1), int(round(0.002 / dt_s)))
                    t_rel_ms = (t1[:n_win:step] - float(t1[0])) * 1e3
                    representative_trace = {
                        "t_ms": [float(x) for x in t_rel_ms],
                        "va_mv": [float(x * 1e3) for x in v1[:n_win:step]],
                        "src_v": [float(x) for x in v2[:n_win:step]],
                    }
                frame_stats.append(stat)

            sweep_rows.append(
                {
                    "group": group,
                    "coupling": coupling,
                    "bias_mv": bias_mv,
                    "pilot_mv": pilot_mv,
                    "n_frames": n_frames,
                    "f0_hz_mean": float(np.mean([f["f0_hz"] for f in frame_stats])),
                    "ch1_dc_mean_v": float(np.mean([f["ch1_dc_mean_v"] for f in frame_stats])),
                    "ch2_dc_mean_v": float(np.mean([f["ch2_dc_mean_v"] for f in frame_stats])),
                    "ch1_va_pk_mv": float(np.mean([f["ch1_va_pk_mv"] for f in frame_stats])),
                    "ch2_src_pk_mv": float(np.mean([f["ch2_src_pk_mv"] for f in frame_stats])),
                    "gain_direct_mean": float(np.mean([f["gain_direct"] for f in frame_stats])),
                    "gain_direct_std": float(np.std([f["gain_direct"] for f in frame_stats], ddof=1)) if n_frames > 1 else 0.0,
                    "gain_err_vs_design_pct": float(np.mean([f["gain_err_vs_design_pct"] for f in frame_stats])),
                    "total_phase_deg_mean": float(np.mean([f["total_phase_deg"] for f in frame_stats])),
                    "extra_phase_deg_mean": float(np.mean([f["extra_phase_deg"] for f in frame_stats])),
                    "extra_phase_deg_std": float(np.std([f["extra_phase_deg"] for f in frame_stats], ddof=1)) if n_frames > 1 else 0.0,
                    "phase_err_vs_design_deg": float(np.mean([f["phase_err_vs_design_deg"] for f in frame_stats])),
                    "extra_delay_us_mean": float(np.mean([f["extra_delay_us"] for f in frame_stats])),
                    "ch1_thd_2_10_pct_mean": float(np.mean([f["ch1_thd_2_10_pct"] for f in frame_stats])),
                    "ch2_thd_2_10_pct_mean": float(np.mean([f["ch2_thd_2_10_pct"] for f in frame_stats])),
                    "ch1_h2_dbc_mean": float(np.mean([f["ch1_h2_dbc"] for f in frame_stats])),
                    "ch2_h2_dbc_mean": float(np.mean([f["ch2_h2_dbc"] for f in frame_stats])),
                }
            )
    finally:
        uart.stop()
        st_end = uart.status()
        if st_end.state != "IDLE" or st_end.bias_mv != 0 or st_end.drops != 0 or st_end.crc_errs != 0:
            raise RuntimeError(f"Board did not return safely to IDLE zero bias: {st_end}")

    if representative_trace is None:
        raise RuntimeError("Missing representative dual-channel DC trace")

    def _find(group: str, bias_mv: float, pilot_mv: float) -> dict[str, Any]:
        return next(
            r for r in sweep_rows
            if r["group"] == group and abs(r["bias_mv"] - bias_mv) < 0.5 and abs(r["pilot_mv"] - pilot_mv) < 0.5
        )

    dc_159 = _find("dc_ref", 0.0, 159.2)
    dc_150 = _find("dc_ref", 0.0, 150.0)
    ac_0_159 = _find("ac_bias_sweep", 0.0, 159.2)
    ac_0_150 = _find("ac_amp_sweep", 0.0, 150.0)

    ac_bias_rows = [r for r in sweep_rows if r["group"] == "ac_bias_sweep"]
    phases_bias = [r["extra_phase_deg_mean"] for r in ac_bias_rows]
    gains_bias = [r["gain_direct_mean"] for r in ac_bias_rows]
    amp_rows = [_find("ac_amp_sweep", 0.0, 84.9), _find("ac_amp_sweep", 0.0, 130.0), ac_0_150, ac_0_159]
    phases_amp = [r["extra_phase_deg_mean"] for r in amp_rows]
    gains_amp = [r["gain_direct_mean"] for r in amp_rows]

    h_design_1k = design_pilot_transfer(1000.0)
    design_extra_deg = math.degrees(cmath.phase(-h_design_1k))
    design_total_deg = math.degrees(cmath.phase(h_design_1k))

    metadata = capture_metadata(
        "c1_pilot_phase",
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
        notes=f"MZM disconnected; SDS824X HD CH1 (ATTN={ch1_attn})->CN1 VA, CH2 (ATTN={ch2_attn})->C48.1 (DITHER1)",
    )

    summary = {
        "metadata": metadata,
        "scope_idn": ident,
        "design_references": {
            "gain_1khz": abs(h_design_1k),
            "total_phase_deg": design_total_deg,
            "extra_phase_deg": design_extra_deg,
            "extra_delay_us": (-design_extra_deg / 360.0) / 1000.0 * 1e6,
            "r_inj_ohm": R_INJ_OHM,
            "c_inj_f": C_INJ_F,
            "r_fb_ohm": R_FB_OHM,
            "c_fb_f": C_FB_F,
        },
        "dc_reference": {
            "pilot_159_2mv": dc_159,
            "pilot_150_0mv": dc_150,
        },
        "ac_vs_dc_skew": {
            "pilot_159_2mv_delta_phase_deg": ac_0_159["extra_phase_deg_mean"] - dc_159["extra_phase_deg_mean"],
            "pilot_159_2mv_delta_gain_pct": (ac_0_159["gain_direct_mean"] - dc_159["gain_direct_mean"]) / dc_159["gain_direct_mean"] * 100.0,
            "pilot_150_0mv_delta_phase_deg": ac_0_150["extra_phase_deg_mean"] - dc_150["extra_phase_deg_mean"],
            "pilot_150_0mv_delta_gain_pct": (ac_0_150["gain_direct_mean"] - dc_150["gain_direct_mean"]) / dc_150["gain_direct_mean"] * 100.0,
        },
        "cross_bias_summary": {
            "extra_phase_deg_mean": float(np.mean(phases_bias)),
            "extra_phase_deg_ptp": float(max(phases_bias) - min(phases_bias)),
            "extra_phase_deg_std": float(np.std(phases_bias, ddof=1)),
            "gain_direct_mean": float(np.mean(gains_bias)),
            "gain_ptp_pct": (max(gains_bias) - min(gains_bias)) / float(np.mean(gains_bias)) * 100.0,
            "extra_delay_us_mean": float(np.mean([r["extra_delay_us_mean"] for r in ac_bias_rows])),
        },
        "amplitude_sweep_summary": {
            "extra_phase_deg_min": float(min(phases_amp)),
            "extra_phase_deg_max": float(max(phases_amp)),
            "extra_phase_deg_ptp": float(max(phases_amp) - min(phases_amp)),
            "gain_direct_min": float(min(gains_amp)),
            "gain_direct_max": float(max(gains_amp)),
            "ch1_va_thd_max_pct": float(max(r["ch1_thd_2_10_pct_mean"] for r in amp_rows)),
            "ch2_src_thd_max_pct": float(max(r["ch2_thd_2_10_pct_mean"] for r in amp_rows)),
        },
        "sweep_rows": sweep_rows,
        "representative_trace": representative_trace,
    }

    csv_fields = list(sweep_rows[0].keys())
    session.write_metadata(metadata)
    session.write_json("summary.json", summary)
    _write_csv(session.run_dir / "c1_pilot_phase_sweep.csv", csv_fields, sweep_rows)

    out_dir.mkdir(parents=True, exist_ok=True)
    archive_summary = {k: v for k, v in summary.items() if k != "representative_trace"}
    atomic_write_json(out_dir / "c1_pilot_phase_summary.json", archive_summary)
    _write_csv(out_dir / "c1_pilot_phase_sweep.csv", csv_fields, sweep_rows)
    generate_c1_pilot_phase_plot(summary, out_dir / "c1_pilot_phase_char.png")
    return archive_summary


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    p.add_argument("--scope-host", default=DEFAULT_SCOPE_HOST)
    p.add_argument("--scope-port", type=int, default=DEFAULT_SCOPE_PORT)
    p.add_argument("--run-dir", default=str(repo_root() / ".agent-bridge" / "exp_runs" / "c1_pilot_phase_live"))
    p.add_argument("--out-dir", default=str(repo_root() / "docs" / "experiments" / "data" / "c1_c2"))
    p.add_argument("--frames", type=int, default=6)
    p.add_argument("--ch1-attn", type=int, default=1)
    p.add_argument("--ch2-attn", type=int, default=10)
    p.add_argument("--dry-run", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    session = RunSession("c1_pilot_phase", run_dir=Path(args.run_dir))

    if args.dry_run:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261128))
        uart_ctx = UARTClient(transport=SimulatedUARTTransport(plant))
        scope_ctx = SDS824XClient(
            host=args.scope_host,
            port=args.scope_port,
            cmd_sleep_s=0.0,
            waveform_sleep_s=0.0,
            settle_scale=0.0,
            socket_factory=lambda *a, **kw: SimulatedDualSDS824XSocket(plant),
        )
    else:
        uart_ctx = UARTClient(port=args.port)
        scope_ctx = SDS824XClient(host=args.scope_host, port=args.scope_port)

    with uart_ctx as uart, scope_ctx as scope:
        run_c1_pilot_phase(
            uart,
            scope,
            session,
            Path(args.out_dir),
            dry_run=args.dry_run,
            n_frames=args.frames,
            ch1_attn=args.ch1_attn,
            ch2_attn=args.ch2_attn,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
