"""Consolidate C1-C6 experiment runs into tables and publication-grade matplotlib figures."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .common import atomic_write_json, repo_root


def _load_summary(run_dir: Path) -> dict[str, Any]:
    summary_path = run_dir / "summary.json"
    if not summary_path.is_file():
        raise FileNotFoundError(f"Missing summary.json in {run_dir}")
    return json.loads(summary_path.read_text(encoding="utf-8"))


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def analyze_c1_c2(run_dir: Path, out_dir: Path) -> dict[str, Any]:
    data = _load_summary(run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Dark CH0 FFT plot (0-32 kHz)
    dump = data["c1_dark_dump"]
    twait = data["c2_twait_scan"]
    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.0), constrained_layout=True)

    freqs_khz = np.asarray(dump["fft_freqs_hz"], dtype=float) * 1e-3
    mags_uv = np.asarray(dump["fft_mag_uv_pk"], dtype=float)
    axes[0].semilogy(freqs_khz[1:], np.maximum(mags_uv[1:], 1e-3), color="#1f4e78", lw=1.2)
    axes[0].set_xlabel("Frequency (kHz)")
    axes[0].set_ylabel("CH0 Dark Spur Peak (uV)")
    axes[0].set_title("C1 Dark CH0 FFT (0-32 kHz, 64 kSPS)")
    axes[0].grid(True, alpha=0.3)

    curve = twait["curve"]
    waits = [pt["wait_ms"] for pt in curve]
    errs = [max(pt["mean_abs_phase_err_deg"], 1e-4) for pt in curve]
    axes[1].semilogy(waits, errs, "o-", color="#1b7837", lw=1.6)
    axes[1].axhline(0.2, color="#b2182b", ls="--", lw=1.2, label="0.2 deg threshold")
    axes[1].set_xlabel("Post-Step T_wait (ms)")
    axes[1].set_ylabel("First-Window Error (deg)")
    axes[1].set_title("C2 Step Recovery T_wait Scan")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    fig_path = out_dir / "c1_c2_fft_and_twait.png"
    fig.savefig(fig_path, dpi=200)
    plt.close(fig)

    step_rows = list(data["c2_step_peaks"].values())
    if step_rows:
        _write_csv(out_dir / "c2_step_peaks_summary.csv", list(step_rows[0].keys()), step_rows)
    atomic_write_json(out_dir / "c1_c2_summary.json", data)
    return {"figure": str(fig_path), "summary": str(out_dir / "c1_c2_summary.json")}


def analyze_c3(run_dir: Path, out_dir: Path) -> dict[str, Any]:
    data = _load_summary(run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    sessions = data["sessions"]

    fig, ax = plt.subplots(figsize=(6.5, 4.0), constrained_layout=True)
    idx = np.arange(len(sessions))
    w = 0.25
    ax.bar(idx - w, [s["labeled_cv_rms_mrad"] for s in sessions], width=w, label="Labeled 2D CV")
    ax.bar(idx, [s["ellipse_dc_cv_rms_mrad"] for s in sessions], width=w, label="Ellipse+DC CV")
    ax.bar(idx + w, [s["diagonal_cv_rms_mrad"] for s in sessions], width=w, label="Diagonal 2D CV")
    ax.set_xticks(idx, [f"Session {s['session'] + 1}" for s in sessions])
    ax.set_ylabel("5-Fold Blocked CV RMS (mrad)")
    ax.set_title("C3 Calibration Generalization Comparison")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)

    fig_path = out_dir / "c3_cv_comparison.png"
    fig.savefig(fig_path, dpi=200)
    plt.close(fig)

    _write_csv(out_dir / "c3_sessions.csv", list(sessions[0].keys()), sessions)
    atomic_write_json(out_dir / "c3_summary.json", data)
    return {"figure": str(fig_path), "summary": str(out_dir / "c3_summary.json")}


def analyze_c4(run_dir: Path, out_dir: Path) -> dict[str, Any]:
    data = _load_summary(run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    methods = data["method_summaries"]

    rows = []
    for m_name, m_val in methods.items():
        s16 = m_val["equidistant_16"]
        rows.append(
            {
                "method": m_name,
                "static_rms_mrad": round(s16["static_rms_mrad"], 3),
                "max_static_abs_deg": round(s16["max_static_abs_deg"], 4),
                "temporal_rms_mrad": round(s16["temporal_rms_mrad"], 3),
                "worst_target_p95_deg": round(s16["worst_target_p95_deg"], 4),
                "max_a2_mrad": round(s16["max_two_period_half_amp_mrad"], 3),
                "pass_1deg": f"{s16['pass_count']}/{s16['n_total']}",
            }
        )
    _write_csv(out_dir / "c4_table_5_5.csv", list(rows[0].keys()), rows)

    fig, ax = plt.subplots(figsize=(7.0, 4.2), constrained_layout=True)
    names = [r["method"] for r in rows]
    vals = [max(r["static_rms_mrad"], 1e-3) for r in rows]
    ax.semilogy(names, vals, "o-", lw=1.8, color="#1f4e78")
    ax.axhline(17.453, color="#b2182b", ls="--", lw=1.2, label="1 deg (17.45 mrad)")
    ax.set_ylabel("16-Target Static RMS (mrad)")
    ax.set_title("C4 Head-to-Head Closed-Loop Comparison")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig_path = out_dir / "c4_method_comparison.png"
    fig.savefig(fig_path, dpi=200)
    plt.close(fig)
    atomic_write_json(out_dir / "c4_summary.json", data)
    return {"figure": str(fig_path), "table": str(out_dir / "c4_table_5_5.csv")}


def analyze_c6(run_dir: Path, out_dir: Path) -> dict[str, Any]:
    data = _load_summary(run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    states = data["states"]

    rows = []
    for label, st in states.items():
        row: dict[str, Any] = {
            "rf_state": label,
            "h1_rel_amp": round(st["h1_rel_amp"], 4),
            "kappa": round(st["kappa"], 2),
            "sigma_min_mv": round(st["sigma_min_mv"], 3),
        }
        for m_name, m_stats in st["methods"].items():
            row[f"{m_name}_p95_deg"] = round(m_stats["worst_target_p95_deg"], 4)
            row[f"{m_name}_rms_mrad"] = round(m_stats["temporal_rms_mrad"], 3)
        rows.append(row)
    if rows:
        _write_csv(out_dir / "c6_table_5_7.csv", list(rows[0].keys()), rows)

    fig, ax = plt.subplots(figsize=(7.5, 4.2), constrained_layout=True)
    x = np.arange(len(rows))
    labels = [r["rf_state"] for r in rows]
    for m_name in next(iter(states.values()))["methods"].keys():
        y = [max(states[lb]["methods"][m_name]["worst_target_p95_deg"], 1e-4) for lb in labels]
        ax.semilogy(x, y, "o-", lw=1.5, label=m_name)
    ax.axhline(1.0, color="#b2182b", ls="--", lw=1.2, label="1 deg threshold")
    ax.set_xticks(x, labels, rotation=20)
    ax.set_ylabel("Worst-Target P95 (deg)")
    ax.set_title("C6 RF Loading Robustness")
    ax.legend(ncol=2, fontsize=8)
    ax.grid(True, alpha=0.3)

    fig_path = out_dir / "c6_rf_robustness.png"
    fig.savefig(fig_path, dpi=200)
    plt.close(fig)
    atomic_write_json(out_dir / "c6_summary.json", data)
    return {"figure": str(fig_path), "table": str(out_dir / "c6_table_5_7.csv")}


def analyze_c5(run_dir: Path, out_dir: Path) -> dict[str, Any]:
    data = _load_summary(run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for tgt_label, runs in data["long_term"].items():
        for r in runs:
            rows.append(r)
    if rows:
        _write_csv(out_dir / "c5_long_term.csv", list(rows[0].keys()), rows)

    fig, ax = plt.subplots(figsize=(6.5, 4.0), constrained_layout=True)
    labels = list(data["long_term"].keys())
    p95_means = [float(np.mean([r["ext_p95_deg"] for r in data["long_term"][lb]])) for lb in labels]
    ax.bar(labels, p95_means, color="#2166ac")
    ax.axhline(1.0, color="#b2182b", ls="--", lw=1.2, label="1 deg threshold")
    ax.set_ylabel("External DC PSI P95 (deg)")
    ax.set_title("C5 Long-Term Autonomous Stability")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)

    fig_path = out_dir / "c5_stability.png"
    fig.savefig(fig_path, dpi=200)
    plt.close(fig)
    atomic_write_json(out_dir / "c5_summary.json", data)
    return {"figure": str(fig_path), "table": str(out_dir / "c5_long_term.csv")}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze C1-C6 experiment runs and generate tables/figures")
    parser.add_argument("--exp", required=True, choices=("c1_c2", "c3", "c4", "c6", "c5"))
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args(argv)

    out_dir = args.out_dir if args.out_dir is not None else (repo_root() / "docs" / "experiments" / "data" / args.exp)
    dispatch = {
        "c1_c2": analyze_c1_c2,
        "c3": analyze_c3,
        "c4": analyze_c4,
        "c6": analyze_c6,
        "c5": analyze_c5,
    }
    res = dispatch[args.exp](args.run_dir, out_dir)
    print(json.dumps(res, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
