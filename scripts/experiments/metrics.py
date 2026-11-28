"""Circular statistics and control metrics matching ch02.tex and ch05.tex."""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np

from .common import wrap_rad

DEFAULT_BAND_1DEG_RAD = float(np.deg2rad(1.0))
DEFAULT_BAND_20MRAD = 0.020


def _validate_1d_finite(values: Sequence[float] | np.ndarray, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.ndim != 1 or arr.size == 0:
        raise ValueError(f"{name} must be a non-empty 1D sequence")
    if np.any(~np.isfinite(arr)):
        raise ValueError(f"{name} contains non-finite values")
    return arr


def circular_mean(errors_rad: Sequence[float] | np.ndarray) -> float:
    """Circular mean arg(mean(exp(j * e))) in (-pi, pi]."""
    arr = _validate_1d_finite(errors_rad, "errors_rad")
    phasor = np.mean(np.exp(1j * arr))
    return float(wrap_rad(float(np.angle(phasor))))


def circular_rms(errors_rad: Sequence[float] | np.ndarray) -> float:
    """RMS of wrapped circular errors in (-pi, pi]."""
    arr = np.asarray(wrap_rad(_validate_1d_finite(errors_rad, "errors_rad")), dtype=float)
    return float(np.sqrt(np.mean(arr ** 2)))


def temporal_rms(errors_rad: Sequence[float] | np.ndarray) -> float:
    """Time-domain RMS retaining static offset (E_time in ch02.tex Eq. 2.25)."""
    return circular_rms(errors_rad)


def short_term_sigma(errors_rad: Sequence[float] | np.ndarray) -> float:
    """Short-term standard deviation after removing circular mean (sigma_time)."""
    arr = _validate_1d_finite(errors_rad, "errors_rad")
    mu = circular_mean(arr)
    centered = np.asarray(wrap_rad(arr - mu), dtype=float)
    return float(np.sqrt(np.mean(centered ** 2)))


def p95_abs(errors_rad: Sequence[float] | np.ndarray) -> float:
    """95th percentile of |wrap(e)| in radians."""
    arr = np.asarray(wrap_rad(_validate_1d_finite(errors_rad, "errors_rad")), dtype=float)
    return float(np.quantile(np.abs(arr), 0.95))


def two_period_half_amp(errors_rad: Sequence[float] | np.ndarray) -> float:
    """Two-period oscillation half-amplitude A2 = |wrap(e_odd - e_even)| / 2."""
    arr = _validate_1d_finite(errors_rad, "errors_rad")
    if arr.size < 2:
        raise ValueError("two_period_half_amp requires at least 2 samples")
    odd_mean = circular_mean(arr[0::2])
    even_mean = circular_mean(arr[1::2])
    return 0.5 * abs(float(wrap_rad(odd_mean - even_mean)))


def static_rms(
    per_target_tails: Sequence[Sequence[float] | np.ndarray],
    *,
    failed_mask: Sequence[bool] | None = None,
    failure_penalty_rad: float = np.pi,
) -> dict[str, float | int]:
    """Cross-target static RMS preserving the full K-target denominator.

    If a target has empty samples and is marked failed in ``failed_mask``, its
    circular mean contribution uses ``failure_penalty_rad`` (pi rad) so failed
    targets are never silently dropped from the denominator.
    """
    n_total = len(per_target_tails)
    if n_total == 0:
        raise ValueError("per_target_tails must be non-empty")
    if failed_mask is not None and len(failed_mask) != n_total:
        raise ValueError("failed_mask length must match per_target_tails")

    means = np.empty(n_total, dtype=float)
    n_failed = 0
    for idx, seq in enumerate(per_target_tails):
        is_failed = bool(failed_mask[idx]) if failed_mask is not None else False
        arr = np.asarray(seq, dtype=float)
        if arr.size == 0:
            if not is_failed:
                raise ValueError(f"Target {idx} has empty tail samples without failure flag")
            means[idx] = float(failure_penalty_rad)
            n_failed += 1
        else:
            if is_failed:
                n_failed += 1
            means[idx] = circular_mean(arr)

    rms = float(np.sqrt(np.mean(means ** 2)))
    max_abs = float(np.max(np.abs(means)))
    return {
        "static_rms_rad": rms,
        "static_rms_mrad": rms * 1e3,
        "static_rms_deg": float(np.degrees(rms)),
        "max_static_abs_rad": max_abs,
        "max_static_abs_deg": float(np.degrees(max_abs)),
        "n_total": n_total,
        "n_failed": n_failed,
        "n_valid": n_total - n_failed,
    }


def cond_kappa_and_sigma_min(matrix: np.ndarray) -> tuple[float, float]:
    """Return (condition_number_kappa, sigma_min) of a 2x2 matrix."""
    mat = np.asarray(matrix, dtype=float)
    if mat.shape != (2, 2):
        raise ValueError(f"Expected 2x2 matrix, got shape {mat.shape}")
    if np.any(~np.isfinite(mat)):
        raise ValueError("Matrix contains non-finite values")
    svals = np.linalg.svd(mat, compute_uv=False)
    sigma_max = float(svals[0])
    sigma_min = float(svals[-1])
    kappa = float("inf") if sigma_min <= 0.0 else sigma_max / sigma_min
    return kappa, sigma_min


def settling_time_steps(
    errors_rad: Sequence[float] | np.ndarray,
    *,
    threshold_rad: float,
    sustain_steps: int = 10,
    dt_s: float | None = None,
    timestamps_s: Sequence[float] | np.ndarray | None = None,
) -> tuple[int | None, float | None]:
    """Find first entry step where |wrap(e)| <= threshold_rad for sustain_steps consecutively.

    Returns (entry_step_index, settling_time_s), or (None, None) if never settled.
    """
    arr = np.abs(np.asarray(wrap_rad(_validate_1d_finite(errors_rad, "errors_rad")), dtype=float))
    if threshold_rad <= 0:
        raise ValueError("threshold_rad must be positive")
    if sustain_steps < 1:
        raise ValueError("sustain_steps must be >= 1")
    if arr.size < sustain_steps:
        return None, None

    inside = arr <= threshold_rad
    # Slide a window of sustain_steps consecutive True values.
    entry_idx: int | None = None
    for idx in range(0, arr.size - sustain_steps + 1):
        if bool(np.all(inside[idx : idx + sustain_steps])):
            entry_idx = idx
            break

    if entry_idx is None:
        return None, None

    if timestamps_s is not None:
        ts = _validate_1d_finite(timestamps_s, "timestamps_s")
        if ts.size != arr.size:
            raise ValueError("timestamps_s length must match errors_rad")
        t_settle = float(ts[entry_idx] - ts[0])
    elif dt_s is not None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        t_settle = float(entry_idx * dt_s)
    else:
        t_settle = None
    return entry_idx, t_settle


def unwrapped_p2p_and_slope(
    timestamps_s: Sequence[float] | np.ndarray,
    values: Sequence[float] | np.ndarray,
    *,
    is_angle_rad: bool = False,
) -> dict[str, float]:
    """Compute endpoint net change, peak-to-peak span, and OLS linear drift slope (per hour)."""
    ts = _validate_1d_finite(timestamps_s, "timestamps_s")
    vals = _validate_1d_finite(values, "values")
    if ts.size != vals.size:
        raise ValueError("timestamps_s and values must have equal length")
    if is_angle_rad:
        vals = np.unwrap(vals)

    net_change = float(vals[-1] - vals[0])
    peak_to_peak = float(np.max(vals) - np.min(vals))
    if ts.size >= 2 and np.max(ts) > np.min(ts):
        t_hr = (ts - ts[0]) / 3600.0
        slope_per_hr = float(np.polyfit(t_hr, vals, 1)[0])
    else:
        slope_per_hr = 0.0

    return {
        "net_change": net_change,
        "peak_to_peak": peak_to_peak,
        "slope_per_hour": slope_per_hr,
    }


def paired_repeatability_stats(samples: Sequence[float] | np.ndarray) -> dict[str, float]:
    """Compute paired-difference single-window sigma and lag-1 correlation (for C2 240 pairs)."""
    arr = _validate_1d_finite(samples, "samples")
    if arr.size < 4 or arr.size % 2 != 0:
        raise ValueError("paired_repeatability_stats requires an even count >= 4")
    even_items = arr[0::2]
    odd_items = arr[1::2]
    diffs = odd_items - even_items
    s_delta = float(np.std(diffs, ddof=1))
    sigma_single = s_delta / float(np.sqrt(2.0))
    if np.std(even_items) > 0 and np.std(odd_items) > 0:
        corr = float(np.corrcoef(even_items, odd_items)[0, 1])
    else:
        corr = 0.0
    return {
        "n_pairs": float(diffs.size),
        "s_delta": s_delta,
        "sigma_single": sigma_single,
        "pair_corr": corr,
    }


def summarize_target_runs(
    per_target_tails: Sequence[Sequence[float] | np.ndarray],
    *,
    failed_mask: Sequence[bool] | None = None,
    pass_p95_threshold_rad: float = DEFAULT_BAND_1DEG_RAD,
) -> dict[str, float | int]:
    """Aggregate C4/C6 metrics across K targets while strictly preserving denominators."""
    n_total = len(per_target_tails)
    if n_total == 0:
        raise ValueError("per_target_tails must be non-empty")
    if failed_mask is not None and len(failed_mask) != n_total:
        raise ValueError("failed_mask length must match per_target_tails")

    st = static_rms(per_target_tails, failed_mask=failed_mask)
    all_samples: list[np.ndarray] = []
    p95_list: list[float] = []
    a2_list: list[float] = []
    passed = 0

    for idx, seq in enumerate(per_target_tails):
        is_failed = bool(failed_mask[idx]) if failed_mask is not None else False
        arr = np.asarray(seq, dtype=float)
        if arr.size == 0:
            p95_val = float(np.pi)
            a2_val = 0.0
            all_samples.append(np.array([float(np.pi)]))
        else:
            all_samples.append(arr)
            p95_val = p95_abs(arr)
            a2_val = two_period_half_amp(arr) if arr.size >= 2 else 0.0
        p95_list.append(p95_val)
        a2_list.append(a2_val)
        if not is_failed and p95_val <= pass_p95_threshold_rad:
            passed += 1

    pooled = np.concatenate(all_samples)
    t_rms = temporal_rms(pooled)
    t_p95 = p95_abs(pooled)
    worst_p95 = float(np.max(p95_list))
    max_a2 = float(np.max(a2_list))

    return {
        "static_rms_rad": st["static_rms_rad"],
        "static_rms_mrad": st["static_rms_mrad"],
        "static_rms_deg": st["static_rms_deg"],
        "max_static_abs_rad": st["max_static_abs_rad"],
        "max_static_abs_deg": st["max_static_abs_deg"],
        "temporal_rms_rad": t_rms,
        "temporal_rms_mrad": t_rms * 1e3,
        "temporal_rms_deg": float(np.degrees(t_rms)),
        "temporal_p95_rad": t_p95,
        "temporal_p95_deg": float(np.degrees(t_p95)),
        "worst_target_p95_rad": worst_p95,
        "worst_target_p95_deg": float(np.degrees(worst_p95)),
        "max_two_period_half_amp_rad": max_a2,
        "max_two_period_half_amp_mrad": max_a2 * 1e3,
        "pass_count": passed,
        "n_total": n_total,
        "n_failed": int(st["n_failed"]),
        "pass_rate": passed / n_total,
    }
