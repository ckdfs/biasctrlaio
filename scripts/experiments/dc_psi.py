"""In-Situ 17-point DC Phase-Shift Interferometry (DC PSI) reference solver."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.optimize import least_squares

from .common import thesis_to_fw_rad, wrap_rad
from .uart_client import UARTClient

DEFAULT_CLAMP_LOW_V = -9.8
DEFAULT_CLAMP_HIGH_V = 9.8
DEFAULT_INITIAL_JUMP_WAIT_BLOCKS = 12  # 240 ms
DEFAULT_STEP_WAIT_BLOCKS = 7           # 140 ms
DEFAULT_RETURN_WAIT_BLOCKS = 12        # 240 ms
DEFAULT_BLOCKS_PER_POINT = 3           # 60 ms averaging per DC point


@dataclass(frozen=True)
class DcPsiResult:
    """Result of 17-point In-Situ DC PSI measurement and 4-parameter NLSQ fit."""

    phi_true_thesis_rad: float
    phi_true_thesis_deg: float
    phi_true_fw_rad: float
    phi_true_fw_deg: float
    vpi_v: float
    dc_a_mv: float
    dc_b_mv: float
    fit_rmse_mv: float
    sigma_phi_deg: float
    drift_mv: float
    dc_pre_mv: float
    dc_post_mv: float
    v_lock_v: float
    v_start_v: float
    voltages_v: tuple[float, ...]
    dc_points_mv: tuple[float, ...]
    residuals_mv: tuple[float, ...]


def compute_scan_window(v_lock_v: float, vpi_est_v: float) -> tuple[float, np.ndarray]:
    """Compute clamped 17-point DC PSI sweep voltages spanning [V_start, V_start + 2*V_pi]."""
    if not np.isfinite(v_lock_v) or not np.isfinite(vpi_est_v) or vpi_est_v <= 0.0:
        raise ValueError(f"Invalid v_lock_v={v_lock_v} or vpi_est_v={vpi_est_v}")
    span_v = 2.0 * vpi_est_v
    max_span = DEFAULT_CLAMP_HIGH_V - DEFAULT_CLAMP_LOW_V
    if span_v > max_span:
        raise ValueError(f"2*V_pi={span_v:.3f} V exceeds safe sweep range [{DEFAULT_CLAMP_LOW_V}, {DEFAULT_CLAMP_HIGH_V}] V")
    v_start = min(max(v_lock_v - vpi_est_v, DEFAULT_CLAMP_LOW_V), DEFAULT_CLAMP_HIGH_V - span_v)
    voltages = np.linspace(v_start, v_start + span_v, 17, dtype=float)
    return float(v_start), voltages


def _analytical_dft_init(voltages_v: np.ndarray, dc_mv: np.ndarray, v_lock_v: float, vpi_est_v: float) -> tuple[float, float, float, float]:
    """Initialize (A, B, V_pi, phi_thesis) analytically via DFT and multi-Vpi linear LSQ."""
    candidates: list[tuple[float, float, float, float, float]] = []

    if voltages_v.size == 17:
        # First 16 points form one periodic DFT window over [V_start, V_start + 2*Vpi_est).
        y16 = dc_mv[:16]
        k = np.arange(16, dtype=float)
        z1 = (2.0 / 16.0) * np.sum(y16 * np.exp(-1j * 2.0 * np.pi * k / 16.0))
        a_dft = float(np.mean(y16))
        b_dft = max(float(np.abs(z1)), 1e-3)
        alpha0 = np.pi * (voltages_v[0] - v_lock_v) / vpi_est_v
        phi_dft = float(wrap_rad(float(np.angle(z1)) - alpha0))
        pred_dft = a_dft + b_dft * np.cos(np.pi * (voltages_v - v_lock_v) / vpi_est_v + phi_dft)
        rss_dft = float(np.sum((dc_mv - pred_dft) ** 2))
        candidates.append((rss_dft, a_dft, b_dft, vpi_est_v, phi_dft))

    for scale in np.linspace(0.75, 1.25, 11):
        vpi_cand = vpi_est_v * float(scale)
        alpha = np.pi * (voltages_v - v_lock_v) / vpi_cand
        design = np.column_stack([np.ones_like(alpha), np.cos(alpha), np.sin(alpha)])
        coeffs, _, _, _ = np.linalg.lstsq(design, dc_mv, rcond=None)
        a_c, c_c, s_c = float(coeffs[0]), float(coeffs[1]), float(coeffs[2])
        b_c = max(float(np.hypot(c_c, s_c)), 1e-3)
        phi_c = float(wrap_rad(float(np.arctan2(-s_c, c_c))))
        pred = a_c + b_c * np.cos(alpha + phi_c)
        rss = float(np.sum((dc_mv - pred) ** 2))
        candidates.append((rss, a_c, b_c, vpi_cand, phi_c))

    candidates.sort(key=lambda item: item[0])
    _, a0, b0, vpi0, phi0 = candidates[0]
    return a0, b0, vpi0, phi0


def fit_dc_psi_waveform(
    voltages_v: Sequence[float] | np.ndarray,
    dc_points_mv: Sequence[float] | np.ndarray,
    *,
    v_lock_v: float,
    vpi_est_v: float,
    dc_pre_mv: float | None = None,
    dc_post_mv: float | None = None,
) -> DcPsiResult:
    """Solve 4-parameter bounded NLSQ for V_CH1(V) = A + B*cos(pi*(V - V_lock)/V_pi + phi)."""
    v_arr = np.asarray(voltages_v, dtype=float)
    y_arr = np.asarray(dc_points_mv, dtype=float)
    if v_arr.ndim != 1 or y_arr.ndim != 1 or v_arr.size != y_arr.size or v_arr.size < 5:
        raise ValueError("voltages_v and dc_points_mv must be matching 1D arrays with >= 5 points")
    if np.any(~np.isfinite(v_arr)) or np.any(~np.isfinite(y_arr)):
        raise ValueError("Non-finite voltage or DC sample in DC PSI fit")
    if not np.isfinite(v_lock_v) or not np.isfinite(vpi_est_v) or vpi_est_v <= 0.0:
        raise ValueError(f"Invalid v_lock_v={v_lock_v} or vpi_est_v={vpi_est_v}")

    a0, b0, vpi0, phi0 = _analytical_dft_init(v_arr, y_arr, v_lock_v, vpi_est_v)

    def residuals(params: np.ndarray) -> np.ndarray:
        a_p, b_p, vpi_p, phi_p = params
        return a_p + b_p * np.cos(np.pi * (v_arr - v_lock_v) / vpi_p + phi_p) - y_arr

    lb = np.array([-np.inf, 1e-6, 0.25 * vpi_est_v, -np.inf], dtype=float)
    ub = np.array([np.inf, np.inf, 4.0 * vpi_est_v, np.inf], dtype=float)
    x0 = np.array([a0, max(b0, 1e-5), np.clip(vpi0, lb[2] * 1.01, ub[2] * 0.99), phi0], dtype=float)

    res = least_squares(
        residuals,
        x0,
        bounds=(lb, ub),
        method="trf",
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12,
        max_nfev=2000,
    )
    if not res.success or np.any(~np.isfinite(res.x)):
        raise RuntimeError(f"DC PSI NLSQ optimization failed: {res.message}")

    a_opt, b_opt, vpi_opt, phi_raw = (float(val) for val in res.x)
    if b_opt <= 0.0 or vpi_opt <= 0.0:
        raise RuntimeError(f"DC PSI solved invalid parameters: B={b_opt}, V_pi={vpi_opt}")

    phi_thesis = float(wrap_rad(phi_raw))
    phi_fw = float(thesis_to_fw_rad(phi_thesis))
    res_mv = np.asarray(res.fun, dtype=float)
    rmse_mv = float(np.sqrt(np.mean(res_mv ** 2)))

    dof = max(v_arr.size - 4, 1)
    rss = float(np.sum(res_mv ** 2))
    s2 = rss / dof
    _, svals, vt = np.linalg.svd(res.jac, full_matrices=False)
    valid = svals > 1e-12 * max(float(svals[0]), 1.0)
    if not bool(valid[-1]) and np.sum(valid) < 4:
        raise RuntimeError("Singular Jacobian in DC PSI covariance estimate")
    inv_s2 = np.where(valid, 1.0 / np.maximum(svals ** 2, 1e-30), 0.0)
    cov = s2 * ((vt.T * inv_s2) @ vt)
    sigma_phi_rad = float(np.sqrt(max(float(cov[3, 3]), 0.0)))
    sigma_phi_deg = float(np.degrees(sigma_phi_rad))

    pre_val = float(dc_pre_mv) if dc_pre_mv is not None else float(a_opt + b_opt * np.cos(phi_thesis))
    post_val = float(dc_post_mv) if dc_post_mv is not None else pre_val
    drift_mv = float(post_val - pre_val)

    return DcPsiResult(
        phi_true_thesis_rad=phi_thesis,
        phi_true_thesis_deg=float(np.degrees(phi_thesis)),
        phi_true_fw_rad=phi_fw,
        phi_true_fw_deg=float(np.degrees(phi_fw)),
        vpi_v=vpi_opt,
        dc_a_mv=a_opt,
        dc_b_mv=b_opt,
        fit_rmse_mv=rmse_mv,
        sigma_phi_deg=sigma_phi_deg,
        drift_mv=drift_mv,
        dc_pre_mv=pre_val,
        dc_post_mv=post_val,
        v_lock_v=float(v_lock_v),
        v_start_v=float(v_arr[0]),
        voltages_v=tuple(float(x) for x in v_arr),
        dc_points_mv=tuple(float(y) for y in y_arr),
        residuals_mv=tuple(float(r) for r in res_mv),
    )


def measure_dc_psi(
    uart: UARTClient,
    v_lock_mv: float,
    vpi_est_mv: float,
    *,
    pre_wait_blocks: int = 0,
    initial_jump_wait_blocks: int = DEFAULT_INITIAL_JUMP_WAIT_BLOCKS,
    step_wait_blocks: int = DEFAULT_STEP_WAIT_BLOCKS,
    return_wait_blocks: int = DEFAULT_RETURN_WAIT_BLOCKS,
    blocks_per_point: int = DEFAULT_BLOCKS_PER_POINT,
) -> DcPsiResult:
    """Execute the 17-point In-Situ DC PSI protocol on hardware via `meas` (`pilot=0`)."""
    v_lock_v = v_lock_mv * 1e-3
    vpi_est_v = vpi_est_mv * 1e-3
    _, voltages_v = compute_scan_window(v_lock_v, vpi_est_v)

    pre_rows = uart.meas(v_lock_mv, 0.0, pre_wait_blocks, blocks_per_point)
    dc_pre_mv = float(np.mean([r.dc_mv for r in pre_rows]))

    dc_points_mv: list[float] = []
    for idx, v_pt_v in enumerate(voltages_v):
        wait_b = initial_jump_wait_blocks if idx == 0 else step_wait_blocks
        rows = uart.meas(float(v_pt_v * 1e3), 0.0, wait_b, blocks_per_point)
        dc_points_mv.append(float(np.mean([r.dc_mv for r in rows])))

    post_rows = uart.meas(v_lock_mv, 0.0, return_wait_blocks, blocks_per_point)
    dc_post_mv = float(np.mean([r.dc_mv for r in post_rows]))

    return fit_dc_psi_waveform(
        voltages_v,
        dc_points_mv,
        v_lock_v=v_lock_v,
        vpi_est_v=vpi_est_v,
        dc_pre_mv=dc_pre_mv,
        dc_post_mv=dc_post_mv,
    )
