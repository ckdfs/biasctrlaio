"""Synchronous meas-driven bias controllers (B1-B4, Affine Frozen/Recal, S3+)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from .common import DEFAULT_PILOT_MV, DEFAULT_WAIT_BLOCKS, thesis_to_fw_rad, wrap_rad
from .metrics import cond_kappa_and_sigma_min
from .uart_client import MeasRow, UARTClient


def _wrap_half_period(angle_rad: float) -> float:
    """Wrap an axis angle difference into (-pi/2, pi/2] (modulo pi)."""
    w = (angle_rad + 0.5 * np.pi) % np.pi - 0.5 * np.pi
    if np.isclose(w, -0.5 * np.pi, atol=1e-15):
        return 0.5 * np.pi
    return float(w)


def _principal_axis_angle(samples: np.ndarray) -> float:
    """Closed-form 2x2 covariance principal eigenvector angle (modulo pi)."""
    pts = np.column_stack([samples.real, samples.imag])
    centered = pts - np.mean(pts, axis=0, keepdims=True)
    cov = (centered.T @ centered) / max(len(pts), 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    vec = eigvecs[:, int(np.argmax(eigvals))]
    # Deterministic hemisphere convention: prefer positive x (or positive y if x==0).
    if vec[0] < -1e-12 or (abs(vec[0]) <= 1e-12 and vec[1] < 0.0):
        vec = -vec
    return float(np.arctan2(vec[1], vec[0]))


def fit_fitzgibbon_ellipse(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Direct least-squares ellipse fit (Halir-Flusser 1998) returning (center, M_spd).

    Fits (z - center)^T M (z - center) = 1 with M symmetric positive definite.
    """
    if x.size != y.size or x.size < 6:
        raise ValueError(f"Need >= 6 points for ellipse fit, got {x.size}")

    mx, sx = float(np.mean(x)), float(np.std(x))
    my, sy = float(np.mean(y)), float(np.std(y))
    if sx <= 1e-15 or sy <= 1e-15:
        raise ValueError("Degenerate ellipse input: zero variance on an axis")

    xn = (x - mx) / sx
    yn = (y - my) / sy
    d1 = np.column_stack([xn * xn, xn * yn, yn * yn])
    d2 = np.column_stack([xn, yn, np.ones_like(xn)])
    s1 = d1.T @ d1
    s2 = d1.T @ d2
    s3 = d2.T @ d2
    t_mat = -np.linalg.solve(s3, s2.T)
    m_mat = s1 + s2 @ t_mat

    c1_inv = np.array([[0.0, 0.0, 0.5], [0.0, -1.0, 0.0], [0.5, 0.0, 0.0]], dtype=float)
    eigvals, eigvecs = np.linalg.eig(c1_inv @ m_mat)
    real_mask = np.abs(eigvals.imag) < 1e-9
    a1_cand = None
    for idx in range(eigvecs.shape[1]):
        if not real_mask[idx]:
            continue
        v = eigvecs[:, idx].real
        cond = 4.0 * v[0] * v[2] - v[1] ** 2
        if cond > 0.0:
            a1_cand = v
            break
    if a1_cand is None:
        raise ValueError("Fitzgibbon ellipse fit failed to find positive-conic eigenvector")

    a2_cand = t_mat @ a1_cand
    an, bn, cn = float(a1_cand[0]), float(a1_cand[1]), float(a1_cand[2])
    dn, en, fn = float(a2_cand[0]), float(a2_cand[1]), float(a2_cand[2])

    # Transform conic coefficients from normalized (xn, yn) back to (x, y).
    inv_sx = 1.0 / sx
    inv_sy = 1.0 / sy
    a = an * inv_sx * inv_sx
    b = bn * inv_sx * inv_sy
    c = cn * inv_sy * inv_sy
    d = dn * inv_sx - 2.0 * an * mx * inv_sx * inv_sx - bn * my * inv_sx * inv_sy
    e = en * inv_sy - 2.0 * cn * my * inv_sy * inv_sy - bn * mx * inv_sx * inv_sy
    f = (
        fn
        + an * (mx * inv_sx) ** 2
        + bn * (mx * inv_sx) * (my * inv_sy)
        + cn * (my * inv_sy) ** 2
        - dn * mx * inv_sx
        - en * my * inv_sy
    )

    q_mat = np.array([[a, 0.5 * b], [0.5 * b, c]], dtype=float)
    p_vec = np.array([d, e], dtype=float)
    center = -0.5 * np.linalg.solve(q_mat, p_vec)
    rhs = float(center.T @ q_mat @ center - f)
    if rhs == 0.0 or not np.isfinite(rhs):
        raise ValueError("Degenerate ellipse conic rhs=0")
    m_spd = q_mat / rhs
    eigvals_m, _ = np.linalg.eigh(m_spd)
    if np.any(eigvals_m <= 0.0):
        raise ValueError(f"Fitted conic is not positive definite: eigvals={eigvals_m}")
    return center, m_spd


@dataclass(frozen=True)
class AffineCalModel:
    """Complete Chapter 5 affine calibration bundle in thesis coordinates."""

    B_cal: np.ndarray            # 2x2 unwarping matrix: u = B_cal @ (z - b_cal)
    b_cal: np.ndarray            # (2,) center offset in [s2, s1] volts
    A_hat: np.ndarray            # 2x2 forward matrix inv(B_cal)
    h1_ref_phase: float          # Principal axis angle for H1 (rad)
    h2_ref_phase: float          # Principal axis angle for H2 (rad)
    h1_mean_complex: complex     # Mean complex H1 offset (V)
    h2_mean_complex: complex     # Mean complex H2 offset (V)
    a1_cal: float                # Full-cycle H1 peak amplitude (V)
    a2_cal: float                # Full-cycle H2 peak amplitude (V)
    a1_signed: float             # A_hat[1, 1] signed sin(phi_thesis) coefficient (V)
    dc_a_v: float                # CH1 DC mean (V)
    dc_b_v: float                # CH1 DC modulation amplitude (V)
    vpi_v: float                 # Half-wave voltage (V)
    null_v_thesis: float         # MATP (phi_thesis = 0) nearest sweep midpoint (V)
    null_v_fw: float             # MITP (phi_fw = 0) nearest sweep midpoint (V)
    pilot_v: float               # Electrode/command pilot amplitude used during cal (V)
    kappa: float
    sigma_min: float
    winding_sign: int

    def project(self, h1_complex_v: complex, h2_complex_v: complex, theta1: float | None = None, theta2: float | None = None) -> np.ndarray:
        """Project complex phasors onto reference axes, returning z = [s2, s1]."""
        t1 = self.h1_ref_phase if theta1 is None else theta1
        t2 = self.h2_ref_phase if theta2 is None else theta2
        s1 = float(h1_complex_v.real * np.cos(t1) + h1_complex_v.imag * np.sin(t1))
        s2 = float(h2_complex_v.real * np.cos(t2) + h2_complex_v.imag * np.sin(t2))
        return np.array([s2, s1], dtype=float)

    def target_voltage_v(self, phi_target_thesis_rad: float, *, v_min_v: float = -9.8, v_max_v: float = 9.8) -> float:
        """Map thesis target phase to nearest valid bias voltage inside [v_min_v, v_max_v]."""
        phi_w = float(wrap_rad(phi_target_thesis_rad))
        base = self.null_v_thesis + (phi_w / np.pi) * self.vpi_v
        period = 2.0 * self.vpi_v
        mid = 0.5 * (v_min_v + v_max_v)
        cand = mid + float(np.remainder(base - mid + self.vpi_v, period) - self.vpi_v)
        if v_min_v <= cand <= v_max_v:
            return cand
        for shift in (-period, period):
            alt = cand + shift
            if v_min_v <= alt <= v_max_v:
                return alt
        return float(np.clip(cand, v_min_v, v_max_v))


def calibrate_affine_from_sweep(
    voltages_v: Sequence[float] | np.ndarray,
    h1_complex_v: Sequence[complex] | np.ndarray,
    h2_complex_v: Sequence[complex] | np.ndarray,
    dc_v: Sequence[float] | np.ndarray,
    *,
    pilot_v: float = DEFAULT_PILOT_MV * 1e-3,
) -> AffineCalModel:
    """Calibrate Chapter 5 affine model from ordered (or bidirectional) sweep data."""
    v_arr = np.asarray(voltages_v, dtype=float)
    h1_arr = np.asarray(h1_complex_v, dtype=complex)
    h2_arr = np.asarray(h2_complex_v, dtype=complex)
    dc_arr = np.asarray(dc_v, dtype=float)
    if not (v_arr.size == h1_arr.size == h2_arr.size == dc_arr.size) or v_arr.size < 12:
        raise ValueError("Calibration sweep arrays must have matching lengths >= 12")
    if np.any(~np.isfinite(v_arr)) or np.any(~np.isfinite(dc_arr)):
        raise ValueError("Calibration sweep contains non-finite values")

    # Sort ascending by bias voltage for consistent winding direction computation.
    order = np.argsort(v_arr, kind="mergesort")
    v_ord = v_arr[order]
    h1_ord = h1_arr[order]
    h2_ord = h2_arr[order]
    dc_ord = dc_arr[order]

    # Pass 1: principal-axis reference phase extraction.
    theta1 = _principal_axis_angle(h1_ord)
    theta2 = _principal_axis_angle(h2_ord)
    h1_mean = complex(np.mean(h1_ord))
    h2_mean = complex(np.mean(h2_ord))

    s1 = h1_ord.real * np.cos(theta1) + h1_ord.imag * np.sin(theta1)
    s2 = h2_ord.real * np.cos(theta2) + h2_ord.imag * np.sin(theta2)

    # Pass 2: Fitzgibbon ellipse fit on X=s2, Y=s1.
    b_hat, m_spd = fit_fitzgibbon_ellipse(s2, s1)
    eigvals, eigvecs = np.linalg.eigh(m_spd)
    m_sqrt = (eigvecs * np.sqrt(eigvals)) @ eigvecs.T

    z_centered = np.column_stack([s2 - b_hat[0], s1 - b_hat[1]])
    w_pts = z_centered @ m_sqrt.T

    # Group samples by unique bias voltage so merged bidirectional sweeps (fwd + rev)
    # yield a single ascending orbit rather than zig-zagging across hysteresis steps.
    uniq_v, inv_idx = np.unique(np.round(v_ord, 6), return_inverse=True)
    if uniq_v.size < 6:
        raise ValueError(f"Insufficient distinct bias voltages ({uniq_v.size}) for affine orientation")
    counts = np.bincount(inv_idx).astype(float)
    w_uniq = np.column_stack(
        [
            np.bincount(inv_idx, weights=w_pts[:, 0]) / counts,
            np.bincount(inv_idx, weights=w_pts[:, 1]) / counts,
        ]
    )

    # Centered winding along increasing bias voltage eliminates reflection ambiguity.
    cross = w_uniq[:-1, 0] * w_uniq[1:, 1] - w_uniq[1:, 0] * w_uniq[:-1, 1]
    area = float(np.sum(cross))
    if abs(area) <= 1e-12:
        raise ValueError("Degenerate ellipse winding area along sweep voltage")
    winding_sign = 1 if area > 0.0 else -1

    # Synchronous CH1 DC regression locks canonical origin phi_c (MATP=0 at max CH1),
    # and unique-voltage phase monotonicity (d(phi)/dV > 0) locks chirality unambiguously.
    dc_span = float(np.max(dc_ord) - np.min(dc_ord))
    if dc_span <= 1e-4:
        raise ValueError(f"CH1 DC span too small ({dc_span:.6f} V) for synchronous origin anchor")

    def _orient(w_sign: int) -> tuple[np.ndarray, float, float, float, float]:
        f_m = np.eye(2, dtype=float) if w_sign > 0 else np.diag([1.0, -1.0])
        w_or = w_pts @ f_m.T
        chi_pts = np.arctan2(w_or[:, 1], w_or[:, 0])
        des = np.column_stack([np.ones_like(chi_pts), np.cos(chi_pts), np.sin(chi_pts)])
        c0_v, cc_v, cs_v = (float(x) for x in np.linalg.lstsq(des, dc_ord, rcond=None)[0])
        dc_b_v = float(np.hypot(cc_v, cs_v))
        if dc_b_v <= 1e-6:
            raise ValueError("Degenerate synchronous CH1 DC cosine amplitude")
        phi_c_v = float(np.arctan2(cs_v, cc_v))
        c_r, s_r = np.cos(-phi_c_v), np.sin(-phi_c_v)
        rot_v = np.array([[c_r, -s_r], [s_r, c_r]], dtype=float)
        b_mat = rot_v @ f_m @ m_sqrt
        u_u = w_uniq @ (rot_v @ f_m).T
        phi_u = np.unwrap(np.arctan2(u_u[:, 1], u_u[:, 0]))
        slp, icp = np.polyfit(uniq_v, phi_u, 1)
        return b_mat, c0_v, dc_b_v, float(slp), float(icp)

    B_cal, c0, dc_b, slope_rad_per_v, intercept_rad = _orient(winding_sign)
    if slope_rad_per_v < 0.0:
        winding_sign = -winding_sign
        B_cal, c0, dc_b, slope_rad_per_v, intercept_rad = _orient(winding_sign)
    if not np.isfinite(slope_rad_per_v) or slope_rad_per_v <= 1e-6:
        raise ValueError(f"Invalid phase-voltage slope in affine calibration: {slope_rad_per_v}")

    A_hat = np.linalg.inv(B_cal)
    vpi_v = float(np.pi / slope_rad_per_v)

    v_mid = 0.5 * (float(uniq_v[0]) + float(uniq_v[-1]))
    period = 2.0 * vpi_v
    raw_null_thesis = -intercept_rad / slope_rad_per_v
    null_v_thesis = v_mid + float(np.remainder(raw_null_thesis - v_mid + vpi_v, period) - vpi_v)
    raw_null_fw = (np.pi - intercept_rad) / slope_rad_per_v
    null_v_fw = v_mid + float(np.remainder(raw_null_fw - v_mid + vpi_v, period) - vpi_v)

    kappa, sigma_min = cond_kappa_and_sigma_min(A_hat)
    a2_cal = float(np.hypot(A_hat[0, 0], A_hat[0, 1]))
    a1_cal = float(np.hypot(A_hat[1, 0], A_hat[1, 1]))
    a1_signed = float(A_hat[1, 1])
    if abs(a1_signed) <= 1e-9:
        raise ValueError("Degenerate H1 diagonal sensitivity A_hat[1, 1]")

    return AffineCalModel(
        B_cal=B_cal,
        b_cal=b_hat,
        A_hat=A_hat,
        h1_ref_phase=theta1,
        h2_ref_phase=theta2,
        h1_mean_complex=h1_mean,
        h2_mean_complex=h2_mean,
        a1_cal=a1_cal,
        a2_cal=a2_cal,
        a1_signed=a1_signed,
        dc_a_v=c0,
        dc_b_v=dc_b,
        vpi_v=vpi_v,
        null_v_thesis=null_v_thesis,
        null_v_fw=null_v_fw,
        pilot_v=float(pilot_v),
        kappa=kappa,
        sigma_min=sigma_min,
        winding_sign=winding_sign,
    )


@dataclass(frozen=True)
class StepObservation:
    """Single closed-loop update record returned by all synchronous controllers."""

    step_idx: int
    bias_mv: float
    next_bias_mv: float
    phi_hat_thesis_rad: float
    phi_hat_fw_rad: float
    err_est_rad: float
    radius: float
    h1_proj_v: float
    h2_proj_v: float
    h1_raw_v: complex
    h2_raw_v: complex
    dc_mv: float
    frozen: bool
    probe_rejected: bool
    recal_triggered: bool
    k_axis: float
    theta1_rad: float
    theta2_rad: float
    rho_ema: float
    ch0_min_mv: float
    ch0_max_mv: float
    ch1_min_mv: float
    ch1_max_mv: float
    drops: int
    crc: int


class BaseBiasController:
    """Common base for synchronous meas-driven single-MZM bias controllers."""

    name: str = "Base"

    def __init__(
        self,
        cal: AffineCalModel,
        target_thesis_rad: float,
        *,
        gain: float = 0.25,
        pilot_mv: float = DEFAULT_PILOT_MV,
        wait_blocks: int = DEFAULT_WAIT_BLOCKS,
        avg_blocks: int = 1,
        v_min_v: float = -9.8,
        v_max_v: float = 9.8,
    ) -> None:
        if not np.isfinite(gain) or gain <= 0.0:
            raise ValueError(f"gain must be positive, got {gain}")
        if not (0.0 <= pilot_mv <= 250.0):
            raise ValueError(f"pilot_mv out of range: {pilot_mv}")
        self.cal = cal
        self.target_thesis_rad = float(wrap_rad(target_thesis_rad))
        self.gain = float(gain)
        self.pilot_mv = float(pilot_mv)
        self.wait_blocks = int(wait_blocks)
        self.avg_blocks = int(avg_blocks)
        self.v_min_v = float(v_min_v)
        self.v_max_v = float(v_max_v)
        self.step_count = 0

    @property
    def target_fw_rad(self) -> float:
        return float(thesis_to_fw_rad(self.target_thesis_rad))

    def _clamp_v(self, v: float) -> float:
        return float(np.clip(v, self.v_min_v, self.v_max_v))

    def _measure_avg(self, uart: UARTClient, bias_v: float, *, pilot_mv: float | None = None, wait_blocks: int | None = None) -> tuple[complex, complex, float, MeasRow]:
        p_mv = self.pilot_mv if pilot_mv is None else pilot_mv
        w_blk = self.wait_blocks if wait_blocks is None else wait_blocks
        rows = uart.meas(bias_v * 1e3, p_mv, w_blk, self.avg_blocks)
        h1 = complex(np.mean([r.h1_complex_v for r in rows]))
        h2 = complex(np.mean([r.h2_complex_v for r in rows]))
        dc_mv = float(np.mean([r.dc_mv for r in rows]))
        return h1, h2, dc_mv, rows[-1]

    def step(self, uart: UARTClient, bias_v: float) -> StepObservation:
        raise NotImplementedError


class B1SingleHarmonic(BaseBiasController):
    """B1 (Fu 2013): single-harmonic H1 matching without optical/RF normalization."""

    name = "B1"

    def step(self, uart: UARTClient, bias_v: float) -> StepObservation:
        h1, h2, dc_mv, tail = self._measure_avg(uart, bias_v)
        z = self.cal.project(h1, h2)
        s2, s1 = float(z[0]), float(z[1])
        y_meas = s1 / self.cal.a1_signed
        y_target = float(np.sin(self.target_thesis_rad))
        slope_sgn = 1.0 if np.cos(self.target_thesis_rad) >= 0.0 else -1.0
        err_est = float(np.clip(slope_sgn * (y_meas - y_target), -0.5 * np.pi, 0.5 * np.pi))

        phi_est = float(np.arcsin(np.clip(y_meas, -1.0, 1.0)))
        if np.cos(self.target_thesis_rad) < 0.0:
            phi_est = float(wrap_rad(np.pi - phi_est))
        next_v = self._clamp_v(bias_v - self.gain * (self.cal.vpi_v / np.pi) * err_est)
        self.step_count += 1
        return StepObservation(
            step_idx=self.step_count,
            bias_mv=bias_v * 1e3,
            next_bias_mv=next_v * 1e3,
            phi_hat_thesis_rad=phi_est,
            phi_hat_fw_rad=float(thesis_to_fw_rad(phi_est)),
            err_est_rad=err_est,
            radius=abs(y_meas),
            h1_proj_v=s1,
            h2_proj_v=s2,
            h1_raw_v=h1,
            h2_raw_v=h2,
            dc_mv=dc_mv,
            frozen=False,
            probe_rejected=False,
            recal_triggered=False,
            k_axis=1.0,
            theta1_rad=self.cal.h1_ref_phase,
            theta2_rad=self.cal.h2_ref_phase,
            rho_ema=0.0,
            ch0_min_mv=tail.ch0_min_mv,
            ch0_max_mv=tail.ch0_max_mv,
            ch1_min_mv=tail.ch1_min_mv,
            ch1_max_mv=tail.ch1_max_mv,
            drops=tail.drops,
            crc=tail.crc,
        )


class B2RatioH1DC(BaseBiasController):
    """B2 (Li 2013): H1 / I_DC ratio control (sensitive to RF J0 in DC denominator)."""

    name = "B2"

    def step(self, uart: UARTClient, bias_v: float) -> StepObservation:
        if self.cal.dc_a_v <= 1e-6:
            raise ValueError("B2RatioH1DC requires positive CH1 DC baseline")
        h1, h2, dc_mv, tail = self._measure_avg(uart, bias_v)
        z = self.cal.project(h1, h2)
        s2, s1 = float(z[0]), float(z[1])
        y_meas = s1 / self.cal.a1_signed
        dc_norm = max((dc_mv * 1e-3) / self.cal.dc_a_v, 0.05)
        c_dc = float(np.clip(self.cal.dc_b_v / self.cal.dc_a_v, 0.0, 0.999))
        r_meas = y_meas / dc_norm
        r_star = float(np.sin(self.target_thesis_rad) / max(1.0 + c_dc * np.cos(self.target_thesis_rad), 0.05))
        slope_sgn = 1.0 if (np.cos(self.target_thesis_rad) + c_dc) >= 0.0 else -1.0
        err_est = float(np.clip(slope_sgn * (r_meas - r_star), -0.5 * np.pi, 0.5 * np.pi))
        next_v = self._clamp_v(bias_v - self.gain * (self.cal.vpi_v / np.pi) * err_est)
        self.step_count += 1
        return StepObservation(
            step_idx=self.step_count,
            bias_mv=bias_v * 1e3,
            next_bias_mv=next_v * 1e3,
            phi_hat_thesis_rad=self.target_thesis_rad + err_est,
            phi_hat_fw_rad=float(thesis_to_fw_rad(self.target_thesis_rad + err_est)),
            err_est_rad=err_est,
            radius=abs(r_meas),
            h1_proj_v=s1,
            h2_proj_v=s2,
            h1_raw_v=h1,
            h2_raw_v=h2,
            dc_mv=dc_mv,
            frozen=False,
            probe_rejected=False,
            recal_triggered=False,
            k_axis=1.0,
            theta1_rad=self.cal.h1_ref_phase,
            theta2_rad=self.cal.h2_ref_phase,
            rho_ema=0.0,
            ch0_min_mv=tail.ch0_min_mv,
            ch0_max_mv=tail.ch0_max_mv,
            ch1_min_mv=tail.ch1_min_mv,
            ch1_max_mv=tail.ch1_max_mv,
            drops=tail.drops,
            crc=tail.crc,
        )


class B3DualHarmonicRatio(BaseBiasController):
    """B3 (Wang 2010 / Wen 2025): unsigned |H1|/|H2| ratio with QTP extremum search."""

    name = "B3"

    def step(self, uart: UARTClient, bias_v: float) -> StepObservation:
        h1, h2, dc_mv, tail = self._measure_avg(uart, bias_v)
        z = self.cal.project(h1, h2)
        s2, s1 = float(z[0]), float(z[1])
        amp1 = abs(h1)
        amp2 = abs(h2)
        rho = self.cal.a1_cal / max(self.cal.a2_cal, 1e-9)

        if abs(np.cos(self.target_thesis_rad)) < 0.05:
            dv_probe = 0.010 * (self.cal.vpi_v / np.pi)
            _, h2_p, _, _ = self._measure_avg(uart, self._clamp_v(bias_v + dv_probe))
            _, h2_m, _, _ = self._measure_avg(uart, self._clamp_v(bias_v - dv_probe))
            diff = abs(h2_p) - abs(h2_m)
            sgn = 1.0 if diff > 0.0 else (-1.0 if diff < 0.0 else 0.0)
            step_ph = self.gain * sgn * 0.015
        else:
            r_target = abs(float(np.tan(self.target_thesis_rad))) * rho
            r_meas = amp1 / max(amp2, 1e-6)
            g_loc = rho / max(float(np.cos(self.target_thesis_rad)) ** 2, 1e-4)
            sgn_s = 1.0 if np.sin(2.0 * self.target_thesis_rad) >= 0.0 else -1.0
            step_ph = float(np.clip(self.gain * sgn_s * (r_meas - r_target) / max(g_loc, 1.0), -0.25, 0.25))

        next_v = self._clamp_v(bias_v - (self.cal.vpi_v / np.pi) * step_ph)
        phi_mag = float(np.arctan2(amp1, max(amp2 * rho, 1e-12)))
        self.step_count += 1
        return StepObservation(
            step_idx=self.step_count,
            bias_mv=bias_v * 1e3,
            next_bias_mv=next_v * 1e3,
            phi_hat_thesis_rad=phi_mag,
            phi_hat_fw_rad=float(thesis_to_fw_rad(phi_mag)),
            err_est_rad=step_ph / self.gain,
            radius=float(np.hypot(amp1 / max(self.cal.a1_cal, 1e-9), amp2 / max(self.cal.a2_cal, 1e-9))),
            h1_proj_v=s1,
            h2_proj_v=s2,
            h1_raw_v=h1,
            h2_raw_v=h2,
            dc_mv=dc_mv,
            frozen=False,
            probe_rejected=False,
            recal_triggered=False,
            k_axis=1.0,
            theta1_rad=self.cal.h1_ref_phase,
            theta2_rad=self.cal.h2_ref_phase,
            rho_ema=0.0,
            ch0_min_mv=tail.ch0_min_mv,
            ch0_max_mv=tail.ch0_max_mv,
            ch1_min_mv=tail.ch1_min_mv,
            ch1_max_mv=tail.ch1_max_mv,
            drops=tail.drops,
            crc=tail.crc,
        )


class B4DiagonalAtan2(BaseBiasController):
    """B4 (Dandridge 1982): diagonal amplitude-normalized atan2 without cross/offset."""

    name = "B4"

    def __init__(
        self,
        cal: AffineCalModel,
        target_thesis_rad: float,
        *,
        subtract_offset: bool = False,
        **kwargs,
    ) -> None:
        super().__init__(cal, target_thesis_rad, **kwargs)
        self.subtract_offset = subtract_offset
        if abs(cal.A_hat[0, 0]) <= 1e-12 or abs(cal.A_hat[1, 1]) <= 1e-12:
            raise ValueError("Singular diagonal elements in A_hat for B4DiagonalAtan2")
        self.B_diag = np.diag([1.0 / cal.A_hat[0, 0], 1.0 / cal.A_hat[1, 1]])

    def step(self, uart: UARTClient, bias_v: float) -> StepObservation:
        h1, h2, dc_mv, tail = self._measure_avg(uart, bias_v)
        z = self.cal.project(h1, h2)
        z_in = z - self.cal.b_cal if self.subtract_offset else z
        u = self.B_diag @ z_in
        r = float(np.hypot(u[0], u[1]))
        phi_hat = float(wrap_rad(float(np.arctan2(u[1], u[0]))))
        err = float(wrap_rad(phi_hat - self.target_thesis_rad))
        next_v = self._clamp_v(bias_v - self.gain * (self.cal.vpi_v / np.pi) * err)
        self.step_count += 1
        return StepObservation(
            step_idx=self.step_count,
            bias_mv=bias_v * 1e3,
            next_bias_mv=next_v * 1e3,
            phi_hat_thesis_rad=phi_hat,
            phi_hat_fw_rad=float(thesis_to_fw_rad(phi_hat)),
            err_est_rad=err,
            radius=r,
            h1_proj_v=float(z[1]),
            h2_proj_v=float(z[0]),
            h1_raw_v=h1,
            h2_raw_v=h2,
            dc_mv=dc_mv,
            frozen=False,
            probe_rejected=False,
            recal_triggered=False,
            k_axis=1.0,
            theta1_rad=self.cal.h1_ref_phase,
            theta2_rad=self.cal.h2_ref_phase,
            rho_ema=0.0,
            ch0_min_mv=tail.ch0_min_mv,
            ch0_max_mv=tail.ch0_max_mv,
            ch1_min_mv=tail.ch1_min_mv,
            ch1_max_mv=tail.ch1_max_mv,
            drops=tail.drops,
            crc=tail.crc,
        )


class AffineFrozen(BaseBiasController):
    """Chapter 5 full affine controller with frozen initial (B_cal, b_cal) and r<0.15 freeze."""

    name = "AffineFrozen"

    def __init__(
        self,
        cal: AffineCalModel,
        target_thesis_rad: float,
        *,
        r_freeze_thresh: float = 0.15,
        rho_alpha: float = 0.15,
        rho_thresh: float = 0.25,
        hold_ticks: int = 10,
        **kwargs,
    ) -> None:
        super().__init__(cal, target_thesis_rad, **kwargs)
        self.r_freeze_thresh = float(r_freeze_thresh)
        self.rho_alpha = float(rho_alpha)
        self.rho_thresh = float(rho_thresh)
        self.hold_ticks = int(hold_ticks)
        self.rho_ema = 0.0
        self.rho_streak = 0
        self.last_phi_hat = self.target_thesis_rad

    def step(self, uart: UARTClient, bias_v: float) -> StepObservation:
        h1, h2, dc_mv, tail = self._measure_avg(uart, bias_v)
        z = self.cal.project(h1, h2)
        u = self.cal.B_cal @ (z - self.cal.b_cal)
        r = float(np.hypot(u[0], u[1]))
        frozen = r < self.r_freeze_thresh
        recal_triggered = False

        if frozen:
            phi_hat = self.last_phi_hat
            err = 0.0
            next_v = bias_v
            self.rho_streak = 0
        else:
            phi_hat = float(wrap_rad(float(np.arctan2(u[1], u[0]))))
            self.last_phi_hat = phi_hat
            err = float(wrap_rad(phi_hat - self.target_thesis_rad))
            next_v = self._clamp_v(bias_v - self.gain * (self.cal.vpi_v / np.pi) * err)
            self.rho_ema = (1.0 - self.rho_alpha) * self.rho_ema + self.rho_alpha * abs(r - 1.0)
            if self.rho_ema > self.rho_thresh:
                self.rho_streak += 1
                recal_triggered = self.rho_streak >= self.hold_ticks
            else:
                self.rho_streak = 0

        self.step_count += 1
        return StepObservation(
            step_idx=self.step_count,
            bias_mv=bias_v * 1e3,
            next_bias_mv=next_v * 1e3,
            phi_hat_thesis_rad=phi_hat,
            phi_hat_fw_rad=float(thesis_to_fw_rad(phi_hat)),
            err_est_rad=err,
            radius=r,
            h1_proj_v=float(z[1]),
            h2_proj_v=float(z[0]),
            h1_raw_v=h1,
            h2_raw_v=h2,
            dc_mv=dc_mv,
            frozen=frozen,
            probe_rejected=False,
            recal_triggered=recal_triggered,
            k_axis=1.0,
            theta1_rad=self.cal.h1_ref_phase,
            theta2_rad=self.cal.h2_ref_phase,
            rho_ema=self.rho_ema,
            ch0_min_mv=tail.ch0_min_mv,
            ch0_max_mv=tail.ch0_max_mv,
            ch1_min_mv=tail.ch1_min_mv,
            ch1_max_mv=tail.ch1_max_mv,
            drops=tail.drops,
            crc=tail.crc,
        )


class AffineRecal(AffineFrozen):
    """Per-state recalibrated affine controller."""

    name = "AffineRecal"

    def recalibrate(self, new_cal: AffineCalModel) -> None:
        self.cal = new_cal
        self.rho_ema = 0.0
        self.rho_streak = 0


class AffineS3Plus(BaseBiasController):
    """S3+: online strong-harmonic phase tracking + +/-20 mrad square-wave axis-ratio probe."""

    name = "AffineS3Plus"

    def __init__(
        self,
        cal: AffineCalModel,
        target_thesis_rad: float,
        *,
        r_freeze_thresh: float = 0.15,
        r_track_thresh: float = 0.30,
        axis_reject_thresh: float = 0.22,
        probe_dither_rad: float = 0.020,
        probe_interval: int = 1,
        alpha_theta: float = 0.20,
        alpha_k_axis: float = 0.25,
        rho_alpha: float = 0.15,
        **kwargs,
    ) -> None:
        super().__init__(cal, target_thesis_rad, **kwargs)
        self.r_freeze_thresh = float(r_freeze_thresh)
        self.r_track_thresh = float(r_track_thresh)
        self.axis_reject_thresh = float(axis_reject_thresh)
        self.probe_dither_rad = float(probe_dither_rad)
        self.probe_interval = max(1, int(probe_interval))
        self.alpha_theta = float(alpha_theta)
        self.alpha_k_axis = float(alpha_k_axis)
        self.rho_alpha = float(rho_alpha)

        self.theta1 = float(cal.h1_ref_phase)
        self.theta2 = float(cal.h2_ref_phase)
        self.k_axis = 1.0  # (1 + delta) ratio of H2-axis to H1-axis
        self.rho_ema = 0.0
        self.last_phi_hat = self.target_thesis_rad
        self.polarity_sign = 1

    def _update_reference_phases(self, h1: complex, h2: complex, r_raw: float) -> None:
        if r_raw <= self.r_track_thresh:
            return
        h1c = h1 - self.cal.h1_mean_complex
        h2c = h2 - self.cal.h2_mean_complex
        a1 = abs(h1c)
        a2 = abs(h2c)
        if a1 > 0.30 * self.cal.a1_cal * r_raw and a1 > 1e-9:
            dtheta1 = _wrap_half_period(float(np.angle(h1c)) - self.theta1)
            self.theta1 += self.alpha_theta * dtheta1
            self.theta2 += 2.0 * self.alpha_theta * dtheta1
        elif a2 > 0.30 * self.cal.a2_cal * r_raw and a2 > 1e-9:
            dtheta2 = _wrap_half_period(float(np.angle(h2c)) - self.theta2)
            self.theta2 += self.alpha_theta * dtheta2
            self.theta1 += 0.5 * self.alpha_theta * dtheta2

    def step(self, uart: UARTClient, bias_v: float) -> StepObservation:
        h1, h2, dc_mv, tail = self._measure_avg(uart, bias_v)

        # Preliminary pullback to check radius before updating online reference phases.
        z0 = self.cal.project(h1, h2, self.theta1, self.theta2)
        u0 = self.cal.B_cal @ (z0 - self.cal.b_cal)
        r0 = float(np.hypot(u0[0], u0[1]))
        self._update_reference_phases(h1, h2, r0)

        z = self.cal.project(h1, h2, self.theta1, self.theta2)
        u_raw = self.cal.B_cal @ (z - self.cal.b_cal)
        r_raw = float(np.hypot(u_raw[0], u_raw[1]))

        u_pre_norm = np.array([u_raw[0] / self.k_axis, u_raw[1]], dtype=float)
        r_eff = float(np.hypot(u_pre_norm[0], u_pre_norm[1]))
        frozen = r_eff < self.r_freeze_thresh or r_raw < self.r_freeze_thresh
        probe_rejected = False

        if frozen:
            phi_hat = self.last_phi_hat
            err = 0.0
            next_v = bias_v
            probe_rejected = True
        else:
            u_unit = u_pre_norm / max(r_eff, 1e-12)
            in_principal_axis_band = (
                abs(u_unit[0]) < self.axis_reject_thresh
                or abs(u_unit[1]) < self.axis_reject_thresh
            )
            if r_raw < self.r_track_thresh or in_principal_axis_band:
                probe_rejected = True
            elif (self.step_count % self.probe_interval) == 0:
                dv_probe = self.probe_dither_rad * (self.cal.vpi_v / np.pi)
                v_plus = self._clamp_v(bias_v + dv_probe)
                v_minus = self._clamp_v(bias_v - dv_probe)
                if v_plus - v_minus > 1e-6:
                    h1_p, h2_p, _, _ = self._measure_avg(uart, v_plus)
                    h1_m, h2_m, _, _ = self._measure_avg(uart, v_minus)
                    zp = self.cal.project(h1_p, h2_p, self.theta1, self.theta2)
                    zm = self.cal.project(h1_m, h2_m, self.theta1, self.theta2)
                    up = self.cal.B_cal @ (zp - self.cal.b_cal)
                    um = self.cal.B_cal @ (zm - self.cal.b_cal)
                    du = up - um
                    denom = u_raw[1] * du[1]
                    if abs(denom) > 1e-12:
                        ratio_sq = -(u_raw[0] * du[0]) / denom
                        if ratio_sq > 0.0:
                            kc_meas = float(np.sqrt(ratio_sq))
                            if 0.25 <= kc_meas <= 4.0:
                                self.k_axis = (1.0 - self.alpha_k_axis) * self.k_axis + self.alpha_k_axis * kc_meas
                                dphi = float(
                                    wrap_rad(
                                        float(np.arctan2(up[1], up[0] / self.k_axis))
                                        - float(np.arctan2(um[1], um[0] / self.k_axis))
                                    )
                                )
                                self.polarity_sign = 1 if dphi >= 0.0 else -1
                            else:
                                probe_rejected = True
                        else:
                            probe_rejected = True
                    else:
                        probe_rejected = True

            u_corr = np.array([u_raw[0] / self.k_axis, u_raw[1]], dtype=float)
            phi_hat = float(wrap_rad(float(np.arctan2(u_corr[1], u_corr[0]))))
            self.last_phi_hat = phi_hat
            err = float(wrap_rad(phi_hat - self.target_thesis_rad))
            next_v = self._clamp_v(bias_v - self.polarity_sign * self.gain * (self.cal.vpi_v / np.pi) * err)
            self.rho_ema = (1.0 - self.rho_alpha) * self.rho_ema + self.rho_alpha * abs(float(np.hypot(u_corr[0], u_corr[1])) - 1.0)

        self.step_count += 1
        return StepObservation(
            step_idx=self.step_count,
            bias_mv=bias_v * 1e3,
            next_bias_mv=next_v * 1e3,
            phi_hat_thesis_rad=phi_hat,
            phi_hat_fw_rad=float(thesis_to_fw_rad(phi_hat)),
            err_est_rad=err,
            radius=r_eff,
            h1_proj_v=float(z[1]),
            h2_proj_v=float(z[0]),
            h1_raw_v=h1,
            h2_raw_v=h2,
            dc_mv=dc_mv,
            frozen=frozen,
            probe_rejected=probe_rejected,
            recal_triggered=False,
            k_axis=self.k_axis,
            theta1_rad=self.theta1,
            theta2_rad=self.theta2,
            rho_ema=self.rho_ema,
            ch0_min_mv=tail.ch0_min_mv,
            ch0_max_mv=tail.ch0_max_mv,
            ch1_min_mv=tail.ch1_min_mv,
            ch1_max_mv=tail.ch1_max_mv,
            drops=tail.drops,
            crc=tail.crc,
        )


CONTROLLER_CLASSES: dict[str, type[BaseBiasController]] = {
    "B1": B1SingleHarmonic,
    "B2": B2RatioH1DC,
    "B3": B3DualHarmonicRatio,
    "B4": B4DiagonalAtan2,
    "AffineFrozen": AffineFrozen,
    "AffineRecal": AffineRecal,
    "AffineS3Plus": AffineS3Plus,
}


def make_controller(name: str, cal: AffineCalModel, target_thesis_rad: float, **kwargs) -> BaseBiasController:
    if name not in CONTROLLER_CLASSES:
        raise ValueError(f"Unknown controller '{name}', expected one of {sorted(CONTROLLER_CLASSES)}")
    return CONTROLLER_CLASSES[name](cal, target_thesis_rad, **kwargs)


def run_closed_loop_episode(
    uart: UARTClient,
    controller: BaseBiasController,
    seed_bias_v: float,
    n_steps: int,
    *,
    on_step: Callable[[StepObservation], None] | None = None,
) -> list[StepObservation]:
    """Execute n_steps synchronous closed-loop updates driven via `uart.meas`."""
    if n_steps < 1:
        raise ValueError(f"n_steps must be >= 1, got {n_steps}")
    cur_v = controller._clamp_v(seed_bias_v)
    history: list[StepObservation] = []
    for _ in range(n_steps):
        obs = controller.step(uart, cur_v)
        history.append(obs)
        if on_step is not None:
            on_step(obs)
        cur_v = obs.next_bias_mv * 1e-3
    return history
