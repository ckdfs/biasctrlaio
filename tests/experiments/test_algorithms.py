"""Unit tests for algorithms.py (calibration + B1-B4, Frozen, Recal, S3+ on simulated MZM plant)."""

from __future__ import annotations

import unittest

import numpy as np

from scripts.experiments.algorithms import (
    AffineFrozen,
    AffineRecal,
    AffineS3Plus,
    B1SingleHarmonic,
    B2RatioH1DC,
    B3DualHarmonicRatio,
    B4DiagonalAtan2,
    calibrate_affine_from_sweep,
    run_closed_loop_episode,
)
from scripts.experiments.common import wrap_rad
from scripts.experiments.sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from scripts.experiments.uart_client import UARTClient


def _calibrate_on_plant(plant: SimulatedMZMPlant, n_pts: int = 91, pilot_mv: float = 159.2):
    uart = UARTClient(transport=SimulatedUARTTransport(plant))
    v_start = -5.5
    vs = np.linspace(v_start, v_start + 2.0 * plant.cfg.vpi_dc_v, n_pts)
    h1s, h2s, dcs = [], [], []
    for idx, vv in enumerate(vs):
        rows = uart.meas(float(vv * 1e3), pilot_mv, 12 if idx == 0 else 3, 2)
        h1s.append(complex(np.mean([r.h1_complex_v for r in rows])))
        h2s.append(complex(np.mean([r.h2_complex_v for r in rows])))
        dcs.append(float(np.mean([r.dc_mv * 1e-3 for r in rows])))
    cal = calibrate_affine_from_sweep(vs, h1s, h2s, dcs, pilot_v=pilot_mv * 1e-3)
    return uart, cal


class TestAlgorithms(unittest.TestCase):
    def test_calibration_and_b1_b4_vs_affine_frozen(self) -> None:
        plant = SimulatedMZMPlant(
            MZMPlantConfig(
                cross_coupling_12=0.05,
                cross_coupling_21=-0.04,
                coherent_offset_h1_uv=complex(120.0, -80.0),
                coherent_offset_h2_uv=complex(-180.0, 90.0),
                noise_h1_uv=2.0,
                noise_h2_uv=2.0,
                seed=11,
            )
        )
        uart, cal = _calibrate_on_plant(plant)
        self.assertAlmostEqual(cal.vpi_v, plant.cfg.vpi_dc_v, delta=0.03)
        self.assertAlmostEqual(cal.null_v_fw, plant.cfg.null_v_fw, delta=0.05)
        self.assertGreater(cal.kappa, 10.0)

        # At QTP- (-90 deg thesis = +90 deg fw), B1 loses first-order sensitivity while AffineFrozen locks < 0.25 deg.
        tgt_quad = -0.5 * np.pi
        seed_v = cal.target_voltage_v(tgt_quad) + 0.5 * (cal.vpi_v / np.pi)

        c_aff = AffineFrozen(cal, tgt_quad, gain=0.30)
        hist_aff = run_closed_loop_episode(uart, c_aff, seed_v, 40)
        err_aff = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(hist_aff[-1].next_bias_mv * 1e-3) - tgt_quad))))
        self.assertLess(err_aff, 0.25)

        # At 45 deg with strong cross-coupling & coherent offset, AffineFrozen beats B4.
        tgt_45 = np.deg2rad(45.0)
        seed_45 = cal.target_voltage_v(tgt_45) + 0.4 * (cal.vpi_v / np.pi)
        c_b4 = B4DiagonalAtan2(cal, tgt_45, gain=0.30)
        hist_b4 = run_closed_loop_episode(uart, c_b4, seed_45, 40)
        err_b4 = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(hist_b4[-1].next_bias_mv * 1e-3) - tgt_45))))

        c_aff45 = AffineFrozen(cal, tgt_45, gain=0.30)
        hist_aff45 = run_closed_loop_episode(uart, c_aff45, seed_45, 40)
        err_aff45 = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(hist_aff45[-1].next_bias_mv * 1e-3) - tgt_45))))
        self.assertLess(err_aff45, 0.25)
        self.assertGreater(err_b4, err_aff45 * 2.0)

        # B3 converges at non-quadrature 45 deg and exercises quadrature extremum probe at 90 deg.
        c_b3 = B3DualHarmonicRatio(cal, tgt_45, gain=0.30)
        hist_b3 = run_closed_loop_episode(uart, c_b3, cal.target_voltage_v(tgt_45) + 0.2 * (cal.vpi_v / np.pi), 40)
        err_b3 = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(hist_b3[-1].next_bias_mv * 1e-3) - tgt_45))))
        self.assertLess(err_b3, 3.0)

        c_b3_q = B3DualHarmonicRatio(cal, 0.5 * np.pi, gain=0.30)
        hist_b3_q = run_closed_loop_episode(uart, c_b3_q, cal.target_voltage_v(0.5 * np.pi) + 0.05, 5)
        self.assertEqual(len(hist_b3_q), 5)

        # B1 converges near MATP (0 deg thesis) up to uncalibrated cross-coupling (~2.3 deg),
        # while at QTP (-90 deg) B1 stalls at large error due to zero first-order slope.
        c_b1 = B1SingleHarmonic(cal, 0.0, gain=0.30)
        hist_b1 = run_closed_loop_episode(uart, c_b1, cal.target_voltage_v(0.0) + 0.4 * (cal.vpi_v / np.pi), 30)
        err_b1 = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(hist_b1[-1].next_bias_mv * 1e-3)))))
        self.assertLess(err_b1, 3.0)

        c_b1_q = B1SingleHarmonic(cal, tgt_quad, gain=0.30)
        hist_b1_q = run_closed_loop_episode(uart, c_b1_q, seed_v, 30)
        err_b1_q = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(hist_b1_q[-1].next_bias_mv * 1e-3) - tgt_quad))))
        self.assertGreater(err_b1_q, 5.0)

    def test_b2_rf_bias_vs_s3_plus_vpi_drift_and_j0_freeze(self) -> None:
        # Calibrate baseline at RF Off, Vpi nominal.
        plant = SimulatedMZMPlant(MZMPlantConfig(noise_h1_uv=1.0, noise_h2_uv=1.0, seed=99))
        uart, cal0 = _calibrate_on_plant(plant)

        # Enable 13 dBm RF + 2.0% Vpi drift + 10 deg pilot injection phase shift.
        plant.cfg.rf_enabled = True
        plant.cfg.rf_power_dbm = 13.0
        plant.cfg.vpi_drift_rel = 0.02
        plant.cfg.pilot_inj_phase_rad += np.deg2rad(10.0)

        tgt_45 = np.deg2rad(45.0)
        seed_v = cal0.target_voltage_v(tgt_45) + 0.3

        c_b2 = B2RatioH1DC(cal0, tgt_45, gain=0.25)
        h_b2 = run_closed_loop_episode(uart, c_b2, seed_v, 45)
        err_b2 = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(h_b2[-1].next_bias_mv * 1e-3) - tgt_45))))
        self.assertGreater(err_b2, 5.0)  # B2 suffers severe static error under 13 dBm RF

        # AffineFrozen suffers uncompensated Vpi drift / phase shift, while S3+ tracks both!
        c_frz = AffineFrozen(cal0, tgt_45, gain=0.30)
        h_frz = run_closed_loop_episode(uart, c_frz, seed_v, 45)
        err_frz = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(h_frz[-1].next_bias_mv * 1e-3) - tgt_45))))

        c_s3p = AffineS3Plus(cal0, tgt_45, gain=0.30, probe_interval=1)
        h_s3p = run_closed_loop_episode(uart, c_s3p, seed_v, 45)
        err_s3p = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(h_s3p[-1].next_bias_mv * 1e-3) - tgt_45))))
        self.assertLess(err_s3p, 0.35)
        self.assertLess(err_s3p, err_frz)
        self.assertFalse(h_s3p[-1].probe_rejected)

        # Principal-axis rejection: near 0 deg (MATP), |u_y| < 0.22 must set probe_rejected=True.
        c_s3p_axis = AffineS3Plus(cal0, 0.0, gain=0.30)
        h_axis = run_closed_loop_episode(uart, c_s3p_axis, cal0.target_voltage_v(0.0), 5)
        self.assertTrue(all(h.probe_rejected for h in h_axis))

        # J0 zero-crossing freeze protection: cross J0(m_RF) = 0 -> ||u|| < 0.15 freezes bias!
        plant.cfg.rf_power_dbm = 16.55
        last_v = h_s3p[-1].next_bias_mv * 1e-3
        h_freeze = run_closed_loop_episode(uart, c_s3p, last_v, 10)
        self.assertTrue(all(h.frozen for h in h_freeze))
        self.assertAlmostEqual(h_freeze[-1].next_bias_mv, last_v * 1e3, places=6)

        # AffineRecal updates cleanly when recalibrate() is called.
        plant.cfg.rf_enabled = False
        plant.cfg.vpi_drift_rel = 0.0
        _, cal_new = _calibrate_on_plant(plant)
        c_recal = AffineRecal(cal0, tgt_45, gain=0.30)
        c_recal.recalibrate(cal_new)
        h_rec = run_closed_loop_episode(uart, c_recal, seed_v, 30)
        err_rec = abs(float(np.degrees(wrap_rad(plant.true_phase_thesis_rad(h_rec[-1].next_bias_mv * 1e-3) - tgt_45))))
        self.assertLess(err_rec, 0.25)

    def test_bidirectional_hysteretic_chirality_stays_positive(self) -> None:
        plant = SimulatedMZMPlant(MZMPlantConfig(noise_h1_uv=1.0, noise_h2_uv=1.0, seed=7))
        v_fwd = np.linspace(-5.8, -5.8 + 2.0 * plant.cfg.vpi_dc_v, 181)
        v_rev = v_fwd[::-1].copy()
        step_rad = float(np.pi * (v_fwd[1] - v_fwd[0]) / plant.cfg.vpi_dc_v)
        hyst_rad = 0.044  # > step_rad (~0.035 rad), matching real LiNbO3 fwd-vs-rev lag

        def _synth(vs: np.ndarray, offset_rad: float):
            h1s, h2s, dcs, phis = [], [], [], []
            for vv in vs:
                plant.cfg.phase_drift_rad = offset_rad
                blk = plant.observe_block(float(vv), 0.1592)
                h1s.append(complex(blk["h1_i_uv"], blk["h1_q_uv"]) * 1e-6)
                h2s.append(complex(blk["h2_i_uv"], blk["h2_q_uv"]) * 1e-6)
                dcs.append(float(blk["dc_mv"]) * 1e-3)
                phis.append(plant.true_phase_thesis_rad(float(vv)))
            return np.asarray(h1s), np.asarray(h2s), np.asarray(dcs), np.asarray(phis)

        h1_f, h2_f, dc_f, phi_f = _synth(v_fwd, 0.5 * hyst_rad)
        h1_r, h2_r, dc_r, phi_r = _synth(v_rev, -0.5 * hyst_rad)
        cal = calibrate_affine_from_sweep(
            np.concatenate([v_fwd, v_rev]),
            np.concatenate([h1_f, h1_r]),
            np.concatenate([h2_f, h2_r]),
            np.concatenate([dc_f, dc_r]),
            pilot_v=0.1592,
        )
        self.assertGreater(cal.a1_signed, 0.0)
        z = np.column_stack(
            [
                h2_f.real * np.cos(cal.h2_ref_phase) + h2_f.imag * np.sin(cal.h2_ref_phase),
                h1_f.real * np.cos(cal.h1_ref_phase) + h1_f.imag * np.sin(cal.h1_ref_phase),
            ]
        )
        u = (z - cal.b_cal) @ cal.B_cal.T
        phi_hat = np.arctan2(u[:, 1], u[:, 0])
        err_fwd_zero_center = wrap_rad(phi_hat - phi_f)
        # Centered phase error once 0.5*hyst_rad common shift is subtracted must stay < 10 mrad.
        err_centered = wrap_rad(err_fwd_zero_center - np.mean(err_fwd_zero_center))
        self.assertLess(float(np.sqrt(np.mean(err_centered**2))) * 1e3, 10.0)


if __name__ == "__main__":
    unittest.main()
