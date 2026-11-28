"""Unit tests for dc_psi.py 17-point In-Situ DC PSI solver and UART sequence."""

from __future__ import annotations

import unittest

import numpy as np

from scripts.experiments.common import thesis_to_fw_rad, wrap_rad
from scripts.experiments.dc_psi import compute_scan_window, fit_dc_psi_waveform, measure_dc_psi
from scripts.experiments.sim_plant import MZMPlantConfig, SimulatedMZMPlant, SimulatedUARTTransport
from scripts.experiments.uart_client import UARTClient


class SpyUARTClient(UARTClient):
    def __init__(self, transport) -> None:
        super().__init__(transport=transport)
        self.meas_calls: list[tuple[float, float, int, int]] = []

    def meas(self, bias_mv: float, pilot_mv: float, wait_blocks: int, n_blocks: int, *, timeout_s: float | None = None):
        self.meas_calls.append((bias_mv, pilot_mv, wait_blocks, n_blocks))
        return super().meas(bias_mv, pilot_mv, wait_blocks, n_blocks, timeout_s=timeout_s)


class TestDcPsi(unittest.TestCase):
    def test_synthetic_full_360_recovery_and_compressed_j0(self) -> None:
        rng = np.random.default_rng(20261126)
        vpi_true = 5.382
        v_lock = -0.340

        for phi_deg in np.arange(-180.0, 180.0, 15.0):
            phi_true = float(wrap_rad(np.radians(phi_deg)))
            _, voltages = compute_scan_window(v_lock, vpi_est_v=5.15)  # 4.5% off initial Vpi estimate
            a_mv, b_mv = 400.0, 390.0
            noise = rng.normal(0.0, 0.25, size=voltages.size)
            dc_mv = a_mv + b_mv * np.cos(np.pi * (voltages - v_lock) / vpi_true + phi_true) + noise

            res = fit_dc_psi_waveform(
                voltages,
                dc_mv,
                v_lock_v=v_lock,
                vpi_est_v=5.15,
                dc_pre_mv=a_mv + b_mv * np.cos(phi_true),
                dc_post_mv=a_mv + b_mv * np.cos(phi_true) + 0.35,
            )
            err_deg = abs(float(np.degrees(wrap_rad(res.phi_true_thesis_rad - phi_true))))
            self.assertLess(err_deg, 0.25, f"Failed at phi_deg={phi_deg}: err={err_deg}")
            self.assertAlmostEqual(res.phi_true_fw_rad, thesis_to_fw_rad(res.phi_true_thesis_rad), places=10)
            self.assertAlmostEqual(res.vpi_v, vpi_true, delta=0.05)
            self.assertAlmostEqual(res.drift_mv, 0.35, places=6)
            self.assertGreater(res.sigma_phi_deg, 0.0)
            self.assertLess(res.sigma_phi_deg, 0.5)

        # J0-compressed waveform (contrast reduced by 6x, e.g. high RF loading).
        phi_j0 = np.radians(42.0)
        _, v_j0 = compute_scan_window(v_lock, vpi_est_v=vpi_true)
        dc_j0 = 400.0 + 60.0 * np.cos(np.pi * (v_j0 - v_lock) / vpi_true + phi_j0) + rng.normal(0.0, 0.15, size=17)
        res_j0 = fit_dc_psi_waveform(v_j0, dc_j0, v_lock_v=v_lock, vpi_est_v=vpi_true)
        self.assertLess(abs(float(np.degrees(wrap_rad(res_j0.phi_true_thesis_rad - phi_j0)))), 0.5)

    def test_measure_dc_psi_protocol_timings_and_clamp(self) -> None:
        plant = SimulatedMZMPlant(MZMPlantConfig(noise_dc_mv=0.05, seed=7))
        uart = SpyUARTClient(SimulatedUARTTransport(plant))
        # Test near positive rail to exercise [-9.8, 9.8] clamping.
        v_lock_mv = 4500.0
        vpi_est_mv = 5382.0
        res = measure_dc_psi(uart, v_lock_mv, vpi_est_mv)

        # 1 pre + 17 scan + 1 post = 19 meas calls, all with pilot=0, 3 blocks/pt
        self.assertEqual(len(uart.meas_calls), 19)
        self.assertEqual(uart.meas_calls[0][1:], (0.0, 0, 3))
        self.assertEqual(uart.meas_calls[1][1:], (0.0, 12, 3))
        for call in uart.meas_calls[2:18]:
            self.assertEqual(call[1:], (0.0, 7, 3))
        self.assertEqual(uart.meas_calls[18], (v_lock_mv, 0.0, 12, 3))

        self.assertGreaterEqual(min(res.voltages_v), -9.8 - 1e-9)
        self.assertLessEqual(max(res.voltages_v), 9.8 + 1e-9)
        true_phi = plant.true_phase_thesis_rad(v_lock_mv * 1e-3)
        self.assertLess(abs(float(np.degrees(wrap_rad(res.phi_true_thesis_rad - true_phi)))), 0.25)


if __name__ == "__main__":
    unittest.main()
