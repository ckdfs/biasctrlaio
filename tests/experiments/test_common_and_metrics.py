"""Unit tests for common.py coordinate conversions, checkpointing, and metrics.py."""

from __future__ import annotations

import os
import shutil
import unittest

import numpy as np

from scripts.experiments.common import (
    RunSession,
    exp_runs_root,
    fw_to_thesis_deg,
    fw_to_thesis_rad,
    thesis_to_fw_deg,
    thesis_to_fw_rad,
    wrap_deg,
    wrap_rad,
)
from scripts.experiments.metrics import (
    circular_mean,
    circular_rms,
    cond_kappa_and_sigma_min,
    p95_abs,
    paired_repeatability_stats,
    settling_time_steps,
    short_term_sigma,
    static_rms,
    summarize_target_runs,
    temporal_rms,
    two_period_half_amp,
    unwrapped_p2p_and_slope,
)


class TestCommonAndMetrics(unittest.TestCase):
    def setUp(self) -> None:
        self.test_root = exp_runs_root() / f"_unittest_common_{os.getpid()}"
        if self.test_root.exists():
            shutil.rmtree(self.test_root)
        self.test_root.mkdir(parents=True, exist_ok=True)

    def tearDown(self) -> None:
        if self.test_root.exists():
            shutil.rmtree(self.test_root)

    def test_coordinate_conversions_and_wrapping(self) -> None:
        self.assertAlmostEqual(wrap_rad(np.pi), np.pi, places=12)
        self.assertAlmostEqual(wrap_rad(-np.pi), np.pi, places=12)
        self.assertAlmostEqual(wrap_deg(-180.0), 180.0, places=12)

        # Cardinal points: fw MIN(0)->MITP(180), QUAD(90)->QTP-(-90), MAX(180)->MATP(0), -90->QTP+(+90)
        self.assertAlmostEqual(fw_to_thesis_deg(0.0), 180.0, places=10)
        self.assertAlmostEqual(fw_to_thesis_deg(90.0), -90.0, places=10)
        self.assertAlmostEqual(fw_to_thesis_deg(180.0), 0.0, places=10)
        self.assertAlmostEqual(fw_to_thesis_deg(-90.0), 90.0, places=10)

        self.assertAlmostEqual(thesis_to_fw_deg(0.0), 180.0, places=10)
        self.assertAlmostEqual(thesis_to_fw_deg(180.0), 0.0, places=10)
        self.assertAlmostEqual(thesis_to_fw_deg(-90.0), 90.0, places=10)
        self.assertAlmostEqual(thesis_to_fw_deg(90.0), -90.0, places=10)

        angles = np.linspace(-3.0 * np.pi, 3.0 * np.pi, 25)
        rt = thesis_to_fw_rad(fw_to_thesis_rad(angles))
        np.testing.assert_allclose(wrap_rad(rt - angles), 0.0, atol=1e-12)

    def test_run_session_checkpoint_and_resume(self) -> None:
        run_dir = self.test_root / "run1"
        sess1 = RunSession("c4", run_dir=run_dir, resume=False)
        sess1.write_metadata({"board_id": "20261110-MZM"})
        sess1.mark_done("step_a", {"val": 42})
        sess1.append_csv("log.csv", ["a", "b"], [{"a": 1, "b": 2}])

        sess2 = RunSession("c4", run_dir=run_dir, resume=True)
        self.assertTrue(sess2.is_done("step_a"))
        self.assertFalse(sess2.is_done("step_b"))
        self.assertEqual(sess2.get_data("step_a"), {"val": 42})

    def test_circular_metrics_and_two_period_oscillation(self) -> None:
        # Seam wrap test around +/-pi.
        errs = np.array([np.pi - 0.02, -np.pi + 0.02])
        self.assertAlmostEqual(abs(circular_mean(errs)), np.pi, places=10)
        self.assertAlmostEqual(short_term_sigma(errs), 0.02, places=10)

        # Two-period oscillation with zero circular mean must yield A2 = 0.30 rad!
        osc = np.array([0.30, -0.30] * 10)
        self.assertAlmostEqual(circular_mean(osc), 0.0, places=10)
        self.assertAlmostEqual(two_period_half_amp(osc), 0.30, places=10)
        self.assertAlmostEqual(temporal_rms(osc), 0.30, places=10)
        self.assertAlmostEqual(circular_rms(osc), 0.30, places=10)
        self.assertAlmostEqual(p95_abs(osc), 0.30, places=10)

    def test_failure_denominator_preservation(self) -> None:
        # 4 targets: 3 near 0 error, 1 failed (empty tail with failed_mask=True -> pi penalty).
        tails = [
            np.full(20, 0.001),
            np.full(20, -0.001),
            np.full(20, 0.002),
            np.array([], dtype=float),
        ]
        failed = [False, False, False, True]
        st = static_rms(tails, failed_mask=failed)
        self.assertEqual(st["n_total"], 4)
        self.assertEqual(st["n_failed"], 1)
        self.assertEqual(st["n_valid"], 3)
        self.assertAlmostEqual(st["static_rms_rad"], np.pi / 2.0, delta=0.01)

        summary = summarize_target_runs(tails, failed_mask=failed)
        self.assertEqual(summary["pass_count"], 3)
        self.assertEqual(summary["n_total"], 4)
        self.assertAlmostEqual(summary["pass_rate"], 0.75, places=6)

    def test_settling_time_and_kappa(self) -> None:
        # 5 large steps, then 12 steps inside 20 mrad and 1 deg.
        errs = np.array([0.5, 0.3, 0.15, 0.05, 0.03] + [0.015] * 12)
        idx_1d, t_1d = settling_time_steps(errs, threshold_rad=np.deg2rad(1.0), sustain_steps=10, dt_s=0.08)
        idx_20m, t_20m = settling_time_steps(errs, threshold_rad=0.020, sustain_steps=10, dt_s=0.08)
        self.assertEqual(idx_1d, 5)
        self.assertAlmostEqual(t_1d, 5 * 0.08, places=10)
        self.assertEqual(idx_20m, 5)
        self.assertAlmostEqual(t_20m, 5 * 0.08, places=10)

        # Unsettled sequence returns (None, None).
        unsettled = np.array([0.5] * 8 + [0.01] * 5 + [0.5])
        self.assertEqual(settling_time_steps(unsettled, threshold_rad=0.02, sustain_steps=10, dt_s=0.1), (None, None))

        kappa, smin = cond_kappa_and_sigma_min(np.array([[0.01, 0.0], [0.0, 0.45]]))
        self.assertAlmostEqual(smin, 0.01, places=10)
        self.assertAlmostEqual(kappa, 45.0, places=10)

        # Repeatability & drift slope
        rng = np.random.default_rng(123)
        pairs = rng.normal(0.0, 0.5, size=200)
        rep = paired_repeatability_stats(pairs)
        self.assertEqual(rep["n_pairs"], 100.0)
        self.assertAlmostEqual(rep["sigma_single"], 0.5, delta=0.15)

        ts = np.array([0.0, 1800.0, 3600.0])
        vals = np.array([100.0, 150.0, 200.0])
        drift = unwrapped_p2p_and_slope(ts, vals)
        self.assertAlmostEqual(drift["net_change"], 100.0, places=8)
        self.assertAlmostEqual(drift["slope_per_hour"], 100.0, places=8)


if __name__ == "__main__":
    unittest.main()
