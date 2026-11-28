"""Unit tests for SDS824XClient waveform decoding, safety checks, and C1 VA scope runner."""

from __future__ import annotations

import json
import os
import shutil
import unittest

from scripts.experiments import run_c1_va_scope
from scripts.experiments.common import exp_runs_root
from scripts.experiments.sds824x_client import SDS824XClient, SDS824XError
from scripts.experiments.sim_plant import MZMPlantConfig, SimulatedMZMPlant


class TestSDS824XAndC1VaScope(unittest.TestCase):
    def setUp(self) -> None:
        self.test_root = exp_runs_root() / f"_unittest_sds824x_{os.getpid()}"
        if self.test_root.exists():
            shutil.rmtree(self.test_root)
        self.test_root.mkdir(parents=True, exist_ok=True)

    def tearDown(self) -> None:
        if self.test_root.exists():
            shutil.rmtree(self.test_root)

    def test_sds824x_client_rejects_gnd_and_arm_hang(self) -> None:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261128))
        sock = run_c1_va_scope.SimulatedSDS824XSocket(plant)
        with SDS824XClient(
            cmd_sleep_s=0.0,
            waveform_sleep_s=0.0,
            settle_scale=0.0,
            socket_factory=lambda *a, **kw: sock,
        ) as scope:
            ident = scope.identify()
            self.assertIn("SDS824X", ident)
            with self.assertRaises(ValueError):
                scope.setup_channel(1, coupling="GND")
            sock.trig_status = "Arm"
            with self.assertRaises(SDS824XError):
                scope.wait_trigger_status(("Auto", "Trig'd"), timeout_s=0.05, poll_s=0.0)
            sock.trig_status = "Auto"
            t, v, meta = scope.capture_auto_waveform(1, settle_s=0.0, stop_wait_s=0.0)
            self.assertEqual(len(t), int(meta["point_num"]))
            self.assertEqual(len(v), len(t))

        self.assertTrue(sock.closed)

    def test_dry_run_c1_va_scope_gain_thd_and_tau_lp(self) -> None:
        run_dir = self.test_root / "run"
        out_dir = self.test_root / "out"
        rc = run_c1_va_scope.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--out-dir",
                str(out_dir),
                "--harm-frames",
                "3",
                "--noise-frames",
                "4",
            ]
        )
        self.assertEqual(rc, 0)
        self.assertTrue((run_dir / "summary.json").is_file())
        self.assertTrue((out_dir / "c1_va_scope_summary.json").is_file())
        self.assertTrue((out_dir / "c1_va_pilot_sweep.csv").is_file())
        self.assertTrue((out_dir / "c1_va_scope_char.png").is_file())

        summary = json.loads((out_dir / "c1_va_scope_summary.json").read_text(encoding="utf-8"))
        flat_159 = summary["part_a_flatness"]["pilot_159_2mv"]
        self.assertAlmostEqual(flat_159["gain_mean"], run_c1_va_scope.DESIGN_PILOT_INJ_GAIN, places=3)
        self.assertLess(flat_159["bias_ptp_pct"], 0.5)
        self.assertLess(abs(summary["part_c_steps"]["rising"]["tau_err_pct"]), 3.0)
        self.assertLess(abs(summary["part_c_steps"]["falling"]["tau_err_pct"]), 3.0)
        asd_1k = summary["part_b_noise"]["0V"]["asd_1k_band_median_nv_rthz"]
        self.assertGreater(asd_1k, 25.0)
        self.assertLess(asd_1k, 120.0)


if __name__ == "__main__":
    unittest.main()
