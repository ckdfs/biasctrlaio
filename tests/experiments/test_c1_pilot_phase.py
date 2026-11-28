"""Unit tests for dual-channel SDS824X capture, phasor phase sign, and C1 pilot phase runner."""

from __future__ import annotations

import cmath
import json
import math
import os
import shutil
import unittest

import numpy as np

from scripts.experiments import run_c1_pilot_phase
from scripts.experiments.common import exp_runs_root
from scripts.experiments.sds824x_client import SDS824XClient, SDS824XError
from scripts.experiments.sim_plant import MZMPlantConfig, SimulatedMZMPlant


class TestC1PilotPhase(unittest.TestCase):
    def setUp(self) -> None:
        self.test_root = exp_runs_root() / f"_unittest_pilot_phase_{os.getpid()}"
        if self.test_root.exists():
            shutil.rmtree(self.test_root)
        self.test_root.mkdir(parents=True, exist_ok=True)

    def tearDown(self) -> None:
        if self.test_root.exists():
            shutil.rmtree(self.test_root)

    def test_analyze_dual_channel_transfer_recovers_exact_design_lag_and_gain(self) -> None:
        fs = 5e6
        n = 100_000
        t = np.arange(n, dtype=np.float64) / fs - 0.01
        f0 = 1000.0
        h_des = run_c1_pilot_phase.design_pilot_transfer(f0)
        des_extra_deg = math.degrees(cmath.phase(-h_des))
        self.assertAlmostEqual(abs(h_des), 0.0942487, places=6)
        self.assertAlmostEqual(des_extra_deg, -18.7534, places=3)

        v_src = 2.5 + 1.592 * np.cos(2.0 * math.pi * f0 * t + 0.42)
        v_va = -0.5 + 1.592 * abs(h_des) * np.cos(2.0 * math.pi * f0 * t + 0.42 + cmath.phase(h_des))
        res = run_c1_pilot_phase.analyze_dual_channel_transfer(t, v_va, t, v_src)
        self.assertAlmostEqual(res["f0_hz"], f0, places=3)
        self.assertAlmostEqual(res["gain_direct"], abs(h_des), places=5)
        self.assertAlmostEqual(res["extra_phase_deg"], des_extra_deg, places=2)
        self.assertAlmostEqual(res["total_phase_deg"], math.degrees(cmath.phase(h_des)), places=2)
        self.assertAlmostEqual(res["ch2_dc_mean_v"], 2.5, places=3)
        self.assertAlmostEqual(res["ch1_dc_mean_v"], -0.5, places=3)

    def test_capture_dual_auto_waveforms_reads_and_validates_alignment(self) -> None:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261128))
        sock = run_c1_pilot_phase.SimulatedDualSDS824XSocket(plant)
        with SDS824XClient(
            cmd_sleep_s=0.0,
            waveform_sleep_s=0.0,
            settle_scale=0.0,
            socket_factory=lambda *a, **kw: sock,
        ) as scope:
            scope.set_mdepth("100K")
            scope.setup_timebase(2e-3, delay_s=0.0)
            (t1, v1, m1), (t2, v2, m2) = scope.capture_dual_auto_waveforms(1, 2, settle_s=0.0, stop_wait_s=0.0)
            self.assertEqual(len(t1), len(t2))
            self.assertAlmostEqual(float(m1["interval"]), float(m2["interval"]), places=15)
            self.assertEqual(len(v1), 100_000)
            self.assertEqual(len(v2), 100_000)

            orig_gen = sock._generate_channel_waveform

            def _bad_interval(ch: int):
                saved = sock.delay
                if ch == 2:
                    sock.delay = 1e-4
                try:
                    return orig_gen(ch)
                finally:
                    sock.delay = saved

            sock._generate_channel_waveform = _bad_interval
            with self.assertRaises(SDS824XError):
                scope.capture_dual_auto_waveforms(1, 2, settle_s=0.0, stop_wait_s=0.0)

    def test_dry_run_c1_pilot_phase_runner_and_outputs(self) -> None:
        run_dir = self.test_root / "run"
        out_dir = self.test_root / "out"
        rc = run_c1_pilot_phase.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--out-dir",
                str(out_dir),
                "--frames",
                "3",
            ]
        )
        self.assertEqual(rc, 0)
        self.assertTrue((run_dir / "summary.json").is_file())
        self.assertTrue((out_dir / "c1_pilot_phase_summary.json").is_file())
        self.assertTrue((out_dir / "c1_pilot_phase_sweep.csv").is_file())
        self.assertTrue((out_dir / "c1_pilot_phase_char.png").is_file())

        summary = json.loads((out_dir / "c1_pilot_phase_summary.json").read_text(encoding="utf-8"))
        dc_159 = summary["dc_reference"]["pilot_159_2mv"]
        self.assertAlmostEqual(dc_159["extra_phase_deg_mean"], -18.753, delta=0.1)
        self.assertAlmostEqual(dc_159["gain_direct_mean"], 0.09425, delta=0.001)
        self.assertLess(summary["cross_bias_summary"]["extra_phase_deg_ptp"], 0.3)
        self.assertAlmostEqual(dc_159["ch2_dc_mean_v"], 2.5, delta=0.05)


if __name__ == "__main__":
    unittest.main()
