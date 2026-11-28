"""Unit tests for DM858EClient SCPI safety checks and C1 DC linearity sweep/DNL pipeline."""

from __future__ import annotations

import math
import os
import shutil
import unittest

from scripts.experiments import run_c1_dc_linearity
from scripts.experiments.common import exp_runs_root
from scripts.experiments.dm858e_client import DM858EClient, DM858EError
from scripts.experiments.sim_plant import MZMPlantConfig, SimulatedMZMPlant


class TestDM858EAndC1DcLinearity(unittest.TestCase):
    def setUp(self) -> None:
        self.test_root = exp_runs_root() / f"_unittest_dm858e_{os.getpid()}"
        if self.test_root.exists():
            shutil.rmtree(self.test_root)
        self.test_root.mkdir(parents=True, exist_ok=True)

    def tearDown(self) -> None:
        if self.test_root.exists():
            shutil.rmtree(self.test_root)

    def test_dm858e_client_overload_error_queue_and_configure(self) -> None:
        plant = SimulatedMZMPlant(MZMPlantConfig(seed=20261128))
        sock = run_c1_dc_linearity.SimulatedDM858ESocket(plant)
        with DM858EClient(connect_settle_s=0.0, cmd_settle_s=0.0, socket_factory=lambda *a, **kw: sock) as dmm:
            ident = dmm.identify()
            self.assertIn("DM858E", ident)
            rng, nplc = dmm.configure_dc_voltage(vrange=10.0, nplc=5.0)
            self.assertEqual(rng, 10.0)
            self.assertEqual(nplc, 5.0)

            # Drive plant past 10 V range ceiling -> overload returns inf; read_finite must fail.
            plant.bias_v = 10.005
            self.assertTrue(math.isinf(dmm.read()))
            with self.assertRaises(DM858EError):
                dmm.read_finite()

            # Non-zero error queue must raise DM858EError.
            sock.errors.append('-222,"Data out of range"')
            with self.assertRaises(DM858EError):
                dmm.check_errors()

        self.assertEqual(sock.vrange, 1000.0)
        self.assertTrue(sock.closed)

    def test_code_to_cmd_mv_and_dry_run_c1_dc_linearity(self) -> None:
        self.assertEqual(run_c1_dc_linearity.code_to_cmd_mv(0), -10000.0)
        self.assertEqual(run_c1_dc_linearity.code_to_cmd_mv(65535), 10000.0)
        self.assertEqual(len(run_c1_dc_linearity.default_full_sweep_codes()), 129)
        self.assertEqual(len(run_c1_dc_linearity.default_single_code_window()), 65)

        run_dir = self.test_root / "run"
        out_dir = self.test_root / "out"
        rc = run_c1_dc_linearity.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--out-dir",
                str(out_dir),
            ]
        )
        self.assertEqual(rc, 0)
        self.assertTrue((run_dir / "summary.json").is_file())
        self.assertTrue((out_dir / "c1_dc_linearity_summary.json").is_file())
        self.assertTrue((out_dir / "c1_dc_full_sweep.csv").is_file())
        self.assertTrue((out_dir / "c1_dc_single_code_dnl.csv").is_file())
        self.assertTrue((out_dir / "c1_dc_linearity.png").is_file())

        import json

        summary = json.loads((out_dir / "c1_dc_linearity_summary.json").read_text(encoding="utf-8"))
        full_m = summary["full_range_metrics"]
        single_m = summary["single_code_metrics"]
        self.assertEqual(full_m["n_rows"], 258)
        self.assertAlmostEqual(full_m["dc_gain_vs_5v_fit"], 4.001, places=2)
        self.assertTrue(single_m["monotonic"])
        self.assertEqual(single_m["msb_carry_transition"], "32767->32768")
        self.assertLess(abs(single_m["dnl_global_max_abs_lsb"]), 0.2)


if __name__ == "__main__":
    unittest.main()
