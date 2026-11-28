"""End-to-end dry-run tests for C1-C6 experiment runners and analyze_and_plot.py."""

from __future__ import annotations

import os
import shutil
import unittest

from scripts.experiments import analyze_and_plot, run_c1_c2, run_c3, run_c4, run_c5, run_c6
from scripts.experiments.common import exp_runs_root


class TestRunnersDryRun(unittest.TestCase):
    def setUp(self) -> None:
        self.test_root = exp_runs_root() / f"_unittest_runners_{os.getpid()}"
        if self.test_root.exists():
            shutil.rmtree(self.test_root)
        self.test_root.mkdir(parents=True, exist_ok=True)

    def tearDown(self) -> None:
        if self.test_root.exists():
            shutil.rmtree(self.test_root)

    def test_c1_c2_dry_run_and_analyze(self) -> None:
        run_dir = self.test_root / "c1_c2"
        out_dir = self.test_root / "out_c1_c2"
        rc = run_c1_c2.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--step-repeats",
                "2",
                "--repeatability-chunks",
                "2",
                "--repeatability-pairs",
                "2",
                "--c1-bias-blocks",
                "4",
            ]
        )
        self.assertEqual(rc, 0)
        self.assertTrue((run_dir / "summary.json").is_file())
        self.assertTrue((run_dir / "metadata.json").is_file())
        res = analyze_and_plot.analyze_c1_c2(run_dir, out_dir)
        self.assertTrue((out_dir / "c1_c2_fft_and_twait.png").is_file())
        self.assertIn("figure", res)

    def test_c3_dry_run_resume_and_analyze(self) -> None:
        run_dir = self.test_root / "c3"
        out_dir = self.test_root / "out_c3"
        rc = run_c3.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--sessions",
                "2",
                "--n-points",
                "37",
                "--blocks-per-point",
                "2",
                "--n-restarts",
                "3",
            ]
        )
        self.assertEqual(rc, 0)
        # Resume should reuse completed checkpoints without error.
        rc_resume = run_c3.main(
            [
                "--dry-run",
                "--resume",
                "--run-dir",
                str(run_dir),
                "--sessions",
                "2",
                "--n-points",
                "37",
                "--blocks-per-point",
                "2",
                "--n-restarts",
                "3",
            ]
        )
        self.assertEqual(rc_resume, 0)
        analyze_and_plot.analyze_c3(run_dir, out_dir)
        self.assertTrue((out_dir / "c3_cv_comparison.png").is_file())

    def test_c4_dry_run_and_analyze(self) -> None:
        run_dir = self.test_root / "c4"
        out_dir = self.test_root / "out_c4"
        rc = run_c4.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--sessions",
                "1",
                "--n-steps",
                "25",
                "--tail-steps",
                "10",
                "--cal-points",
                "25",
                "--no-dense",
                "--skip-gain-sweep",
                "--methods",
                "B4,AffineFrozen",
            ]
        )
        self.assertEqual(rc, 0)
        analyze_and_plot.analyze_c4(run_dir, out_dir)
        self.assertTrue((out_dir / "c4_table_5_5.csv").is_file())
        self.assertTrue((out_dir / "c4_method_comparison.png").is_file())

    def test_c6_dry_run_and_rf_gate(self) -> None:
        run_dir = self.test_root / "c6"
        out_dir = self.test_root / "out_c6"
        # Non-dry-run without --confirm-rf must fail fast!
        with self.assertRaises(PermissionError):
            run_c6.main(["--run-dir", str(run_dir)])

        rc = run_c6.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--rf-powers-dbm",
                "0,13",
                "--sessions",
                "1",
                "--n-steps",
                "20",
                "--tail-steps",
                "10",
                "--cal-points",
                "25",
                "--methods",
                "B2,AffineFrozen,AffineRecal,AffineS3Plus",
                "--run-j0-sweep",
                "--j0-powers-dbm",
                "13,16.6",
            ]
        )
        self.assertEqual(rc, 0)
        analyze_and_plot.analyze_c6(run_dir, out_dir)
        self.assertTrue((out_dir / "c6_table_5_7.csv").is_file())
        self.assertTrue((out_dir / "c6_rf_robustness.png").is_file())

    def test_c5_dry_run_and_analyze(self) -> None:
        import csv

        run_dir = self.test_root / "c5"
        out_dir = self.test_root / "out_c5"
        with self.assertRaises(PermissionError):
            run_c5.main(["--run-dir", str(run_dir)])

        rc = run_c5.main(
            [
                "--dry-run",
                "--run-dir",
                str(run_dir),
                "--runs",
                "1",
                "--duration-s",
                "0.05",
                "--anchor-interval-s",
                "0.05",
                "--targets-thesis-deg=90,45",
                "--step-trials",
                "1",
                "--disturbance-trials",
                "1",
            ]
        )
        self.assertEqual(rc, 0)
        with (run_dir / "c5_tel_thesis_90deg_run0.csv").open("r", encoding="utf-8") as fh:
            elapsed_vals = [float(row["elapsed_s"]) for row in csv.DictReader(fh)]
        self.assertGreater(len(elapsed_vals), 5)
        self.assertTrue(all(b > a for a, b in zip(elapsed_vals, elapsed_vals[1:])))
        analyze_and_plot.analyze_c5(run_dir, out_dir)
        self.assertTrue((out_dir / "c5_long_term.csv").is_file())
        self.assertTrue((out_dir / "c5_stability.png").is_file())


if __name__ == "__main__":
    unittest.main()
