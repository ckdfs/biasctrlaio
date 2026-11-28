# Tasks: Phase 7 — Single-MZM Complex-Phasor Affine Bias Control Thesis Experiments (C1–C6)

- [x] t0_preflight_plan Verify read-only UART (/dev/cu.usbmodem102) and DG922 Pro (192.168.99.115:5025) connectivity, author 07-affine-thesis-experiments OpenSpec artifacts, and sync SQLite task DAG
  owned: openspec/changes/07-affine-thesis-experiments/
  depends:
  verify: test -f openspec/changes/07-affine-thesis-experiments/proposal.md && test -f openspec/changes/07-affine-thesis-experiments/design.md && test -f openspec/changes/07-affine-thesis-experiments/tasks.md && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t1_fw_exp_cli Implement lightweight experiment diagnostic and telemetry CLI (hw info, adc stat, adc dump, meas, step, telemetry on|off) with block min/max and I/Q stats within STM32H503 RAM/Flash and 64 kSPS ISR budgets
  owned: app/CMakeLists.txt, app/src/, tests/ztest/, docs/architecture.md, docs/user-guide.md
  depends: t0_preflight_plan
  verify: bash -ec 'source scripts/env.sh; west build -b biasctrl_h503 -d build/h503_p7 app --pristine; python3 scripts/check_mem_budget.py --elf build/h503_p7/zephyr/zephyr.elf' && python3 scripts/run_qemu_ztest.py --build-dir build/ztest_p7 --pristine --timeout 180 && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t2_fw_thesis_affine Upgrade onboard MZM calibration and closed-loop observer to thesis Chapter 5 complex-phasor affine formulation (principal-axis projection, O(1)-RAM ellipse + CH1 DC regression, pure circular PI without spring/obs_dc, T_wait discard, r<0.15 freeze, recalibration trigger, cal_mzm_record_v2)
  owned: lib/dsp/, lib/control/, app/src/, tests/host/dsp/, tests/host/control/, tests/host/sim/, tests/ztest/, docs/architecture.md, docs/design/algorithm.md, docs/design/optical.md, docs/user-guide.md
  depends: t1_fw_exp_cli
  verify: cmake -S tests/host -B build/host-tests -G Ninja && cmake --build build/host-tests && ctest --test-dir build/host-tests --output-on-failure && bash -ec 'source scripts/env.sh; west build -b biasctrl_h503 -d build/h503_p7 app --pristine; python3 scripts/check_mem_budget.py --elf build/h503_p7/zephyr/zephyr.elf' && python3 scripts/run_qemu_ztest.py --build-dir build/ztest_p7 --pristine --timeout 180 && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t3_host_exp_suite Build Python experiment automation suite (UART client, fail-safe DG922 Pro 50-ohm SCPI client, 17-point In-Situ DC PSI, synchronous B1/B2/B3/B4/Affine-frozen/Affine-recal/S3+ controllers, circular metrics, plotting/reporting) and unittest verification
  owned: scripts/experiments/, tests/experiments/
  depends: t1_fw_exp_cli
  verify: python3 -m unittest discover -s tests/experiments -v && .venv/bin/pytest tests/host && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t4_exp_c1_c2 [Human-Gated] Flash Phase 7 firmware and execute C1 onboard checks (3 cold boots, CLOCK=0x0322, >=1e5 DRDY rate, EEPROM reload, dark CH0 0-32 kHz FFT) and full C2 (0.1/0.3/0.5 V step peaks, 0-300 ms T_wait scan, CH1 margin, 0.15/0.08 V 240-pair repeatability)
  owned: docs/experiments/data/c1_c2/, docs/experiments/reports/exp-007-c1-c2-hw-and-recovery.md, docs/experiments/README.md
  depends: t2_fw_thesis_affine, t3_host_exp_suite
  verify: test -f docs/experiments/reports/exp-007-c1-c2-hw-and-recovery.md && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t5_exp_c3 [Human-Gated] Execute C3 full-cycle bidirectional calibration and generalization (3 sessions x 181-point fwd/rev x 3 blocks, 5-fold CV, 10 ref restarts, 512-code INL check, cross-session frozen retest P95 <= 1 deg)
  owned: docs/experiments/data/c3/, docs/experiments/reports/exp-008-c3-bidirectional-cal.md, docs/experiments/README.md
  depends: t4_exp_c1_c2
  verify: test -f docs/experiments/reports/exp-008-c3-bidirectional-cal.md && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t6_exp_c4 [Human-Gated] Execute C4 gain sweep (4 pts x 5 gains) and 16-target plus dense peak/null/quad 5-method head-to-head closed-loop comparison (B1-B4 vs Affine, +-1 rad seeds, 3 sessions x >=60 steps, In-Situ DC PSI + onboard firmware spot check)
  owned: docs/experiments/data/c4/, docs/experiments/reports/exp-009-c4-16target-comparison.md, docs/experiments/README.md
  depends: t5_exp_c3
  verify: test -f docs/experiments/reports/exp-009-c4-16target-comparison.md && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t7_exp_c6 [Human-Gated] Execute C6 20/25 MHz RF loading (DG922 Pro Off/0/3/6/9/13 dBm + return Off x 8 targets x 3 sessions comparing B2/B3/B4/Frozen/Recal/S3+ via In-Situ DC PSI) and J0 zero-crossing freeze protection verification
  owned: docs/experiments/data/c6/, docs/experiments/reports/exp-010-c6-rf-robustness.md, docs/experiments/README.md
  depends: t6_exp_c4
  verify: test -f docs/experiments/reports/exp-010-c6-rf-robustness.md && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t8_exp_c5 [Human-Gated] Execute C5 onboard autonomous 3x3 h closed-loop stability at 90 deg and 45 deg (continuous telemetry + periodic 3 s In-Situ DC PSI + RF zero-bias spot check at 90 deg), 5-target x 10-trial step settling (1 deg / 20 mrad), and 5-trial disturbance-triggered recalibration recovery
  owned: docs/experiments/data/c5/, docs/experiments/reports/exp-011-c5-stability-and-recovery.md, docs/experiments/README.md
  depends: t7_exp_c6
  verify: test -f docs/experiments/reports/exp-011-c5-stability-and-recovery.md && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync

- [x] t9_report_and_handoff Consolidate C1-C6 archive and compile thesis table handoff report mapping measured results and explicit bench exclusions to MasterThesisTest ch03.tex Table 3.1 and ch05.tex C3/Tables 5.5-5.7
  owned: docs/experiments/reports/exp-012-thesis-c1-c6-summary.md, docs/experiments/data/, docs/experiments/README.md, docs/architecture.md
  depends: t4_exp_c1_c2, t5_exp_c3, t6_exp_c4, t7_exp_c6, t8_exp_c5
  verify: test -f docs/experiments/reports/exp-012-thesis-c1-c6-summary.md && python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync
