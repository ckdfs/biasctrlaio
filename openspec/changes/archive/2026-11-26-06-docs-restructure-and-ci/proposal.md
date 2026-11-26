# Proposal: Phase 6 — Domain Documentation Restructure, PCB Errata & GitHub Actions CI

## Why
The repository currently concentrates hardware, real-time scheduling, DSP/control algorithms, and Phase 5 planning into a single `docs/design.md` file, lacks a dedicated optical-path design document and a PCB errata/rework register, stores bring-up verification reports under `docs/bringup/` without a master experiment ledger or template, contains local `file` URI absolute links in Markdown files that break on GitHub, and has no `.github/workflows/` CI/Release automation.

## What Changes
- Restructure `docs/` into clear domain documents and an experiment archive:
  - `docs/README.md` (documentation index)
  - `docs/architecture.md` (top-level split-board architecture, RT isolation, `U21` EEPROM layout, Phase 4 resource baseline, Phase 5 roadmap; supersedes monolithic `docs/design.md`)
  - `docs/design/optical.md` (single-MZM & 3-arm DPMZM optical models, power/PD budget, operating points & `null_q_sign = -1` polarity contract, optical testbench topology)
  - `docs/design/electrical.md` (split-board mechanics/stackup, 30-pin `FPC2` mapping & `low-speed` rule, power tree & thermal budget, DC/pilot/TIA/ADC signal-chain equations, dual-crystal heterodyne clocking & `15.625 µs` SPI/DMA timing)
  - `docs/design/pcb-errata.md` (`ERR-01`..`ERR-06` hardware rework & BOM/mechanical errata register with next-revision checklist)
  - `docs/design/algorithm.md` (Jacobi-Anger harmonic model, zero-leakage Goertzel, `O(1)`-RAM 3×3 Cholesky LSQ calibration, Bessel-scaled observer & anti-windup PI, Phase 5 seamless switching & fixed dual-pilot DPMZM, host optical simulator & baseline comparison)
  - `docs/experiments/` (`README.md` ledger, `template.md`, migrated `bringup/` reports, `reports/.gitkeep`, `data/.gitkeep`)
- Replace all local `file` URI links and non-repo-relative links across `docs/`, `README.md`, `AGENTS.md`, and `openspec/` with repository-relative paths, and update `AGENTS.md` and active Phase 5 tasks to reference the new domain doc layout.
- Add public-repo GitHub Actions automation and supporting scripts:
  - `scripts/check_repo_hygiene.py` (FPC2 `low-speed` & `PA8` `MCO1` guard, `/tmp` prohibition guard, Markdown link / local-`file`-URI check, optional PR doc-sync advisory)
  - `scripts/run_qemu_ztest.py` (automated Zephyr `qemu_cortex_m3` `ztest` runner with process-group termination on `PROJECT EXECUTION SUCCESSFUL`)
  - `.github/workflows/ci.yml` (`hygiene-and-docs`, `host-tests`, `zephyr-build-and-budget`, `zephyr-ztest`)
  - `.github/workflows/release.yml` (`v*` tag / manual release packaging with full test gates, `SHA256SUMS.txt`, and draft GitHub Release)
  - `tests/host/test_check_repo_hygiene.py` (pytest unit tests for `check_repo_hygiene.py`)

## Out of Scope
- Modifying firmware logic under `app/`, `boards/`, `drivers/`, `lib/`, or `include/`.
- Running physical hardware flashing or optical bench tests in cloud CI.
