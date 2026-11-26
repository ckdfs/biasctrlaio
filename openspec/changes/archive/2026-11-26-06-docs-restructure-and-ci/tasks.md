# Tasks: Phase 6 — Domain Documentation Restructure, PCB Errata & GitHub Actions CI

- [x] p6_docs_restructure Restructure docs into architecture, optical/electrical/algorithm/pcb-errata design docs, and experiments archive; fix all local file-URI links; update README.md, AGENTS.md, user-guide.md, and Phase 5 task doc paths
  owned: docs/, README.md, AGENTS.md, openspec/changes/05-tdm-multichannel-and-ui/tasks.md, openspec/changes/05-tdm-multichannel-and-ui/proposal.md
  depends:
  verify: test -f docs/README.md && test -f docs/architecture.md && test -f docs/design/optical.md && test -f docs/design/electrical.md && test -f docs/design/algorithm.md && test -f docs/design/pcb-errata.md && test -f docs/experiments/README.md && test -f docs/experiments/template.md && test -f docs/experiments/bringup/phase4-closed-loop-verify.md && ! test -f docs/design.md

- [x] p6_github_actions Add GitHub Actions CI/Release workflows, scripts/check_repo_hygiene.py, scripts/run_qemu_ztest.py, and host unit tests
  owned: .github/, scripts/check_repo_hygiene.py, scripts/run_qemu_ztest.py, tests/host/test_check_repo_hygiene.py
  depends: p6_docs_restructure
  verify: python3 scripts/check_repo_hygiene.py && .venv/bin/pytest tests/host/test_check_mem_budget.py tests/host/test_check_repo_hygiene.py
