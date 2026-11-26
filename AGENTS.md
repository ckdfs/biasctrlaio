# biasctrlaio Agent Governance

This project builds a Zephyr RTOS v4.5.0 opto-electronic split-board electro-optic modulator bias controller (`10%` monitor optical path + `20261110-MZM` Analog Daughterboard + `STM32H503CBT6` Digital Baseboard).

## Orchestration Model
- **Antigravity**: Architecture, OpenSpec planning (`openspec/`), SQLite task scheduling & blocker resolution (`.agent-bridge/bridge.db`), and independent verification.
- **Codex CLI**: Scoped implementation within assigned `owned_paths`, unit/host tests, documentation synchronization within task scope, and structured reporting via `bridge_ctl.py`.

## Mandatory Rules
1. **Never read or write `/tmp`**: Store all temporary files, test outputs, logs, and SQLite state under `.agent-bridge/` (already in `.gitignore`).
2. **Fast-fail**: Expose errors directly with explicit context. Never add broad `try/except` fallbacks or silent hardware degradation.
3. **Never auto-rollback Git**: Do not run `git reset --hard`, `git checkout -- <file>`, or `git restore`.
4. **Stay inside `owned_paths`**: When working on a dispatched task, edit only paths authorized in your prompt. Request a coordinator decision (`need_decision`) if an interface or doc outside `owned_paths` must change.
5. **Hardware & Real-Time Safety**:
   - Legacy single-board hardware compatibility is excluded.
   - Never modify `FPC2` pins to `very-high-speed`; keep `low-speed`.
   - Never call blocking I2C/UART/sleep functions from `64 kSPS` `DRDY#` ISR paths.
   - Respect STM32H503CBT6 limits: static RAM `<= 24576 B`, Flash `<= 110592 B`.
   - Do not flash or drive physical hardware unless executing a human-approved verification task.

## Standardized Agent Documentation-Sync Workflow

Every change MUST execute the following 5 stages so documentation and figures never lag behind code or hardware:

1. **Stage 1 — Plan with OpenSpec & `owned:` boundaries**:
   - Evaluate opto-electronic cross-domain impact (`optical`, `electrical`, `mechanical`, `algorithm`, `pcb-errata`, `experiments`) in `openspec/changes/<id>/design.md`.
   - Include every required doc path from the impact matrix below in each task's `owned:` list and include `python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync` in its `verify:` command.
2. **Stage 2 — Implement within `owned_paths`**:
   - Modify code/tests/figures/specs strictly inside task `owned_paths`.
3. **Stage 3 — Synchronize docs & vector figures in the same change**:
   - Update all triggered documents in [the documentation index](docs/README.md) according to the Code-to-Doc Impact Matrix below, record notable architectural/hardware/algorithm updates in [the architecture changelog](docs/architecture.md), and run `python3 scripts/render_docs_figures.py` if any `docs/figures/src/*.tex` changes.
4. **Stage 4 — Run strict local pre-review gate**:
   - Run `python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync` before completing a turn or reporting a task done.
   - Antigravity enforces this automatically via the `.agents/hooks.json` `Stop` hook (`scripts/agent_doc_gate.py`).
5. **Stage 5 — Review, Git Pre-Commit & CI Gate**:
   - `codex_board_review` MUST verify that required documentation updates and changelog entries are present before calling `approve`.
   - Local commits run `.githooks/pre-commit` (`--staged --strict-doc-sync`, enabled by `scripts/bootstrap.{sh,ps1}` via `git config core.hooksPath .githooks`).
   - GitHub Actions (`hygiene-and-docs` in `.github/workflows/ci.yml`) runs `--diff-base <rev> --strict-doc-sync` on both `push` and `pull_request`.

### Code-to-Doc Impact Matrix

`scripts/check_repo_hygiene.py` checks these mappings mechanically (plus non-exemptable `.tex -> .svg`, index coverage, zero Mermaid fences, and valid relative Markdown links):

| Changed Paths | Mandatory Docs (Same Change / Commit) | Exemptable by Human? |
| --- | --- | --- |
| `docs/figures/src/<stem>.tex` | `docs/figures/<stem>.svg` (via `python3 scripts/render_docs_figures.py`) | Never |
| `docs/experiments/bringup/*.md`, `docs/experiments/reports/*.md` | `docs/experiments/README.md` | Never |
| `app/**`, `boards/**`, `drivers/**`, `include/**`, `lib/**` | `docs/architecture.md` (incl. changelog) | Human-only |
| `boards/**`, `drivers/ads131m02/**`, `drivers/dac8568/**`, `drivers/analog_board/**`, `drivers/pilot_mux/**`, `drivers/eeprom_board/**` | `docs/design/electrical.md` | Human-only |
| `lib/control/**`, `lib/dsp/**` | `docs/design/algorithm.md` | Human-only |
| `lib/control/ctrl_mod_*`, `drivers/analog_board/**` | `docs/design/optical.md` | Human-only |
| `app/src/cli*`, `app/src/diag_cli*`, `app/src/ui*`, `app/src/main.c`, `drivers/oled_lite/**`, `scripts/bootstrap*` | `docs/user-guide.md` | Human-only |
| `scripts/bootstrap*`, `scripts/run_qemu_ztest.py`, `scripts/check_repo_hygiene.py`, `scripts/render_docs_figures.py`, `.github/workflows/**` | `README.md` | Human-only |

### Human-Only Exemption Rule
- Agents MUST NEVER set `BIAS_DOC_SYNC_EXEMPT` or add `[docs-exempt: <reason>]` commit trailers on their own.
- Only when the human user explicitly confirms a pure internal refactor has zero external/behavioral/spec impact may `[docs-exempt: <reason>]` (CI) or `BIAS_DOC_SYNC_EXEMPT="<reason>"` (local pre-commit) be used; even then, non-exemptable rules (`.tex -> .svg`, experiment index, design index, no Mermaid, valid links, `FPC2` low-speed, no `/tmp`) remain strictly enforced.
