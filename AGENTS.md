# biasctrlaio Agent Governance

This project builds a Zephyr RTOS v4.5.0 split-board electro-optic modulator bias controller (`STM32H503CBT6` Digital Baseboard + `20261110-MZM` Analog Daughterboard).

## Orchestration Model
- **Antigravity**: Architecture, OpenSpec planning (`openspec/`), SQLite task scheduling & blocker resolution (`.agent-bridge/bridge.db`), and independent verification.
- **Codex CLI**: Scoped implementation within assigned `owned_paths`, unit/host tests, and structured reporting via `bridge_ctl.py`.

## Mandatory Rules
1. **Never read or write `/tmp`**: Store all temporary files, test outputs, logs, and SQLite state under `.agent-bridge/` (already in `.gitignore`).
2. **Fast-fail**: Expose errors directly with explicit context. Never add broad `try/except` fallbacks or silent hardware degradation.
3. **Never auto-rollback Git**: Do not run `git reset --hard`, `git checkout -- <file>`, or `git restore`.
4. **Stay inside `owned_paths`**: When working on a dispatched task, edit only paths authorized in your prompt. Request a coordinator decision (`need_decision`) if an interface outside `owned_paths` must change.
5. **Hardware & Real-Time Safety**:
   - Legacy single-board hardware compatibility is excluded.
   - Never modify `FPC2` pins to `very-high-speed`; keep `low-speed`.
   - Never call blocking I2C/UART/sleep functions from `64 kSPS` `DRDY#` ISR paths.
   - Respect STM32H503CBT6 limits: static RAM `<= 24576 B`, Flash `<= 110592 B`.
   - Do not flash or drive physical hardware unless executing a human-approved verification task.
6. **Documentation Synchronization (Mandatory)**:
   - Every change to hardware configuration/wiring, architecture, interfaces, DSP/control algorithms, firmware features, CLI/UI behavior, or resource budgets MUST update the relevant pages in [the documentation index](docs/README.md) covering [architecture](docs/architecture.md), [optics](docs/design/optical.md), [electronics](docs/design/electrical.md), [mechanics & enclosure](docs/design/mechanical.md), [control algorithms](docs/design/algorithm.md), or [PCB errata](docs/design/pcb-errata.md) (re-rendering `docs/figures/src/*.tex` via `python3 scripts/render_docs_figures.py` if any vector diagram source changes), and record the change in the architecture changelog, `docs/user-guide.md` (when wiring, build/flash workflow, CLI/UI usage, or troubleshooting changes), and `README.md` (when top-level status, doc links, or commands change) in the same change/commit.
   - `codex_board_review` MUST verify that required documentation updates are present before approving any task.
