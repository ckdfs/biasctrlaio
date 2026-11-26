#!/usr/bin/env python3
"""Antigravity Stop-hook gate enforcing repository hygiene and doc synchronization."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def main() -> int:
    raw = sys.stdin.read()
    if not raw.strip():
        raise ValueError("Expected JSON hook payload on stdin")
    payload = json.loads(raw)

    if payload.get("terminationReason") != "NO_TOOL_CALL" or payload.get("continuationActive"):
        print(json.dumps({"decision": "allow"}))
        return 0

    repo_root = Path(__file__).resolve().parents[1]
    checker = repo_root / "scripts" / "check_repo_hygiene.py"
    proc = subprocess.run(
        [sys.executable, str(checker), "--root", str(repo_root), "--working-tree", "--strict-doc-sync"],
        cwd=repo_root,
        text=True,
        capture_output=True,
    )
    if proc.returncode == 0:
        print(json.dumps({"decision": "allow"}))
        return 0

    output = "\n".join(part for part in (proc.stdout.strip(), proc.stderr.strip()) if part)
    reason = (
        "Documentation synchronization or repository hygiene check failed before turn completion.\n"
        f"{output}\n\n"
        "Update the required docs (and re-render docs/figures/src/*.tex via `python3 scripts/render_docs_figures.py` if any .tex changed), "
        "then re-run `python3 scripts/check_repo_hygiene.py --working-tree --strict-doc-sync`."
    )
    print(json.dumps({"decision": "continue", "reason": reason}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
