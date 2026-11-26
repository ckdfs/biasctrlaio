#!/usr/bin/env python3
"""Check board constraints, temporary paths, figures, links, and documentation sync."""

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote

BOARD = Path("boards/xidian/biasctrl_h503/biasctrl_h503.dts")
PINS = ("i2c1_scl_pb8", "i2c1_sda_pb7", "spi1_sck_pa5", "spi1_mosi_pa7",
        "spi2_sck_pb13", "spi2_miso_pb14", "spi2_mosi_pb15")
GPIOS = ("pilot-sw", "spare-pa8", "dac-clr", "dac-sync", "dac-ldac",
         "adc-cs", "adc-sync", "adc-drdy")
TMP = re.compile(rb"""(?<![A-Za-z0-9_.-])/tmp(?:/|['"\s]|$)""")
LINK = re.compile(r"""\]\(\s*(?:<([^>\n]+)>|((?:[^()\s]|\([^()\n]*\))+))(?:\s+["'][^\n]*?["'])?\s*\)""")
MERMAID = re.compile(r"^\s*(?:`{3,}|~{3,})\s*mermaid\b", re.I)
EXEMPT_TRAILER = re.compile(r"\[docs-exempt:\s*([^\]\n]+?)\s*\]", re.I)


def check_fpc2_and_pa8(root):
    source = (root / BOARD).read_text()
    for forbidden in ("very-high-speed", "mco1"):
        if forbidden in source.lower():
            raise ValueError(f"{BOARD}: forbidden {forbidden}")
    source = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
    for pin in PINS:
        blocks = re.findall(rf"&{pin}\s*\{{([^{{}}]*)\}}", source)
        if not blocks or any(not re.search(r'\bslew-rate\s*=\s*"low-speed"\s*;', b)
                             for b in blocks):
            raise ValueError(f"{BOARD}: {pin} requires low-speed slew-rate")
    user = re.search(r"\bzephyr,user\s*\{([^{}]*)\}", source)
    for gpio in GPIOS:
        name = gpio + "-gpios"
        prop = re.search(rf"(?<![\w-]){name}\s*=\s*([^;]*);", user[1]) if user else None
        cells = re.findall(r"<([^<>]+)>", prop[1]) if prop else []
        if not cells or any(not re.search(r"\bSTM32_GPIO_LOW_SPEED\b", cell) for cell in cells):
            raise ValueError(f"{BOARD}: each {name} pin requires STM32_GPIO_LOW_SPEED")


def check_no_tmp(root):
    skip = {Path("scripts/check_repo_hygiene.py"), Path("tests/host/test_check_repo_hygiene.py")}
    for name in ("app", "boards", "drivers", "include", "lib", "scripts", "tests",
                 ".github", ".githooks", ".agents/hooks.json", "pyproject.toml"):
        base = root / name
        if not base.exists():
            continue
        for path in ([base] if base.is_file() else base.rglob("*")):
            if "__pycache__" in path.parts or path.relative_to(root) in skip or not path.is_file():
                continue
            for number, line in enumerate(path.read_bytes().splitlines(), 1):
                if TMP.search(line):
                    raise ValueError(f"{path}:{number}: hardcoded /tmp path")


def check_no_mermaid(root):
    targets = []
    if (root / "README.md").is_file():
        targets.append(root / "README.md")
    if (root / "docs").is_dir():
        targets.extend(sorted((root / "docs").rglob("*.md")))
    for path in targets:
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if MERMAID.match(line):
                raise ValueError(f"{path}:{number}: forbidden mermaid fence (use LaTeX TikZ/CircuitikZ SVG)")


def check_figures_and_index(root):
    fig_src = root / "docs" / "figures" / "src"
    if fig_src.is_dir():
        for tex in sorted(fig_src.glob("*.tex")):
            svg = root / "docs" / "figures" / f"{tex.stem}.svg"
            if not svg.is_file() or svg.stat().st_size == 0:
                raise ValueError(f"{tex}: missing rendered SVG {svg.relative_to(root)}")
            if 'data-bg="white"' not in svg.read_text(encoding="utf-8"):
                raise ValueError(f"{svg}: missing white contrast background rect")

    design_dir = root / "docs" / "design"
    if design_dir.is_dir():
        docs_readme = root / "docs" / "README.md"
        if not docs_readme.is_file():
            raise ValueError("docs/README.md missing while docs/design exists")
        readme_text = docs_readme.read_text(encoding="utf-8")
        for md in sorted(design_dir.glob("*.md")):
            ref = f"design/{md.name}"
            if ref not in readme_text:
                raise ValueError(f"docs/README.md: unindexed design document {ref}")

    exp_dir = root / "docs" / "experiments"
    if exp_dir.is_dir():
        exp_readme = exp_dir / "README.md"
        if not exp_readme.is_file():
            raise ValueError("docs/experiments/README.md missing while docs/experiments exists")
        exp_text = exp_readme.read_text(encoding="utf-8")
        for sub in ("bringup", "reports"):
            sub_dir = exp_dir / sub
            if not sub_dir.is_dir():
                continue
            for md in sorted(sub_dir.glob("*.md")):
                ref = f"{sub}/{md.name}"
                if ref not in exp_text:
                    raise ValueError(f"docs/experiments/README.md: unindexed experiment record {ref}")


def check_markdown_links(root):
    root = root.resolve()
    candidates = [root / "README.md", root / "AGENTS.md"]
    if (root / "docs").is_dir():
        candidates.extend(sorted((root / "docs").rglob("*.md")))
    if (root / "openspec").is_dir():
        candidates.extend(sorted((root / "openspec").rglob("*.md")))
    for path in candidates:
        if not path.is_file():
            continue
        fence = ""
        for number, line in enumerate(path.read_text().splitlines(), 1):
            if "file://" in line.lower():
                raise ValueError(f"{path}:{number}: local file URI")
            if path.relative_to(root).parts[0] == "openspec":
                continue
            marker = re.match(r"^ {0,3}(`{3,}|~{3,})(.*)$", line)
            if fence:
                if marker and marker[1].startswith(fence) and not marker[2].strip():
                    fence = ""
                continue
            if marker:
                fence = marker[1]
                continue
            for match in LINK.finditer(line):
                target = match[1] or match[2]
                if target.startswith(("https://", "http://", "mailto:", "sidecar://", "#")):
                    continue
                local = unquote(target.split("#", 1)[0]).replace("\\", "/")
                if local.startswith("/") or re.match(r"^[A-Za-z]:", local):
                    raise ValueError(f"{path}:{number}: absolute link {target}")
                resolved = (path.parent / local).resolve()
                if (not resolved.is_relative_to(root) or
                        resolved.relative_to(root).parts[:1] in (("build",), (".agent-bridge",)) or
                        not resolved.exists()):
                    raise ValueError(f"{path}:{number}: invalid repository link {target}")


def _collect_changed_paths(root, base, mode):
    if mode == "diff-base":
        if not base:
            raise ValueError("diff-base mode requires a base revision")
        raw = subprocess.check_output(
            ["git", "diff", "--name-only", "-z", "--end-of-options", f"{base}...HEAD", "--"],
            cwd=root, text=True)
        return {p for p in raw.split("\0") if p}
    if mode == "staged":
        raw = subprocess.check_output(
            ["git", "diff", "--cached", "--name-only", "-z", "--"],
            cwd=root, text=True)
        return {p for p in raw.split("\0") if p}
    if mode == "working-tree":
        tracked = subprocess.check_output(
            ["git", "diff", "--name-only", "-z", "HEAD", "--"],
            cwd=root, text=True)
        untracked = subprocess.check_output(
            ["git", "ls-files", "--others", "--exclude-standard", "-z"],
            cwd=root, text=True)
        return {p for p in (tracked + "\0" + untracked).split("\0") if p}
    raise ValueError(f"Unsupported doc-sync mode: {mode}")


def _resolve_exempt_reason(root, base, mode, explicit_reason):
    if explicit_reason is not None:
        return explicit_reason.strip()
    env_reason = os.environ.get("BIAS_DOC_SYNC_EXEMPT", "").strip()
    if env_reason:
        return env_reason
    if mode == "diff-base" and base:
        log_text = subprocess.check_output(
            ["git", "log", "--format=%B", f"{base}..HEAD"],
            cwd=root, text=True)
        match = EXEMPT_TRAILER.search(log_text)
        if match:
            return match.group(1).strip()
    return ""


def _compute_required_docs(root, changed):
    hard_required = set()
    mapped_required = set()
    for path in changed:
        if path.startswith("docs/figures/src/") and path.endswith(".tex"):
            if (root / path).exists():
                stem = Path(path).stem
                hard_required.add(f"docs/figures/{stem}.svg")
        if path.startswith(("docs/experiments/bringup/", "docs/experiments/reports/")) and path.endswith(".md"):
            hard_required.add("docs/experiments/README.md")

        if path.startswith(("app/", "boards/", "drivers/", "include/", "lib/")):
            mapped_required.add("docs/architecture.md")
        if path.startswith(("boards/", "drivers/ads131m02/", "drivers/dac8568/",
                            "drivers/analog_board/", "drivers/pilot_mux/", "drivers/eeprom_board/")):
            mapped_required.add("docs/design/electrical.md")
        if path.startswith(("lib/control/", "lib/dsp/")):
            mapped_required.add("docs/design/algorithm.md")
        if path.startswith(("lib/control/ctrl_mod_", "drivers/analog_board/")):
            mapped_required.add("docs/design/optical.md")
        if path.startswith(("app/src/cli", "app/src/diag_cli", "app/src/ui",
                            "app/src/main.c", "drivers/oled_lite/", "scripts/bootstrap")):
            mapped_required.add("docs/user-guide.md")
        if path.startswith(("scripts/bootstrap", "scripts/run_qemu_ztest.py",
                            "scripts/check_repo_hygiene.py", "scripts/render_docs_figures.py",
                            ".github/workflows/")):
            mapped_required.add("README.md")
    return hard_required, mapped_required


def report_doc_sync(root, base=None, summary_file=None, strict=False, *, mode="diff-base", exempt_reason=None):
    changed = _collect_changed_paths(root, base, mode)
    hard_required, mapped_required = _compute_required_docs(root, changed)
    resolved_exempt = _resolve_exempt_reason(root, base, mode, exempt_reason)

    missing_hard = sorted(hard_required - changed)
    missing_mapped = sorted(mapped_required - changed)

    lines = ["### Documentation sync", ""]
    if missing_hard:
        lines.append("Non-exemptable figure/index updates missing: " + ", ".join(f"`{p}`" for p in missing_hard))
    if missing_mapped:
        if resolved_exempt:
            lines.append(
                f"Code-to-doc sync exempted by human reason: {resolved_exempt} "
                "(skipped: " + ", ".join(f"`{p}`" for p in missing_mapped) + ")"
            )
        else:
            lines.append("Review " + ", ".join(f"`{p}`" for p in missing_mapped))
    if not missing_hard and not missing_mapped:
        lines.append("No outstanding suggested checks.")

    message = "\n".join(lines)
    print(message)
    if summary_file:
        with summary_file.open("a", encoding="utf-8") as stream:
            print(message, file=stream)
    return int(strict and (bool(missing_hard) or (bool(missing_mapped) and not resolved_exempt)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("."))
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument("--diff-base")
    scope.add_argument("--staged", action="store_true")
    scope.add_argument("--working-tree", action="store_true")
    parser.add_argument("--summary-file", type=Path)
    parser.add_argument("--strict-doc-sync", action="store_true")
    args = parser.parse_args()
    try:
        check_fpc2_and_pa8(args.root)
        check_no_tmp(args.root)
        check_no_mermaid(args.root)
        check_figures_and_index(args.root)
        check_markdown_links(args.root)
        if args.diff_base:
            return report_doc_sync(args.root, args.diff_base, args.summary_file, args.strict_doc_sync, mode="diff-base")
        if args.staged:
            return report_doc_sync(args.root, None, args.summary_file, args.strict_doc_sync, mode="staged")
        if args.working_tree:
            return report_doc_sync(args.root, None, args.summary_file, args.strict_doc_sync, mode="working-tree")
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"Hygiene check failed: {exc}", file=sys.stderr)
        return 1
    print("Repository hygiene checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
