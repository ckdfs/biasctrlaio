#!/usr/bin/env python3
"""Check board constraints, temporary paths and repository documentation."""

import argparse
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
    for name in ("app", "boards", "drivers", "include", "lib", "scripts", "tests", ".github", "pyproject.toml"):
        base = root / name
        for path in ([base] if base.is_file() else base.rglob("*")):
            if "__pycache__" in path.parts or path.relative_to(root) in skip or not path.is_file():
                continue
            for number, line in enumerate(path.read_bytes().splitlines(), 1):
                if TMP.search(line):
                    raise ValueError(f"{path}:{number}: hardcoded /tmp path")


def check_markdown_links(root):
    root = root.resolve()
    for path in (root / "README.md", root / "AGENTS.md", *(root / "docs").rglob("*.md"),
                 *(root / "openspec").rglob("*.md")):
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


def report_doc_sync(root, base, summary_file, strict):
    output = subprocess.check_output(
        ["git", "diff", "--name-only", "-z", "--end-of-options", base + "...HEAD", "--"],
        cwd=root, text=True)
    changed = set(output.split("\0"))
    docs = set()
    for path in changed:
        if path.startswith(("app/", "boards/", "drivers/", "include/", "lib/")):
            docs.add("docs/architecture.md")
        if path.startswith(("boards/", "drivers/")):
            docs.add("docs/design/electrical.md")
        if path.startswith(("lib/control/", "lib/dsp/")):
            docs.add("docs/design/algorithm.md")
        if path.startswith(("app/src/cli", "app/src/ui", "scripts/bootstrap")):
            docs.add("docs/user-guide.md")
    missing = sorted(docs - changed)
    message = "### Documentation sync\n\n" + (
        "Review " + ", ".join(f"`{path}`" for path in missing) if missing else "No outstanding suggested checks.")
    print(message)
    if summary_file:
        with summary_file.open("a") as stream:
            print(message, file=stream)
    return int(strict and bool(missing))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--diff-base")
    parser.add_argument("--summary-file", type=Path)
    parser.add_argument("--strict-doc-sync", action="store_true")
    args = parser.parse_args()
    try:
        check_fpc2_and_pa8(args.root)
        check_no_tmp(args.root)
        check_markdown_links(args.root)
        if args.diff_base:
            return report_doc_sync(args.root, args.diff_base, args.summary_file, args.strict_doc_sync)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"Hygiene check failed: {exc}", file=sys.stderr)
        return 1
    print("Repository hygiene checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
