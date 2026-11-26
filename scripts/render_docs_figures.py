#!/usr/bin/env python3
"""Render docs/figures/src/*.tex standalone TikZ/CircuitikZ figures to SVG."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "docs" / "figures" / "src"
OUT_DIR = ROOT / "docs" / "figures"
BUILD_DIR = ROOT / ".agent-bridge" / "tmp" / "tex_build"


def resolve_tool(name: str, fallback: str | None = None) -> str:
    found = shutil.which(name)
    if found:
        return found
    if fallback and Path(fallback).is_file():
        return fallback
    raise FileNotFoundError(f"Required executable not found: {name}")


def inject_white_background(svg_path: Path) -> None:
    text = svg_path.read_text(encoding="utf-8")
    if 'data-bg="white"' in text:
        return
    match = re.search(r"<svg\b[^>]*>", text)
    if not match:
        raise ValueError(f"Malformed SVG without <svg> root tag: {svg_path}")
    insert_at = match.end()
    bg_rect = '\n<rect data-bg="white" width="100%" height="100%" fill="#ffffff"/>'
    svg_path.write_text(text[:insert_at] + bg_rect + text[insert_at:], encoding="utf-8")


def render_one(
    tex_path: Path,
    xelatex: str,
    dvisvgm: str,
    pdftoppm: str | None,
    preview_dir: Path | None,
) -> Path:
    stem = tex_path.stem
    work_dir = BUILD_DIR / stem
    work_dir.mkdir(parents=True, exist_ok=True)

    # Use pgfsys-dvisvgm.def with -no-pdf so TikZ/CircuitikZ emit native SVG specials in XDV.
    tex_cmd = rf"\def\pgfsysdriver{{pgfsys-dvisvgm.def}}\input{{{tex_path.name}}}"
    subprocess.run(
        [
            xelatex,
            "-no-pdf",
            "-interaction=nonstopmode",
            "-halt-on-error",
            f"-jobname={stem}",
            f"-output-directory={work_dir}",
            tex_cmd,
        ],
        cwd=tex_path.parent,
        check=True,
    )

    xdv_path = work_dir / f"{stem}.xdv"
    if not xdv_path.is_file():
        raise FileNotFoundError(f"Expected compiled XDV missing: {xdv_path}")

    svg_path = OUT_DIR / f"{stem}.svg"
    subprocess.run(
        [
            dvisvgm,
            "--no-fonts",
            "--exact-bbox",
            f"--output={svg_path}",
            str(xdv_path),
        ],
        check=True,
    )
    if not svg_path.is_file() or svg_path.stat().st_size == 0:
        raise RuntimeError(f"Failed to generate non-empty SVG: {svg_path}")
    inject_white_background(svg_path)

    if preview_dir is not None:
        if pdftoppm is None:
            raise FileNotFoundError("pdftoppm is required when --preview-dir is set")
        pdf_work_dir = BUILD_DIR / f"{stem}_pdf"
        pdf_work_dir.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                xelatex,
                "-interaction=nonstopmode",
                "-halt-on-error",
                f"-output-directory={pdf_work_dir}",
                tex_path.name,
            ],
            cwd=tex_path.parent,
            check=True,
        )
        pdf_path = pdf_work_dir / f"{stem}.pdf"
        if not pdf_path.is_file():
            raise FileNotFoundError(f"Expected compiled PDF missing: {pdf_path}")
        preview_dir.mkdir(parents=True, exist_ok=True)
        prefix = preview_dir / stem
        subprocess.run(
            [pdftoppm, "-png", "-r", "180", "-singlefile", str(pdf_path), str(prefix)],
            check=True,
        )
    return svg_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", help="Render only the matching figure stem (without .tex)")
    parser.add_argument("--preview-dir", type=Path, help="Optional directory to emit PNG previews")
    args = parser.parse_args(argv)

    xelatex = resolve_tool("xelatex", "/Library/TeX/texbin/xelatex")
    dvisvgm = resolve_tool("dvisvgm", "/Library/TeX/texbin/dvisvgm")
    pdftoppm = resolve_tool("pdftoppm", "/opt/homebrew/bin/pdftoppm") if args.preview_dir else None

    if not SRC_DIR.is_dir():
        raise FileNotFoundError(f"Source directory missing: {SRC_DIR}")

    sources = sorted(SRC_DIR.glob("*.tex"))
    if args.only:
        sources = [p for p in sources if p.stem == args.only]
        if not sources:
            raise FileNotFoundError(f"No .tex source matching stem {args.only!r} in {SRC_DIR}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for tex in sources:
        svg = render_one(tex, xelatex, dvisvgm, pdftoppm, args.preview_dir)
        print(f"Rendered {svg.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
