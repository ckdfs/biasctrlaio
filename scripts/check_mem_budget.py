#!/usr/bin/env python3
"""Verify Zephyr ELF RAM and Flash usage against STM32H503CB hard budgets."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_MAX_RAM = 24576    # 24 KiB out of 32 KiB SRAM
DEFAULT_MAX_FLASH = 110592  # 108 KiB out of 128 KiB Flash


def resolve_size_tool(explicit_tool: str | None) -> str:
    """Resolve the size binary without silent fallback to an incompatible tool."""
    if explicit_tool:
        found = shutil.which(explicit_tool) or (explicit_tool if os.path.isfile(explicit_tool) else None)
        if not found:
            raise FileNotFoundError(f"Requested size tool not found: {explicit_tool}")
        return found

    sdk_dir = os.environ.get("ZEPHYR_SDK_INSTALL_DIR")
    if sdk_dir:
        for rel in (
            Path("gnu") / "arm-zephyr-eabi" / "bin" / "arm-zephyr-eabi-size.exe",
            Path("gnu") / "arm-zephyr-eabi" / "bin" / "arm-zephyr-eabi-size",
            Path("arm-zephyr-eabi") / "bin" / "arm-zephyr-eabi-size.exe",
            Path("arm-zephyr-eabi") / "bin" / "arm-zephyr-eabi-size",
        ):
            sdk_size = Path(sdk_dir) / rel
            if sdk_size.is_file():
                return str(sdk_size)

    for candidate in ("arm-zephyr-eabi-size", "arm-none-eabi-size"):
        found = shutil.which(candidate)
        if found:
            return found

    raise FileNotFoundError(
        "No ARM size tool found (checked ZEPHYR_SDK_INSTALL_DIR, arm-zephyr-eabi-size, arm-none-eabi-size)"
    )


def parse_berkeley_size(output: str) -> tuple[int, int, int]:
    """Parse standard Berkeley `size` output into (text, data, bss) bytes."""
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    if len(lines) < 2:
        raise ValueError(f"Unexpected size output format:\n{output}")
    header = lines[0].split()
    if header[:3] != ["text", "data", "bss"]:
        raise ValueError(f"Unexpected size header: {lines[0]!r}")
    parts = lines[1].split()
    if len(parts) < 3:
        raise ValueError(f"Unexpected size data row: {lines[1]!r}")
    return int(parts[0]), int(parts[1]), int(parts[2])


def check_budget(elf_path: Path, max_ram: int, max_flash: int, size_tool: str | None = None) -> int:
    if not elf_path.is_file():
        raise FileNotFoundError(f"ELF file does not exist: {elf_path}")
    if max_ram <= 0 or max_flash <= 0:
        raise ValueError(f"Budgets must be positive integers: max_ram={max_ram}, max_flash={max_flash}")

    tool = resolve_size_tool(size_tool)
    proc = subprocess.run([tool, str(elf_path)], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"{tool} failed with code {proc.returncode}: {proc.stderr.strip()}")

    text_b, data_b, bss_b = parse_berkeley_size(proc.stdout)
    flash_b = text_b + data_b
    ram_b = data_b + bss_b

    ram_pct = (ram_b / max_ram) * 100.0
    flash_pct = (flash_b / max_flash) * 100.0

    print(f"ELF:   {elf_path}")
    print(f"Tool:  {tool}")
    print(f"text={text_b} B, data={data_b} B, bss={bss_b} B")
    print(f"Flash: {flash_b} / {max_flash} B ({flash_pct:.1f}%)")
    print(f"RAM:   {ram_b} / {max_ram} B ({ram_pct:.1f}%)")

    exceeded = []
    if flash_b > max_flash:
        exceeded.append(f"Flash {flash_b} B > budget {max_flash} B (+{flash_b - max_flash} B)")
    if ram_b > max_ram:
        exceeded.append(f"RAM {ram_b} B > budget {max_ram} B (+{ram_b - max_ram} B)")

    if exceeded:
        print("FAIL: " + "; ".join(exceeded), file=sys.stderr)
        return 1

    print("PASS: Within RAM and Flash budgets.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check ELF RAM/Flash against hard limits.")
    parser.add_argument("--elf", required=True, type=Path, help="Path to compiled zephyr.elf")
    parser.add_argument("--max-ram", type=int, default=DEFAULT_MAX_RAM, help="Max static RAM (data+bss) bytes")
    parser.add_argument("--max-flash", type=int, default=DEFAULT_MAX_FLASH, help="Max Flash (text+data) bytes")
    parser.add_argument("--size-tool", default=None, help="Explicit size executable path or name")
    args = parser.parse_args(argv)

    try:
        return check_budget(args.elf, args.max_ram, args.max_flash, args.size_tool)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
