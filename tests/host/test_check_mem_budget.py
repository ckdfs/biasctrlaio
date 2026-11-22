import subprocess
import sys
from pathlib import Path

import pytest


CHECKER = Path(__file__).resolve().parents[2] / "scripts" / "check_mem_budget.py"


def run_checker(elf, size_tool):
    return subprocess.run(
        [sys.executable, CHECKER, "--elf", elf, "--size-tool", size_tool],
        capture_output=True,
        text=True,
    )


@pytest.mark.parametrize(
    ("text", "bss", "exit_code", "error"),
    [
        pytest.param(109568, 23552, 0, "", id="at-both-limits"),
        pytest.param(109568, 23553, 1, "RAM 24577 B > budget 24576 B", id="ram-over"),
        pytest.param(109569, 23552, 1, "Flash 110593 B > budget 110592 B", id="flash-over"),
    ],
)
def test_default_budgets(tmp_path, text, bss, exit_code, error):
    elf = tmp_path / "zephyr.elf"
    elf.touch()
    size_tool = tmp_path / "size"
    data = 1024
    total = text + data + bss
    size_tool.write_text(
        "#!/bin/sh\n"
        "echo 'text data bss dec hex filename'\n"
        f'echo "{text} {data} {bss} {total} {total:x} $1"\n'
    )
    size_tool.chmod(0o755)

    result = run_checker(elf, size_tool)

    assert result.returncode == exit_code, result.stderr
    if error:
        assert error in result.stderr
    else:
        assert "PASS: Within RAM and Flash budgets." in result.stdout


def test_missing_elf(tmp_path):
    elf = tmp_path / "missing.elf"
    result = run_checker(elf, tmp_path / "missing-size")

    assert result.returncode == 2
    assert f"ELF file does not exist: {elf}" in result.stderr


def test_missing_size_tool(tmp_path):
    elf = tmp_path / "zephyr.elf"
    elf.touch()
    size_tool = tmp_path / "missing-size"
    result = run_checker(elf, size_tool)

    assert result.returncode == 2
    assert f"Requested size tool not found: {size_tool}" in result.stderr
