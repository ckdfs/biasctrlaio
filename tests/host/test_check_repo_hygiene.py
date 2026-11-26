import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("hygiene", ROOT / "scripts/check_repo_hygiene.py")
hygiene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hygiene)


def init_git_repo(repo_dir: Path):
    def git(*args, env=None):
        return subprocess.run(
            ["git", "-c", "user.email=test@example.org", "-c", "user.name=Test",
             "-c", "commit.gpgsign=false", *args],
            cwd=repo_dir, check=True, env=env, capture_output=True, text=True,
        )

    git("init", "-q")
    git("commit", "--allow-empty", "-qm", "base")
    git("tag", "base")
    return git


@pytest.mark.parametrize(("before", "after", "reason"), [
    ("FPC2 ribbon", "very-high-speed ribbon", "very-high-speed"),
    ("FPC2 ribbon", "MCO1 clock", "mco1"),
    ("&spi2_sck_pb13 {", "&different_pin {", "spi2_sck_pb13"),
    ('slew-rate = "low-speed";', '/* slew-rate = "low-speed"; */', "i2c1_scl_pb8"),
    ("<&gpioa 1 (GPIO_ACTIVE_HIGH | STM32_GPIO_LOW_SPEED)>", "<&gpioa 1 GPIO_ACTIVE_HIGH>", "pilot-sw"),
    ("spare-pa8-gpios =", "other-gpios =", "spare-pa8"),
])
def test_board_guards(tmp_path, before, after, reason):
    board = tmp_path / hygiene.BOARD
    board.parent.mkdir(parents=True)
    source = (ROOT / hygiene.BOARD).read_text()
    assert before in source
    board.write_text(source.replace(before, after, 1))
    with pytest.raises(ValueError, match=reason):
        hygiene.check_fpc2_and_pa8(tmp_path)


def test_tmp_path_boundaries(tmp_path):
    script = tmp_path / "scripts" / "demo.sh"
    script.parent.mkdir()
    script.write_text('dir=".agent-bridge/tmp"\nother=/tmpfile\n')
    hygiene.check_no_tmp(tmp_path)
    script.write_text("cd '/tmp'\n")
    with pytest.raises(ValueError, match=r"demo.sh:1"):
        hygiene.check_no_tmp(tmp_path)


@pytest.fixture
def docs(tmp_path):
    (tmp_path / "README.md").touch()
    (tmp_path / "AGENTS.md").touch()
    (tmp_path / "docs").mkdir()
    (tmp_path / "openspec").mkdir()
    return tmp_path


@pytest.mark.parametrize("target", ["/absolute", "C:/example", "../build/file",
                                    "../.agent-bridge/log", "missing.md", "../../outside"])
def test_invalid_link(docs, target):
    for name in ("build/file", ".agent-bridge/log"):
        path = docs / name
        path.parent.mkdir()
        path.touch()
    (docs / "docs/page.md").write_text(f"[link]({target})")
    with pytest.raises(ValueError, match="link"):
        hygiene.check_markdown_links(docs)


def test_relative_external_and_fenced_links(docs):
    (docs / "docs/page.md").write_text(
        "[root](../README.md#top) [web](https://example.com) [section](#top)\n"
        "~~~~python\n~~~\n[x](absent)\n~~~~\n")
    hygiene.check_markdown_links(docs)


def test_local_uri_in_openspec_fence(docs):
    (docs / "openspec/spec.md").write_text("```text\nfile:///example\n```\n")
    with pytest.raises(ValueError, match="local file URI"):
        hygiene.check_markdown_links(docs)


def test_no_mermaid_guard(docs):
    hygiene.check_no_mermaid(docs)
    (docs / "docs/diagram.md").write_text("```mermaid\ngraph LR;\n```\n")
    with pytest.raises(ValueError, match="forbidden mermaid fence"):
        hygiene.check_no_mermaid(docs)


def test_figures_and_index_guards(docs):
    src = docs / "docs/figures/src"
    src.mkdir(parents=True)
    tex = src / "demo.tex"
    tex.write_text("\\documentclass{standalone}")
    with pytest.raises(ValueError, match="missing rendered SVG"):
        hygiene.check_figures_and_index(docs)

    svg = docs / "docs/figures/demo.svg"
    svg.write_text("<svg></svg>")
    with pytest.raises(ValueError, match="white contrast background"):
        hygiene.check_figures_and_index(docs)

    svg.write_text('<svg><rect data-bg="white" width="100%" height="100%"/></svg>')
    (docs / "docs/design").mkdir()
    (docs / "docs/design/optical.md").touch()
    (docs / "docs/README.md").write_text("# index\n")
    with pytest.raises(ValueError, match="unindexed design document"):
        hygiene.check_figures_and_index(docs)

    (docs / "docs/README.md").write_text("[opt](design/optical.md)\n")
    (docs / "docs/experiments/reports").mkdir(parents=True)
    (docs / "docs/experiments/reports/exp004.md").touch()
    (docs / "docs/experiments/README.md").write_text("# ledger\n")
    with pytest.raises(ValueError, match="unindexed experiment record"):
        hygiene.check_figures_and_index(docs)

    (docs / "docs/experiments/README.md").write_text("[exp](reports/exp004.md)\n")
    hygiene.check_figures_and_index(docs)


def test_doc_sync_uses_committed_diff_and_appends_summary(docs):
    git = init_git_repo(docs)
    (docs / "boards").mkdir()
    (docs / "boards/example.dts").touch()
    git("add", "boards")
    git("commit", "-qm", "board change")
    summary = docs / "summary.md"
    summary.write_text("Prior results\n")
    assert hygiene.report_doc_sync(docs, "base", summary, False) == 0
    assert summary.read_text().startswith("Prior results\n")
    assert "docs/design/electrical.md" in summary.read_text()
    assert hygiene.report_doc_sync(docs, "base", None, True) == 1
    for name in ("architecture.md", "design/electrical.md"):
        path = docs / "docs" / name
        path.parent.mkdir(exist_ok=True)
        path.touch()
    git("add", "docs")
    git("commit", "-qm", "update documentation")
    assert hygiene.report_doc_sync(docs, "base", None, True) == 0


def test_staged_and_working_tree_doc_sync_modes(docs):
    git = init_git_repo(docs)
    (docs / "lib/control").mkdir(parents=True)
    (docs / "lib/control/ctrl_mod_dpmzm.c").write_text("int f(void) { return 0; }\n")

    assert hygiene.report_doc_sync(docs, None, None, True, mode="working-tree") == 1
    assert hygiene.report_doc_sync(docs, None, None, True, mode="staged") == 0

    git("add", "lib/control/ctrl_mod_dpmzm.c")
    assert hygiene.report_doc_sync(docs, None, None, True, mode="staged") == 1

    (docs / "docs/design").mkdir(parents=True)
    for rel in ("docs/architecture.md", "docs/design/algorithm.md", "docs/design/optical.md"):
        (docs / rel).touch()
    git("add", "docs")
    assert hygiene.report_doc_sync(docs, None, None, True, mode="staged") == 0
    assert hygiene.report_doc_sync(docs, None, None, True, mode="working-tree") == 0


def test_non_exemptable_tex_and_experiment_sync(docs, monkeypatch):
    git = init_git_repo(docs)
    monkeypatch.setenv("BIAS_DOC_SYNC_EXEMPT", "pure internal refactor")
    (docs / "docs/figures/src").mkdir(parents=True)
    (docs / "docs/figures/src/loop.tex").write_text("% fig")
    (docs / "docs/experiments/reports").mkdir(parents=True)
    (docs / "docs/experiments/reports/exp004.md").write_text("# exp")
    (docs / "lib/dsp").mkdir(parents=True)
    (docs / "lib/dsp/helper.c").write_text("int x;")
    git("add", ".")
    git("commit", "-qm", "refactor [docs-exempt: pure internal refactor]")

    # Even with env var + trailer, missing .svg and experiments/README.md still fail strict check.
    assert hygiene.report_doc_sync(docs, "base", None, True, mode="diff-base") == 1

    (docs / "docs/figures/loop.svg").write_text('<svg><rect data-bg="white"/></svg>')
    (docs / "docs/experiments/README.md").write_text("[exp](reports/exp004.md)\n")
    git("add", "docs")
    git("commit", "-qm", "sync svg and ledger")

    # Now only mapped code-to-doc entries remain, which human exemption waives.
    assert hygiene.report_doc_sync(docs, "base", None, True, mode="diff-base") == 0


def test_agent_doc_gate_script(tmp_path):
    gate = ROOT / "scripts/agent_doc_gate.py"

    def run_gate(payload):
        proc = subprocess.run(
            [sys.executable, str(gate)],
            input=json.dumps(payload),
            text=True,
            capture_output=True,
            check=True,
        )
        return json.loads(proc.stdout)

    assert run_gate({"terminationReason": "USER_CANCELED", "continuationActive": False}) == {"decision": "allow"}
    assert run_gate({"terminationReason": "NO_TOOL_CALL", "continuationActive": True}) == {"decision": "allow"}

    # Clean/synced workspace check should allow; dirty repo unit check via copied check script:
    sandbox = tmp_path / "repo"
    sandbox.mkdir()
    git = init_git_repo(sandbox)
    (sandbox / hygiene.BOARD).parent.mkdir(parents=True)
    (sandbox / hygiene.BOARD).write_text((ROOT / hygiene.BOARD).read_text())
    (sandbox / "README.md").touch()
    (sandbox / "AGENTS.md").touch()
    (sandbox / "docs").mkdir()
    (sandbox / "openspec").mkdir()
    (sandbox / "scripts").mkdir()
    (sandbox / "scripts/check_repo_hygiene.py").write_text((ROOT / "scripts/check_repo_hygiene.py").read_text())
    (sandbox / "scripts/agent_doc_gate.py").write_text(gate.read_text())
    git("add", ".")
    git("commit", "-qm", "seed repo")

    sandbox_gate = sandbox / "scripts/agent_doc_gate.py"
    clean = subprocess.run(
        [sys.executable, str(sandbox_gate)],
        input=json.dumps({"terminationReason": "NO_TOOL_CALL", "continuationActive": False}),
        text=True, capture_output=True, check=True,
    )
    assert json.loads(clean.stdout) == {"decision": "allow"}

    (sandbox / "lib/dsp").mkdir(parents=True)
    (sandbox / "lib/dsp/test_filter.c").write_text("int y;")
    dirty = subprocess.run(
        [sys.executable, str(sandbox_gate)],
        input=json.dumps({"terminationReason": "NO_TOOL_CALL", "continuationActive": False}),
        text=True, capture_output=True, check=True,
    )
    parsed = json.loads(dirty.stdout)
    assert parsed["decision"] == "continue"
    assert "docs/design/algorithm.md" in parsed["reason"]
