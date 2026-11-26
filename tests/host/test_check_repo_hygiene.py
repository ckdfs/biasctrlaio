import importlib.util
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("hygiene", ROOT / "scripts/check_repo_hygiene.py")
hygiene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hygiene)


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


def test_doc_sync_uses_committed_diff_and_appends_summary(docs):
    def git(*args):
        subprocess.run(["git", "-c", "user.email=test@example.org", "-c", "user.name=Test",
                        "-c", "commit.gpgsign=false", *args], cwd=docs, check=True)

    git("init", "-q")
    git("commit", "--allow-empty", "-qm", "base")
    git("tag", "base")
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
