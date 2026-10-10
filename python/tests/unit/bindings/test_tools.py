"""Check the reusable migration tools without modifying the bindings."""

import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[4] / "scripts/nanobind"


@pytest.mark.parametrize(
    ("tool", "argument"),
    [
        ("codemod.py", "--self-check"),
        ("api_surface.py", "--self-check"),
        ("check_guards.py", "--self-test"),
    ],
)
def test_migration_tool_self_checks(tool, argument):
    result = subprocess.run(
        [sys.executable, str(TOOLS / tool), argument],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_codemod_preserves_an_existing_destination(tmp_path):
    sentinel = tmp_path / "keep.cpp"
    sentinel.write_text("existing binding\n")
    result = subprocess.run(
        [sys.executable, str(TOOLS / "codemod.py"), "--output", str(tmp_path)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 2
    assert "destination must not exist" in result.stderr
    assert sentinel.read_text() == "existing binding\n"
