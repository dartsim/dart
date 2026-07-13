"""Tests for the deterministic DART AI setup sequence."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("setup_ai", ROOT / "scripts/setup_ai.py")
assert SPEC and SPEC.loader
setup_ai = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(setup_ai)


def test_setup_commands_are_ordered_and_use_active_python():
    assert setup_ai.setup_commands() == (
        [sys.executable, "scripts/sync_ai_commands.py"],
        [sys.executable, "scripts/install_git_hooks.py"],
    )


def test_setup_runs_sync_then_hook_install(monkeypatch, capsys):
    calls = []

    def run(command, cwd):
        calls.append((command, cwd))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(setup_ai.subprocess, "run", run)

    assert setup_ai.main() == 0
    assert [command for command, _ in calls] == list(setup_ai.setup_commands())
    assert all(cwd == ROOT for _, cwd in calls)
    assert "setup complete" in capsys.readouterr().out


def test_setup_stops_after_first_failure(monkeypatch, capsys):
    calls = []

    def run(command, cwd):
        calls.append((command, cwd))
        return SimpleNamespace(returncode=7)

    monkeypatch.setattr(setup_ai.subprocess, "run", run)

    assert setup_ai.main() == 7
    assert len(calls) == 1
    assert "setup complete" not in capsys.readouterr().out


def test_setup_propagates_hook_install_failure(monkeypatch, capsys):
    returncodes = iter((0, 9))
    calls = []

    def run(command, cwd):
        calls.append((command, cwd))
        return SimpleNamespace(returncode=next(returncodes))

    monkeypatch.setattr(setup_ai.subprocess, "run", run)

    assert setup_ai.main() == 9
    assert len(calls) == 2
    assert "setup complete" not in capsys.readouterr().out
