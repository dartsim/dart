"""Tests for the fast staged-file hook entrypoint."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

import check_agent_hook as hook  # noqa: E402
import pretool_guard_bridge as bridge  # noqa: E402


def _repo(tmp_path: Path) -> Path:
    root = tmp_path / "repo"
    root.mkdir()
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(
        ["git", "-C", str(root), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(root), "config", "user.name", "DART Test"], check=True
    )
    subprocess.run(
        ["git", "-C", str(root), "config", "core.excludesFile", "/dev/null"],
        check=True,
    )
    return root


def _stage(root: Path, relative: str, text: str) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    subprocess.run(["git", "-C", str(root), "add", relative], check=True)


def _args(root: Path) -> argparse.Namespace:
    return argparse.Namespace(root=root, profile="main")


@pytest.mark.parametrize("input_key", ("command", "cmd"))
def test_pretool_forwards_commit_payload_to_shared_guard(
    tmp_path, monkeypatch, input_key
):
    root = _repo(tmp_path)
    guard = root / ".claude" / "hooks" / "pre-commit-guard.sh"
    guard.parent.mkdir(parents=True)
    guard.write_text("#!/bin/sh\nexit 0\n")
    bash = Path("/test/git-bash")
    calls = []
    monkeypatch.setattr(bridge, "find_git_bash", lambda: bash)
    monkeypatch.setattr(
        bridge.subprocess,
        "run",
        lambda args, **kwargs: calls.append((args, kwargs))
        or subprocess.CompletedProcess(args, 0, b"", b""),
    )
    payload = json.dumps({"tool_input": {input_key: "git commit -m test"}}).encode()

    assert bridge.forward(root, payload) == 0
    assert calls[0][0] == [
        str(bash),
        "--noprofile",
        "--norc",
        str(guard),
    ]
    assert calls[0][1]["input"] == payload
    assert calls[0][1]["cwd"] == root
    assert calls[0][1]["env"]["CLAUDE_PROJECT_DIR"] == str(root)
    assert calls[0][1]["env"]["CODEX_PROJECT_DIR"] == str(root)
    assert calls[0][1]["env"]["DART_HOOK_PYTHON"] == str(
        Path(sys.executable).resolve()
    ).replace("\\", "/")


@pytest.mark.parametrize("command", ("git status", "echo hello"))
def test_pretool_skips_non_commit_without_git_bash(tmp_path, monkeypatch, command):
    root = _repo(tmp_path)
    monkeypatch.setattr(
        bridge,
        "find_git_bash",
        lambda: pytest.fail("non-commit payload must not discover Git Bash"),
    )
    payload = json.dumps({"tool_input": {"command": command}}).encode()

    assert bridge.forward(root, payload) == 0


@pytest.mark.parametrize(
    "command",
    (
        "GIT COMMIT -m test",
        "g---i---t\nc---o---m---m---i---t",
    ),
)
def test_pretool_requires_git_bash_for_commit(tmp_path, monkeypatch, capsys, command):
    root = _repo(tmp_path)
    monkeypatch.setattr(bridge, "find_git_bash", lambda: None)
    payload = json.dumps({"tool_input": {"command": command}}).encode()

    assert bridge.forward(root, payload) == 2
    assert "Git Bash is required" in capsys.readouterr().err


def test_pretool_maps_guard_failure_to_codex_block(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    guard = root / ".claude" / "hooks" / "pre-commit-guard.sh"
    guard.parent.mkdir(parents=True)
    guard.write_text("#!/bin/sh\nexit 2\n")
    monkeypatch.setattr(bridge, "find_git_bash", lambda: Path("/test/git-bash"))
    monkeypatch.setattr(
        bridge.subprocess,
        "run",
        lambda args, **kwargs: subprocess.CompletedProcess(
            args, 2, b"guard output\n", b"guard error\n"
        ),
    )
    payload = json.dumps({"tool_input": {"command": "git commit"}}).encode()

    assert bridge.forward(root, payload) == 2
    captured = capsys.readouterr()
    assert "guard output" in captured.err
    assert "guard error" in captured.err


def test_pretool_rejects_malformed_payload(tmp_path):
    assert bridge.forward(_repo(tmp_path), b"{}") == 2


def test_non_ai_change_uses_only_fast_diff_check(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    _stage(root, "README.md", "clean\n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda *_: (_ for _ in ()).throw(AssertionError("must not run")),
    )

    started = time.monotonic()
    result = hook.main()

    assert result == 0
    assert time.monotonic() - started < 2
    assert "no AI-infra changes" in capsys.readouterr().out


def test_ai_change_runs_static_infrastructure_checks(tmp_path, monkeypatch):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/README.md", "clean\n")
    calls = []
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(hook, "detect_profile", lambda *_: "main")
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda checked_root, profile: calls.append((checked_root, profile)) or [],
    )

    assert hook.main() == 0
    assert len(calls) == 1
    assert calls[0][0] != root.resolve()
    assert calls[0][1] == "main"


def test_shared_staged_profile_alias_uses_branch_auto_detection(tmp_path, monkeypatch):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/README.md", "clean\n")
    requests = []
    monkeypatch.setattr(
        hook,
        "parse_args",
        lambda: argparse.Namespace(root=root, profile="staged"),
    )
    monkeypatch.setattr(
        hook,
        "detect_profile",
        lambda checked_root, requested: requests.append((checked_root, requested))
        or "main",
    )
    monkeypatch.setattr(hook, "run_checks", lambda *_: [])

    assert hook.main() == 0
    assert requests == [(root.resolve(), "auto")]


def test_mixed_staged_and_unstaged_ai_file_is_rejected(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/README.md", "broken staged version\n")
    (root / "docs/ai/README.md").write_text("unstaged fix\n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda *_: (_ for _ in ()).throw(AssertionError("must not run")),
    )

    assert hook.main() == 1
    output = capsys.readouterr().out
    assert "unstaged or untracked changes" in output
    assert "docs/ai/README.md" in output


def test_unstaged_related_ai_file_is_rejected_across_files(
    tmp_path, monkeypatch, capsys
):
    root = _repo(tmp_path)
    _stage(root, ".codex/config.toml", "[agents]\n")
    subprocess.run(["git", "-C", str(root), "commit", "-qm", "fixture"], check=True)
    (root / ".codex/config.toml").write_text("[agents]\nmax_threads = 4\n")
    _stage(root, "docs/ai/README.md", "staged docs\n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda *_: (_ for _ in ()).throw(AssertionError("must not run")),
    )

    assert hook.main() == 1
    output = capsys.readouterr().out
    assert ".codex/config.toml" in output


def test_untracked_related_ai_file_is_rejected(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/README.md", "staged docs\n")
    untracked = root / "dart" / "solver" / "AGENTS.md"
    untracked.parent.mkdir(parents=True)
    untracked.write_text("untracked instructions\n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda *_: (_ for _ in ()).throw(AssertionError("must not run")),
    )

    assert hook.main() == 1
    assert "dart/solver/AGENTS.md" in capsys.readouterr().out


def test_untracked_personal_skill_does_not_block_dart_infrastructure_commit(
    tmp_path, monkeypatch
):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/README.md", "staged docs\n")
    personal = root / ".agents" / "skills" / "personal-helper" / "SKILL.md"
    personal.parent.mkdir(parents=True)
    personal.write_text("---\nname: personal-helper\n---\n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(hook, "detect_profile", lambda *_: "main")
    monkeypatch.setattr(hook, "run_checks", lambda *_: [])

    assert hook.main() == 0


def test_untracked_marker_owned_adapter_blocks_staged_source_commit(
    tmp_path, monkeypatch, capsys
):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/README.md", "staged docs\n")
    generated = root / ".agents" / "skills" / "dart-stale" / "SKILL.md"
    generated.parent.mkdir(parents=True)
    generated.write_text(
        "<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->\n"
        "<!-- Sync script: scripts/sync_ai_commands.py -->\n"
    )
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda *_: (_ for _ in ()).throw(AssertionError("must not run")),
    )

    assert hook.main() == 1
    assert ".agents/skills/dart-stale/SKILL.md" in capsys.readouterr().out


def test_staged_whitespace_failure_blocks_before_infrastructure_check(
    tmp_path, monkeypatch, capsys
):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/README.md", "trailing space \n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda *_: (_ for _ in ()).throw(AssertionError("must not run")),
    )

    assert hook.main() == 1
    assert "staged diff check failed" in capsys.readouterr().out


def test_staged_paths_are_sorted_and_nul_safe(tmp_path):
    root = _repo(tmp_path)
    _stage(root, "z file.md", "z\n")
    _stage(root, "a.md", "a\n")

    assert hook.staged_paths(root) == ["a.md", "z file.md"]


def test_rename_out_of_ai_surface_still_runs_structural_checks(tmp_path, monkeypatch):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/owner.md", "tracked owner\n")
    subprocess.run(["git", "-C", str(root), "commit", "-qm", "fixture"], check=True)
    subprocess.run(
        ["git", "-C", str(root), "mv", "docs/ai/owner.md", "owner.md"],
        check=True,
    )
    calls = []
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(hook, "detect_profile", lambda *_: "main")
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda checked_root, profile: calls.append((checked_root, profile)) or [],
    )

    assert hook.main() == 0
    assert "docs/ai/owner.md" in hook.staged_paths(root)
    assert len(calls) == 1
    assert calls[0][0] != root.resolve()
    assert calls[0][1] == "main"


def test_staged_checks_do_not_consume_untracked_reference_target(
    tmp_path, monkeypatch, capsys
):
    root = _repo(tmp_path)
    _stage(root, "docs/ai/example.md", "See `docs/new-owner.md`.\n")
    owner = root / "docs" / "new-owner.md"
    owner.write_text("worktree only\n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(hook, "detect_profile", lambda *_: "main")

    def check(snapshot, _profile):
        assert (snapshot / "docs/ai/example.md").is_file()
        return [] if (snapshot / "docs/new-owner.md").exists() else ["missing owner"]

    monkeypatch.setattr(hook, "run_checks", check)

    assert hook.main() == 1
    assert "missing owner" in capsys.readouterr().out


def test_staged_checks_do_not_consume_ignored_generated_adapter(
    tmp_path, monkeypatch, capsys
):
    root = _repo(tmp_path)
    _stage(root, ".gitignore", ".agents/\n")
    _stage(root, ".claude/commands/dart-new.md", "new workflow\n")
    generated = root / ".agents" / "skills" / "dart-new" / "SKILL.md"
    generated.parent.mkdir(parents=True)
    generated.write_text("generated but ignored\n")
    monkeypatch.setattr(hook, "parse_args", lambda: _args(root))
    monkeypatch.setattr(hook, "detect_profile", lambda *_: "main")
    monkeypatch.setattr(
        hook,
        "run_checks",
        lambda snapshot, _profile: (
            []
            if (snapshot / ".agents/skills/dart-new/SKILL.md").exists()
            else ["generated adapter is absent from index"]
        ),
    )

    assert hook.main() == 1
    assert "generated adapter is absent from index" in capsys.readouterr().out


def test_staged_snapshot_does_not_accept_intent_to_add_content(tmp_path):
    root = _repo(tmp_path)
    path = root / ".agents" / "skills" / "dart-new" / "SKILL.md"
    path.parent.mkdir(parents=True)
    path.write_text("worktree content\n")
    subprocess.run(
        ["git", "-C", str(root), "add", "-N", "-f", str(path.relative_to(root))],
        check=True,
    )

    with hook.staged_snapshot(root) as snapshot:
        staged = snapshot / path.relative_to(root)
        assert not staged.exists() or staged.read_text() != "worktree content\n"


@pytest.mark.parametrize(
    "path",
    (
        ".gitignore",
        "dart/common/AGENTS.md",
        "docs/onboarding/ai-tools.md",
        ".github/workflows/ci_lint.yml",
        ".github/workflows/ci_ubuntu.yml",
        "python/tests/unit/test_agent_capture.py",
        "python/tests/unit/gui/test_offscreen_render.py",
    ),
)
def test_structural_ai_paths_trigger_the_hook(path):
    assert hook.is_ai_infrastructure_path(path)
