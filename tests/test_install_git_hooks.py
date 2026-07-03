"""Tests for scripts/install_git_hooks.py and the Claude Code commit guard.

Covers the enforcement wiring added with `pixi run install-hooks`: idempotent
install, foreign-hook preservation, the core.hooksPath refusal, and the
`.claude/hooks/pre-commit-guard.sh` commit-detection verdicts. POSIX-only:
the hook and guard are `/bin/sh` scripts gated on the executable bit.
"""

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
INSTALLER = ROOT / "scripts" / "install_git_hooks.py"
GUARD = ROOT / ".claude" / "hooks" / "pre-commit-guard.sh"

pytestmark = pytest.mark.skipif(
    os.name != "posix", reason="git hooks and the guard are POSIX sh scripts"
)


def _git_env(tmp_path: Path) -> dict[str, str]:
    """Environment that isolates git from user/system config."""
    env = dict(os.environ)
    env["GIT_CONFIG_GLOBAL"] = os.devnull
    env["GIT_CONFIG_SYSTEM"] = os.devnull
    env["HOME"] = str(tmp_path)
    env.pop("DART_SKIP_HOOKS", None)
    env.pop("DART_HOOK_DRY_RUN", None)
    return env


def _init_repo(tmp_path: Path) -> tuple[Path, dict[str, str]]:
    repo = tmp_path / "repo"
    repo.mkdir()
    env = _git_env(tmp_path)
    subprocess.run(["git", "init", "-q", str(repo)], check=True, env=env)
    return repo, env


def _install(repo: Path, env: dict[str, str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(INSTALLER)],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )


def _hook(repo: Path) -> Path:
    return repo / ".git" / "hooks" / "pre-commit"


def test_install_writes_executable_hook_and_is_idempotent(tmp_path):
    repo, env = _init_repo(tmp_path)

    first = _install(repo, env)
    assert first.returncode == 0, first.stderr
    hook = _hook(repo)
    assert hook.exists()
    assert os.access(hook, os.X_OK)
    assert "DART-MANAGED-HOOK" in hook.read_text()
    digest = hashlib.sha256(hook.read_bytes()).hexdigest()

    second = _install(repo, env)
    assert second.returncode == 0, second.stderr
    assert hashlib.sha256(hook.read_bytes()).hexdigest() == digest


def test_installed_hook_honors_skip_and_dry_run(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    hook = _hook(repo)

    skipped = subprocess.run(
        [str(hook)],
        cwd=repo,
        env={**env, "DART_SKIP_HOOKS": "1"},
        capture_output=True,
        text=True,
    )
    assert skipped.returncode == 0
    assert "skipped" in skipped.stderr

    dry = subprocess.run(
        [str(hook)],
        cwd=repo,
        env={**env, "DART_HOOK_DRY_RUN": "1"},
        capture_output=True,
        text=True,
    )
    assert dry.returncode == 0
    assert "would run 'pixi run check-lint-quick'" in dry.stderr


def test_foreign_hook_is_preserved_and_chained(tmp_path):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\nexit 7\n")
    hook.chmod(0o755)

    result = _install(repo, env)
    assert result.returncode == 0, result.stderr
    local = hook.parent / "pre-commit.local"
    assert local.exists()
    assert "exit 7" in local.read_text()

    # The chained foreign hook still runs and its failure propagates before
    # the lint gate is reached (so no dry-run flag is needed here).
    run = subprocess.run(
        [str(hook)],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )
    assert run.returncode == 7


def test_refuses_when_foreign_hook_and_local_both_exist(tmp_path):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\nexit 0\n")
    (hook.parent / "pre-commit.local").write_text("#!/bin/sh\nexit 0\n")

    result = _install(repo, env)
    assert result.returncode != 0
    assert "refusing" in result.stderr


def test_refuses_when_core_hookspath_is_set(tmp_path):
    repo, env = _init_repo(tmp_path)
    subprocess.run(
        ["git", "config", "core.hooksPath", ".githooks"],
        cwd=repo,
        check=True,
        env=env,
    )

    result = _install(repo, env)
    assert result.returncode != 0
    assert "core.hooksPath" in result.stderr
    assert not (repo / ".githooks" / "pre-commit").exists()


def _guard_verdict(tmp_path: Path, command: str, extra_env: dict | None = None):
    """Run the guard against a hookless scratch repo in dry-run mode.

    Returns (returncode, stderr). A 'commit' verdict surfaces as the dry-run
    notice on stderr; a 'skip' verdict produces no output.
    """
    repo, env = _init_repo(tmp_path)
    env.update(
        {"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"},
    )
    if extra_env:
        env.update(extra_env)
    payload = json.dumps({"tool_input": {"command": command}})
    run = subprocess.run(
        [str(GUARD)],
        input=payload,
        env=env,
        capture_output=True,
        text=True,
    )
    return run.returncode, run.stderr


@pytest.mark.parametrize(
    "command",
    [
        "git commit -m x",
        "command git commit -m x",
        "env A=1 git commit -m x",
        "(git commit -m x)",
        'FOO="a b" git commit -m x',
        "pixi run lint && git commit -m x",
    ],
)
def test_guard_detects_commit_forms(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)
    assert returncode == 0
    assert "would run 'pixi run check-lint-quick'" in stderr


@pytest.mark.parametrize(
    "command",
    [
        "git log --grep commit",
        "echo commit",
        "DART_SKIP_HOOKS=1 git commit -m x",
        "git -C /somewhere/else commit -m x",
    ],
)
def test_guard_skips_non_commits_and_bypasses(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)
    assert returncode == 0
    assert "would run" not in stderr


def test_guard_stands_down_when_executable_hook_installed(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})
    payload = json.dumps({"tool_input": {"command": "git commit -m x"}})
    run = subprocess.run(
        [str(GUARD)], input=payload, env=env, capture_output=True, text=True
    )
    assert run.returncode == 0
    assert run.stderr == ""
