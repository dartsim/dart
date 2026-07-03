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
    assert os.access(local, os.X_OK)

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


def test_disabled_foreign_hook_stays_disabled_when_preserved(tmp_path):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\nexit 7\n")
    hook.chmod(0o644)

    result = _install(repo, env)
    assert result.returncode == 0, result.stderr
    local = hook.parent / "pre-commit.local"
    assert local.exists()
    assert "exit 7" in local.read_text()
    assert not os.access(local, os.X_OK)

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    pixi = bin_dir / "pixi"
    pixi.write_text("#!/bin/sh\necho pixi:$* >&2\nexit 0\n")
    pixi.chmod(0o755)

    run = subprocess.run(
        [str(hook)],
        cwd=repo,
        env={**env, "PATH": f"{bin_dir}{os.pathsep}{env['PATH']}"},
        capture_output=True,
        text=True,
    )

    assert run.returncode == 0
    assert "pixi:run check-lint-quick" in run.stderr


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


def _run_guard(repo: Path, env: dict[str, str], command: str):
    payload = json.dumps({"tool_input": {"command": command}})
    run = subprocess.run(
        [str(GUARD)],
        cwd=repo,
        input=payload,
        env=env,
        capture_output=True,
        text=True,
    )
    return run.returncode, run.stderr


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
    return _run_guard(repo, env, command)


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


@pytest.mark.parametrize("use_absolute", [False, True])
def test_guard_detects_git_c_commits_inside_this_repo(tmp_path, use_absolute):
    repo, env = _init_repo(tmp_path)
    docs = repo / "docs"
    docs.mkdir()
    target = str(docs) if use_absolute else "docs"
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git -C {target} commit -m x")

    assert returncode == 0
    assert "would run 'pixi run check-lint-quick'" in stderr


@pytest.mark.parametrize(
    "target",
    [
        "$CLAUDE_PROJECT_DIR",
        "${CLAUDE_PROJECT_DIR}",
        '"$CLAUDE_PROJECT_DIR"',
        '"${CLAUDE_PROJECT_DIR}"',
    ],
)
def test_guard_expands_git_c_env_var_paths_inside_this_repo(tmp_path, target):
    repo, env = _init_repo(tmp_path)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git -C {target} commit -m x")

    assert returncode == 0
    assert "would run 'pixi run check-lint-quick'" in stderr


def test_guard_skips_git_c_commits_in_another_repo(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git -C {other} commit -m x")

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_expands_git_c_env_var_paths_before_other_repo_skip(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update(
        {
            "CLAUDE_PROJECT_DIR": str(repo),
            "DART_HOOK_DRY_RUN": "1",
            "OTHER_REPO": str(other),
        }
    )

    returncode, stderr = _run_guard(repo, env, 'git -C "$OTHER_REPO" commit -m x')

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_runs_when_only_foreign_executable_hook_installed(tmp_path):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\nexit 0\n")
    hook.chmod(0o755)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "git commit -m x")

    assert returncode == 0
    assert "would run 'pixi run check-lint-quick'" in stderr


def test_guard_stands_down_when_dart_managed_executable_hook_installed(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "git commit -m x")

    assert returncode == 0
    assert stderr == ""


@pytest.mark.parametrize(
    "command",
    [
        "git -c user.name=DART commit -m x",
        "git --config-env=user.name=GIT_USER_NAME commit -m x",
    ],
)
def test_guard_stands_down_for_unrelated_git_config_with_managed_hook(
    tmp_path, command
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update(
        {
            "CLAUDE_PROJECT_DIR": str(repo),
            "DART_HOOK_DRY_RUN": "1",
            "GIT_USER_NAME": "DART",
        }
    )

    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert stderr == ""


@pytest.mark.parametrize("flag", ["--no-verify", "-n"])
def test_guard_runs_for_no_verify_even_with_dart_managed_hook(tmp_path, flag):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git commit {flag} -m x")

    assert returncode == 0
    assert "would run 'pixi run check-lint-quick'" in stderr


def test_guard_runs_for_core_hookspath_override_even_with_dart_managed_hook(
    tmp_path,
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, "git -c core.hooksPath=/tmp/empty commit -m x"
    )

    assert returncode == 0
    assert "would run 'pixi run check-lint-quick'" in stderr


@pytest.mark.parametrize(
    "option",
    [
        "--config-env=core.hooksPath=EMPTY_HOOKS",
        "--config-env core.hooksPath=EMPTY_HOOKS",
    ],
)
def test_guard_runs_for_config_env_hookspath_override_even_with_dart_managed_hook(
    tmp_path, option
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update(
        {
            "CLAUDE_PROJECT_DIR": str(repo),
            "DART_HOOK_DRY_RUN": "1",
            "EMPTY_HOOKS": "/tmp/empty-hooks",
        }
    )

    returncode, stderr = _run_guard(repo, env, f"git {option} commit -m x")

    assert returncode == 0
    assert "would run 'pixi run check-lint-quick'" in stderr
