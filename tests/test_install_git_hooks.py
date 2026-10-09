"""Tests for scripts/install_git_hooks.py and the shared agent commit guard.

Covers the managed staged, commit-message and publication gates in temporary
repositories, foreign-hook preservation, the core.hooksPath refusal, and the
`.claude/hooks/pre-commit-guard.sh` commit-detection verdicts. POSIX-only:
the hook and guard are `/bin/sh` scripts gated on the executable bit.
"""

import hashlib
import json
import os
import shlex
import shutil
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


def _write_gate(
    repo: Path,
    body: str = "print('direct-agent-gate', file=__import__('sys').stderr)\n",
) -> None:
    gate = repo / "scripts" / "check_agent_hook.py"
    gate.parent.mkdir(parents=True, exist_ok=True)
    gate.write_text(body)


def _hook(repo: Path, name: str = "pre-commit") -> Path:
    return repo / ".git" / "hooks" / name


def _push_repo(tmp_path):
    repo, env = _init_repo(tmp_path)
    env["DART_HOOK_PYTHON"] = sys.executable

    def git(*args, check=True):
        return subprocess.run(
            ["git", *args],
            cwd=repo,
            env=env,
            capture_output=True,
            text=True,
            check=check,
        )

    git("config", "user.name", "DART Test")
    git("config", "user.email", "test@example.com")
    git("checkout", "-b", "main")
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    git("add", ".")
    git("commit", "-qm", "Public base")
    remote = tmp_path / "remote.git"
    git("init", "--bare", "--initial-branch=main", str(remote))
    git("remote", "add", "origin", str(remote))
    git("push", "origin", "main")
    return repo, env, git


@pytest.mark.parametrize("removal", ["delete", "rename"])
@pytest.mark.parametrize(
    "name,new_ref",
    [
        ("pre-commit", False),
        ("commit-msg", False),
        ("pre-push", False),
        ("pre-push", True),
    ],
)
def test_hooks_block_removed_tracked_checkers(tmp_path, removal, name, new_ref):
    repo, env, git = _push_repo(tmp_path)
    _write_gate(repo, (ROOT / "scripts/check_agent_hook.py").read_text())
    git("add", "scripts/check_agent_hook.py")
    git("commit", "-qm", "Install staged checker fixture")
    git("push", "origin", "main")
    assert _install(repo, env).returncode == 0
    for script in ("check_agent_hook.py", "check_local_paths.py"):
        if removal == "delete":
            git("rm", f"scripts/{script}")
        else:
            git("mv", f"scripts/{script}", f"scripts/renamed_{script}")
    private_path = "/home/" + "example/private.md"
    (repo / "notes.md").write_text(private_path + "\n")
    git("add", "notes.md")
    if name == "pre-commit":
        result = git("commit", "-qm", "Public change", check=False)
    elif name == "commit-msg":
        message = repo / "message.txt"
        message.write_text(private_path + "\n")
        result = subprocess.run(
            [str(_hook(repo, name)), str(message)],
            cwd=repo,
            env=env,
            capture_output=True,
            text=True,
        )
    else:
        git("-c", "core.hooksPath=", "commit", "-qm", "Public change")
        target = "topic" if new_ref else "main"
        result = git("push", "origin", f"HEAD:refs/heads/{target}", check=False)
    assert result.returncode != 0, result.stderr


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
@pytest.mark.parametrize("failure", ["show", "python"])
def test_hooks_fail_closed_when_tracked_checker_cannot_be_recovered(
    tmp_path, name, failure
):
    repo, env, git = _push_repo(tmp_path)
    base = git("rev-parse", "HEAD").stdout.strip()
    assert _install(repo, env).returncode == 0
    git("rm", "scripts/check_agent_hook.py", "scripts/check_local_paths.py")
    if name == "pre-push":
        git("-c", "core.hooksPath=", "commit", "-qm", "Public change")
    real_git = shutil.which("git")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    if failure == "show":
        executable = bin_dir / "git"
        executable.write_text(
            '#!/bin/sh\nif [ "$1" = show ]; then exit 1; fi\n'
            f'exec {shlex.quote(real_git)} "$@"\n'
        )
    else:
        executable = bin_dir / "python3"
        executable.write_text("#!/bin/sh\nexit 1\n")
        env["DART_HOOK_PYTHON"] = str(executable)
    executable.chmod(0o755)
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    message = repo / "message.txt"
    message.write_text("Public summary\n")
    updates = None
    if name == "pre-push":
        head = git("rev-parse", "HEAD").stdout.strip()
        updates = f"refs/heads/main {head} refs/heads/main {base}\n"
        command = [str(_hook(repo, name)), "origin", str(tmp_path / "remote.git")]
    else:
        command = [str(_hook(repo, name)), str(message)]
    result = subprocess.run(
        command, cwd=repo, env=env, input=updates, capture_output=True, text=True
    )
    assert result.returncode != 0, result.stderr
    assert "cannot recover tracked checker" in result.stderr


def test_pre_push_recovers_checker_for_clean_change_and_removes_temporary_files(
    tmp_path,
):
    repo, env, git = _push_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    git("rm", "scripts/check_local_paths.py")
    git("-c", "core.hooksPath=", "commit", "-qm", "Public change")
    temporary = tmp_path / "checker-files"
    temporary.mkdir()
    env["TMPDIR"] = str(temporary)
    result = git("push", "origin", "main", check=False)
    assert result.returncode == 0, result.stderr
    assert "using tracked scripts/check_local_paths.py" in result.stderr
    assert list(temporary.iterdir()) == []


@pytest.mark.parametrize("leak", ["message", "content"])
@pytest.mark.parametrize("new_ref", [False, True])
def test_pre_push_blocks_cherry_picked_local_path(tmp_path, leak, new_ref):
    repo, env, git = _push_repo(tmp_path)
    base = git("rev-parse", "HEAD").stdout.strip()
    private_path = "/home/" + "example/private.md"
    git("checkout", "-b", "donor")
    (repo / "notes.md").write_text(
        private_path + "\n" if leak == "content" else "Public summary\n"
    )
    git("add", "notes.md")
    git("commit", "-qm", private_path if leak == "message" else "Public change")
    donor = git("rev-parse", "HEAD").stdout.strip()
    git("checkout", "-b", "topic", base)
    # A different parent forces cherry-pick to create a new commit.
    git("commit", "--allow-empty", "-qm", "Public topic")
    assert _install(repo, env).returncode == 0
    git("cherry-pick", donor)
    target = "topic" if new_ref else "main"
    result = git("push", "origin", f"HEAD:refs/heads/{target}", check=False)
    assert result.returncode != 0, result.stderr
    assert private_path in result.stdout
    assert "push blocked" in result.stderr
    assert git("ls-remote", "origin", f"refs/heads/{target}").stdout.strip() == (
        "" if new_ref else f"{base}\trefs/heads/main"
    )


def test_pre_push_allows_clean_updates_and_branch_deletion(tmp_path):
    repo, env, git = _push_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    git("commit", "--allow-empty", "-qm", "Public change")
    pushed = git("push", "origin", "main", "HEAD:refs/heads/topic")
    assert pushed.stderr.count("DART pre-push: scanning") == 2
    result = git("push", "origin", ":refs/heads/topic")
    assert "--commit-range" not in result.stderr
    assert git("ls-remote", "origin", "refs/heads/topic").stdout == ""


@pytest.mark.parametrize("new_ref", [False, True])
def test_pre_push_fetches_missing_remote_base_without_changing_refs(tmp_path, new_ref):
    repo, env, git = _push_repo(tmp_path)
    git("clone", str(tmp_path / "remote.git"), str(tmp_path / "other"))
    git(
        "-C",
        str(tmp_path / "other"),
        "-c",
        "user.name=DART Test",
        "-c",
        "user.email=test@example.com",
        "commit",
        "--allow-empty",
        "-qm",
        "Remote change",
    )
    git("-C", str(tmp_path / "other"), "push", "origin", "main")
    refs_before = git("show-ref").stdout
    fetch_head = repo / ".git/FETCH_HEAD"
    fetch_head.write_text("Keep existing fetch state\n")
    assert _install(repo, env).returncode == 0
    target = "topic" if new_ref else "main"
    result = git("push", "--force", "origin", f"HEAD:refs/heads/{target}")
    assert "DART pre-push: scanning" in result.stderr
    assert fetch_head.read_text() == "Keep existing fetch state\n"
    # A successful push may update its tracking ref, but fetching the base must not.
    if new_ref:
        assert git("show-ref", "refs/remotes/origin/main").stdout in refs_before


@pytest.mark.parametrize("mode", ["pixi", "without-tomllib"])
def test_pre_push_selects_compatible_python(tmp_path, mode):
    repo, env, git = _push_repo(tmp_path)
    env.pop("DART_HOOK_PYTHON")
    if mode == "pixi":
        python = repo / ".pixi/envs/default/bin/python"
        python.parent.mkdir(parents=True)
    else:
        python = tmp_path / "python3"
        env["DART_HOOK_PYTHON"] = str(python)
    python.write_text(
        '#!/bin/sh\n[ "$1" = "-c" ] && [ "$2" = "import tomllib" ] && exit 1\n'
        '[ "$1" = "-c" ] || echo selected-pre-push-python >&2\n'
        f'exec "{sys.executable}" "$@"\n'
    )
    python.chmod(0o755)
    assert _install(repo, env).returncode == 0
    private_path = "/home/" + "example/private.md"
    git(
        "-c",
        "core.hooksPath=unused-hooks",
        "commit",
        "--allow-empty",
        "-qm",
        private_path,
    )
    result = git("push", "origin", "main", check=False)
    assert result.returncode != 0
    assert private_path in result.stdout
    assert "selected-pre-push-python" in result.stderr
    assert "push blocked" in result.stderr


@pytest.mark.parametrize("unavailable", ["checker", "python"])
@pytest.mark.parametrize("local_status", [0, 7])
def test_pre_push_skips_unavailable_gate_and_honors_local_hook(
    tmp_path, unavailable, local_status
):
    repo, env, git = _push_repo(tmp_path)
    base = git("rev-parse", "HEAD").stdout.strip()
    git("commit", "--allow-empty", "-qm", "Public change")
    if unavailable == "checker":
        git("rm", "scripts/check_local_paths.py")
        git("commit", "-qm", "Checkout without checker")
        # An older branch lacks the checker at the remote base as well.
        git("push", "origin", "main")
        base = git("rev-parse", "HEAD").stdout.strip()
        git("commit", "--allow-empty", "-qm", "Public legacy change")
    else:
        bin_dir = tmp_path / "bin"
        bin_dir.mkdir()
        python = bin_dir / "python3"
        python.write_text("#!/bin/sh\nexit 1\n")
        python.chmod(0o755)
        env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
        env["DART_HOOK_PYTHON"] = str(python)
    hook = _hook(repo, "pre-push")
    hook.write_text(f"#!/bin/sh\ncat > push-input.txt\nexit {local_status}\n")
    hook.chmod(0o755)
    assert _install(repo, env).returncode == 0

    result = git("push", "origin", "main", "HEAD:refs/heads/topic", check=False)

    assert len((repo / "push-input.txt").read_text().splitlines()) == 2
    assert "DART pre-push: scanning" not in result.stderr
    if local_status:
        assert result.returncode != 0, result.stderr
        assert git("ls-remote", "origin", "refs/heads/main").stdout.strip() == (
            f"{base}\trefs/heads/main"
        )
        assert git("ls-remote", "origin", "refs/heads/topic").stdout == ""
    else:
        assert result.returncode == 0, result.stderr
        assert result.stderr.count(
            "DART pre-push: local-path gate unavailable in this checkout; skipping scan."
        ) == (2 if unavailable == "checker" else 1)
        head = git("rev-parse", "HEAD").stdout.strip()
        for ref in ("main", "topic"):
            assert git("ls-remote", "origin", f"refs/heads/{ref}").stdout.strip() == (
                f"{head}\trefs/heads/{ref}"
            )
        git("push", "origin", ":refs/heads/topic")


def test_pre_push_new_ref_uses_remote_default_branch_merge_base(tmp_path):
    repo, env, git = _push_repo(tmp_path)
    private_path = "/home/" + "example/private.md"
    git("commit", "--allow-empty", "-qm", private_path)
    git("push", "origin", "HEAD:refs/heads/trunk")
    git(
        "--git-dir",
        str(tmp_path / "remote.git"),
        "symbolic-ref",
        "HEAD",
        "refs/heads/trunk",
    )
    assert _install(repo, env).returncode == 0
    git("commit", "--allow-empty", "-qm", "Public change")
    result = git("push", "origin", "HEAD:refs/heads/topic")
    assert "DART pre-push: scanning" in result.stderr
    assert private_path not in result.stdout


@pytest.mark.parametrize("history", ["empty-remote", "unrelated"])
def test_pre_push_without_merge_base_scans_all_local_history(tmp_path, history):
    repo, env, git = _push_repo(tmp_path)
    if history == "empty-remote":
        git(
            "--git-dir",
            str(tmp_path / "remote.git"),
            "update-ref",
            "-d",
            "refs/heads/main",
        )
    else:
        git("checkout", "--orphan", "isolated")
    private_path = "/home/" + "example/private.md"
    git("commit", "--allow-empty", "-qm", private_path)
    assert _install(repo, env).returncode == 0
    result = git("push", "origin", "HEAD:refs/heads/topic", check=False)
    assert result.returncode != 0
    assert private_path in result.stdout
    assert "push blocked" in result.stderr


def test_pre_push_replays_stdin_to_foreign_hook_and_scans_every_ref(tmp_path):
    repo, env, git = _push_repo(tmp_path)
    hook = _hook(repo, "pre-push")
    hook.write_text('#!/bin/sh\ncat > push-input.txt\n[ "$1" = origin ] || exit 9\n')
    hook.chmod(0o755)
    assert _install(repo, env).returncode == 0
    base = git("rev-parse", "HEAD").stdout.strip()
    private_path = "/home/" + "example/private.md"
    git(
        "-c",
        "core.hooksPath=unused-hooks",
        "commit",
        "--allow-empty",
        "-qm",
        private_path,
    )
    head = git("rev-parse", "HEAD").stdout.strip()
    result = git(
        "push",
        "origin",
        f"{base}:refs/heads/clean",
        "HEAD:refs/heads/leaked",
        check=False,
    )
    assert result.returncode != 0, result.stderr
    assert private_path in result.stdout
    lines = (repo / "push-input.txt").read_text().splitlines()
    assert len(lines) == 2
    assert any(head in line and "refs/heads/leaked" in line for line in lines)


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
def test_install_writes_executable_hook_and_is_idempotent(tmp_path, name):
    repo, env = _init_repo(tmp_path)

    first = _install(repo, env)
    assert first.returncode == 0, first.stderr
    hook = _hook(repo, name)
    assert hook.exists()
    assert os.access(hook, os.X_OK)
    assert "DART-MANAGED-HOOK" in hook.read_text()
    assert "DART-MANAGED-HOOK v13 " in hook.read_text()
    digest = hashlib.sha256(hook.read_bytes()).hexdigest()

    second = _install(repo, env)
    assert second.returncode == 0, second.stderr
    assert hashlib.sha256(hook.read_bytes()).hexdigest() == digest


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
def test_installed_hook_honors_skip_and_dry_run(tmp_path, name):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    hook = _hook(repo, name)

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
    script = "check_agent_hook.py" if name == "pre-commit" else "check_local_paths.py"
    assert f"would run selected Python: scripts/{script}" in dry.stderr


@pytest.mark.parametrize(
    ("updated", "expected"),
    (
        (b"updated\r\n", 0),
        (b"updated \r\n", 1),
    ),
)
def test_installed_hook_fallback_accepts_crlf_but_rejects_trailing_space(
    tmp_path, updated, expected
):
    repo, env = _init_repo(tmp_path)
    subprocess.run(
        ["git", "config", "user.email", "test@example.com"],
        cwd=repo,
        check=True,
        env=env,
    )
    subprocess.run(
        ["git", "config", "user.name", "DART Test"],
        cwd=repo,
        check=True,
        env=env,
    )
    path = repo / "legacy.txt"
    path.write_bytes(b"base\r\n")
    subprocess.run(["git", "add", "legacy.txt"], cwd=repo, check=True, env=env)
    subprocess.run(
        ["git", "commit", "--no-verify", "-q", "-m", "base"],
        cwd=repo,
        check=True,
        env=env,
    )
    assert _install(repo, env).returncode == 0
    path.write_bytes(updated)
    subprocess.run(["git", "add", "legacy.txt"], cwd=repo, check=True, env=env)

    run = subprocess.run(
        [str(_hook(repo))],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )

    assert run.returncode == expected


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg"])
def test_installed_hook_prefers_repository_pixi_python(tmp_path, name):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    _write_gate(repo)
    (repo / "scripts" / "check_local_paths.py").write_text(
        "print('direct-message-gate', file=__import__('sys').stderr)\n"
    )
    pixi_python = repo / ".pixi" / "envs" / "default" / "bin" / "python"
    pixi_python.parent.mkdir(parents=True)
    pixi_python.symlink_to(sys.executable)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    incompatible = bin_dir / "python3"
    incompatible.write_text("#!/bin/sh\nexit 1\n")
    incompatible.chmod(0o755)

    run = subprocess.run(
        [str(_hook(repo, name)), "COMMIT_EDITMSG"],
        cwd=repo,
        env={**env, "PATH": f"{bin_dir}{os.pathsep}{env['PATH']}"},
        capture_output=True,
        text=True,
    )

    assert run.returncode == 0
    marker = "direct-agent-gate" if name == "pre-commit" else "direct-message-gate"
    assert marker in run.stderr
    assert str(pixi_python) in run.stderr


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg"])
def test_hooks_select_python_for_their_own_requirements(tmp_path, name):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    # Model a Python 3.9+ interpreter without tomllib, using real gate execution.
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    python = bin_dir / "python3"
    python.write_text(
        '#!/bin/sh\n[ "$1" = "-c" ] && [ "$2" = "import tomllib" ] && exit 1\n'
        f'exec "{sys.executable}" "$@"\n'
    )
    python.chmod(0o755)
    (bin_dir / "git").symlink_to(
        subprocess.run(
            ["which", "git"], capture_output=True, text=True, check=True
        ).stdout.strip()
    )
    private_path = "/home/" + "example/notes.md"
    (repo / "COMMIT_EDITMSG").write_text(private_path + "\n")
    run = subprocess.run(
        [str(_hook(repo, name)), "COMMIT_EDITMSG"],
        cwd=repo,
        env={**env, "PATH": str(bin_dir), "DART_HOOK_PYTHON": str(python)},
        capture_output=True,
        text=True,
    )
    if name == "commit-msg":
        assert run.returncode == 1, run.stderr
        assert f"1: {private_path}" in run.stdout
        assert "commit blocked" in run.stderr
    else:
        assert run.returncode == 0, run.stderr
        assert "staged diff fallback" in run.stderr
        assert "direct-agent-gate" not in run.stderr


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
def test_foreign_hook_is_preserved_and_chained(tmp_path, name):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo, name)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text('#!/bin/sh\n[ "$1" = "message with spaces" ] || exit 9\nexit 7\n')
    hook.chmod(0o755)

    result = _install(repo, env)
    assert result.returncode == 0, result.stderr
    local = hook.parent / f"{name}.local"
    assert local.exists()
    assert "exit 7" in local.read_text()
    assert os.access(local, os.X_OK)

    # The chained foreign hook still runs and its failure propagates before
    # the staged safety gate is reached (so no dry-run flag is needed here).
    run = subprocess.run(
        [str(hook), "message with spaces"],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )
    assert run.returncode == 7


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
def test_disabled_foreign_hook_stays_disabled_when_preserved(tmp_path, name):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo, name)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\nexit 7\n")
    hook.chmod(0o644)

    result = _install(repo, env)
    assert result.returncode == 0, result.stderr
    local = hook.parent / f"{name}.local"
    assert local.exists()
    assert "exit 7" in local.read_text()
    assert not os.access(local, os.X_OK)

    _write_gate(repo)
    (repo / "scripts" / "check_local_paths.py").write_text(
        "print('direct-agent-gate', file=__import__('sys').stderr)\n"
    )

    run = subprocess.run(
        [str(hook), "COMMIT_EDITMSG"],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )

    assert run.returncode == 0
    if name != "pre-push":
        assert "direct-agent-gate" in run.stderr


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
def test_refuses_when_foreign_hook_and_local_both_exist(tmp_path, name):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo, name)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\nexit 0\n")
    (hook.parent / f"{name}.local").write_text("#!/bin/sh\nexit 0\n")

    result = _install(repo, env)
    assert result.returncode != 0
    assert "refusing" in result.stderr


@pytest.mark.parametrize("hooks_path", [".githooks", ""])
def test_refuses_when_core_hookspath_is_set(tmp_path, hooks_path):
    repo, env = _init_repo(tmp_path)
    subprocess.run(
        ["git", "config", "core.hooksPath", hooks_path],
        cwd=repo,
        check=True,
        env=env,
    )

    result = _install(repo, env)
    assert result.returncode != 0
    assert "core.hooksPath" in result.stderr
    assert not (repo / ".githooks" / "pre-commit").exists()
    assert not (repo / ".githooks" / "commit-msg").exists()
    assert not (repo / ".githooks" / "pre-push").exists()
    assert not (repo / "pre-commit").exists()
    assert not (repo / "commit-msg").exists()
    assert not (repo / "pre-push").exists()


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
@pytest.mark.parametrize("target_kind", ["foreign", "managed", "dangling"])
def test_install_preserves_hook_symlink_without_writing_target(
    tmp_path, name, target_kind
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    hook = _hook(repo, name)
    target = tmp_path / "hook-target"
    content = hook.read_bytes() if target_kind == "managed" else b"#!/bin/sh\nexit 0\n"
    if target_kind != "dangling":
        target.write_bytes(content)
    hook.unlink()
    hook.symlink_to(target)

    result = _install(repo, env)
    assert result.returncode == 0, result.stderr
    assert not hook.is_symlink()
    assert "DART-MANAGED-HOOK" in hook.read_text()
    local = hook.with_name(f"{name}.local")
    assert local.is_symlink()
    assert local.readlink() == target
    assert target.exists() == (target_kind != "dangling")
    if target.exists():
        assert target.read_bytes() == content


@pytest.mark.parametrize("name", ["pre-commit", "commit-msg", "pre-push"])
def test_install_refuses_symlink_hook_backup_conflict_before_any_writes(tmp_path, name):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    hook = _hook(repo, name)
    hook.unlink()
    hook.symlink_to("missing-target")
    local = hook.with_name(f"{name}.local")
    local.symlink_to("missing-backup")
    other = _hook(repo, "commit-msg" if name == "pre-commit" else "pre-commit")
    other.write_text("#!/bin/sh\nexit 0\n")
    result = _install(repo, env)
    assert result.returncode != 0
    assert "refusing" in result.stderr
    assert hook.readlink() == Path("missing-target")
    assert local.readlink() == Path("missing-backup")
    assert other.read_text() == "#!/bin/sh\nexit 0\n"
    assert not other.with_name(f"{other.name}.local").exists()


def test_commit_msg_hook_blocks_private_message_in_real_commit(tmp_path):
    repo, env = _init_repo(tmp_path)
    _write_gate(repo)
    (repo / "scripts" / "check_local_paths.py").write_bytes(
        (ROOT / "scripts" / "check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    message = repo / "message with spaces.txt"
    private_path = ".sisyphus" + "/plans/private.md"
    message.write_text(f"{private_path}\n")
    direct = subprocess.run(
        [str(_hook(repo, "commit-msg")), message.name],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )
    assert direct.returncode == 1, direct.stderr
    assert f"1: {private_path}" in direct.stdout
    command = [
        "git",
        "-c",
        "user.name=DART Test",
        "-c",
        "user.email=test@example.com",
        "commit",
        "--allow-empty",
        "-q",
        "-F",
        message.name,
    ]
    blocked = subprocess.run(command, cwd=repo, env=env, capture_output=True, text=True)
    assert blocked.returncode == 1, blocked.stderr
    assert f"1: {private_path}" in blocked.stderr
    message.write_text("Public summary\n")
    allowed = subprocess.run(command, cwd=repo, env=env, capture_output=True, text=True)
    assert allowed.returncode == 0, allowed.stderr


@pytest.mark.parametrize("prefix", ["", "# See "])
def test_commit_msg_hook_blocks_inline_hash_message(tmp_path, prefix):
    repo, env = _init_repo(tmp_path)
    _write_gate(repo)
    (repo / "scripts" / "check_local_paths.py").write_bytes(
        (ROOT / "scripts" / "check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    private_path = ".sisyphus" + "/plans/private.md"
    result = subprocess.run(
        [
            "git",
            "-c",
            "user.name=DART Test",
            "-c",
            "user.email=test@example.com",
            "commit",
            "--allow-empty",
            "-m",
            prefix + private_path,
        ],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1, result.stderr
    assert f"1: {private_path}" in result.stderr


def test_commit_msg_hook_auto_comment_without_instructions_fails_closed(tmp_path):
    repo, env = _init_repo(tmp_path)
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    private_path = "/home/" + "example/private.md"
    message = repo / "message.txt"
    message.write_text(f"Public summary\n# {private_path}\n")
    editor = repo / "editor.sh"
    editor.write_text('#!/bin/sh\ncat message.txt > "$1"\n')
    editor.chmod(0o755)
    command = [
        "git",
        "-c",
        "user.name=Example",
        "-c",
        "user.email=example@example.com",
        "-c",
        "core.commentChar=auto",
        "commit",
        "--allow-empty",
        "-e",
        "-F",
        message.name,
    ]
    result = subprocess.run(
        command,
        cwd=repo,
        env={**env, "GIT_EDITOR": str(editor)},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1, result.stderr
    assert private_path in result.stderr
    # Prove Git retains this hash line after selecting another comment character.
    result = subprocess.run(
        [*command[:1], "-c", "core.hooksPath=" + os.devnull, *command[1:]],
        cwd=repo,
        env={**env, "GIT_EDITOR": str(editor)},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    published = subprocess.run(
        ["git", "show", "-s", "--format=%B", "HEAD"],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )
    assert f"# {private_path}" in published.stdout


@pytest.mark.parametrize("arguments", [[], ["--no-verbose"], ["--no-verbose", "-v"]])
def test_commit_msg_hook_configured_verbose_ignores_fixture_diff(tmp_path, arguments):
    repo, env = _init_repo(tmp_path)
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    fixture = repo / "tests/test_check_local_paths.py"
    fixture.parent.mkdir()
    fixture.write_text('fixture = "' + "/home/" + 'example/private.md"\n')
    command = [
        "git",
        "-c",
        "user.name=Example",
        "-c",
        "user.email=example@example.com",
        "commit",
    ]
    subprocess.run(["git", "add", "."], cwd=repo, env=env, check=True)
    subprocess.run(
        [*command, "-qm", "Public base"],
        cwd=repo,
        env=env,
        check=True,
        capture_output=True,
    )
    fixture.write_text(fixture.read_text() + "# Public fixture update\n")
    subprocess.run(["git", "add", "."], cwd=repo, env=env, check=True)
    subprocess.run(
        ["git", "config", "commit.verbose", "true"], cwd=repo, env=env, check=True
    )
    assert _install(repo, env).returncode == 0
    editor = repo / "editor.sh"
    editor.write_text(
        '#!/bin/sh\nprintf "Public summary\\n" > summary.txt\ncat "$1" >> summary.txt\ncat summary.txt > "$1"\n'
    )
    editor.chmod(0o755)
    result = subprocess.run(
        [*command, *arguments],
        cwd=repo,
        env={**env, "GIT_EDITOR": str(editor)},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_pre_push_blocks_gitlink_hidden_by_diff_config(tmp_path):
    repo, env, git = _push_repo(tmp_path)
    base = git("rev-parse", "HEAD").stdout.strip()
    private_path = "scratchpad" + "/private-module"
    git("config", "diff.ignoreSubmodules", "all")
    git("update-index", "--add", "--cacheinfo", f"160000,{base},{private_path}")
    git("commit", "-qm", "Public gitlink")
    assert _install(repo, env).returncode == 0
    result = git("push", "origin", "main", check=False)
    assert result.returncode != 0, result.stderr
    assert private_path in result.stdout
    assert "push blocked" in result.stderr
    assert (
        git("ls-remote", "origin", "refs/heads/main").stdout.strip()
        == f"{base}\trefs/heads/main"
    )


@pytest.mark.parametrize(
    "cleanup,expected", [("strip", 0), ("verbatim", 1), ("whitespace", 1)]
)
def test_merge_hook_checks_editor_comments_kept_by_cleanup(tmp_path, cleanup, expected):
    repo, env = _init_repo(tmp_path)

    def git(*args, check=True):
        return subprocess.run(
            ["git", *args],
            cwd=repo,
            env=env,
            check=check,
            capture_output=True,
            text=True,
        )

    git("config", "user.name", "Example")
    git("config", "user.email", "test@example.com")
    git("checkout", "-b", "base")
    git("commit", "--allow-empty", "-qm", "Public base")
    git("checkout", "-b", "topic")
    (repo / "topic.md").write_text("Public topic\n")
    git("add", "topic.md")
    git("commit", "-qm", "Public topic")
    git("checkout", "base")
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    private_path = "/home/" + "example/private.md"
    editor = repo / "editor.sh"
    editor.write_text(
        '#!/bin/sh\nprintf "\\n# %s\\n" ' + shlex.quote(private_path) + ' >> "$1"\n'
    )
    editor.chmod(0o755)
    env["GIT_EDITOR"] = str(editor)
    result = git(
        "merge", "--no-ff", "--edit", f"--cleanup={cleanup}", "topic", check=False
    )
    assert result.returncode == expected, result.stderr
    if expected:
        assert private_path in result.stderr
    else:
        assert private_path not in git("log", "-1", "--format=%B").stdout


@pytest.mark.parametrize("mode", ["autoedit-no", "no-edit", "non-interactive"])
def test_merge_hook_blocks_annotated_tag_comments_with_default_cleanup(tmp_path, mode):
    repo, env = _init_repo(tmp_path)

    def git(*args, check=True):
        return subprocess.run(
            ["git", *args],
            cwd=repo,
            env=env,
            check=check,
            capture_output=True,
            text=True,
        )

    git("config", "user.name", "Example")
    git("config", "user.email", "test@example.com")
    git("checkout", "-b", "base")
    git("commit", "--allow-empty", "-qm", "Public base")
    git("checkout", "-b", "topic")
    git("commit", "--allow-empty", "-qm", "Public topic")
    private_path = "/home/" + "example/private.md"
    git("tag", "-a", "topic-tag", "--cleanup=verbatim", "-m", f"# Build {private_path}")
    git("checkout", "base")
    base = git("rev-parse", "HEAD").stdout.strip()
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    env.pop("GIT_MERGE_AUTOEDIT", None)
    env["GIT_EDITOR"] = ":"
    if mode == "autoedit-no":
        env["GIT_MERGE_AUTOEDIT"] = "no"
    result = git(
        "merge",
        "--no-ff",
        *(["--no-edit"] if mode == "no-edit" else []),
        "topic-tag",
        check=False,
    )
    assert result.returncode != 0, result.stderr
    assert private_path in result.stderr
    assert git("rev-parse", "HEAD").stdout.strip() == base


def test_commit_msg_hook_reports_unavailable_checker(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    result = subprocess.run(
        [str(_hook(repo, "commit-msg")), "COMMIT_EDITMSG"],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "local-path gate unavailable" in result.stderr


def test_commit_msg_hook_passes_git_parent_pid(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    scripts = repo / "scripts"
    scripts.mkdir()
    (scripts / "check_local_paths.py").write_text(
        "import sys\nprint(' '.join(sys.argv[1:]))\n"
    )
    result = subprocess.run(
        [str(_hook(repo, "commit-msg")), "message.txt"],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == f"--commit-msg-file message.txt --git-pid {os.getpid()}\n"


def test_commit_msg_hook_scans_template_instruction_from_file_parent(tmp_path):
    repo, env = _init_repo(tmp_path)
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    message = repo / "message.txt"
    private_path = "scratchpad" + "/example.md"
    message.write_text(
        "Public summary\n# Lines starting with '#' will be ignored,\n"
        f"# {private_path}\n"
    )
    result = subprocess.run(
        [
            "git",
            "-c",
            "user.name=Example",
            "-c",
            "user.email=example@example.com",
            "commit",
            "--allow-empty",
            "-F",
            message.name,
        ],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1, result.stderr
    assert private_path in result.stderr


@pytest.mark.parametrize("global_args", [[], ["-c", "commit.cleanup=strip"]])
def test_commit_msg_hook_with_commit_named_directory(tmp_path, global_args):
    repo, env = _init_repo(tmp_path)
    target = tmp_path / "commit"
    repo.rename(target)
    _write_gate(target)
    (target / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    assert _install(target, env).returncode == 0
    result = subprocess.run(
        [
            "git",
            "-C",
            "commit",
            "-c",
            "user.name=Example",
            "-c",
            "user.email=example@example.com",
            *global_args,
            "commit",
            "--allow-empty",
            "-m",
            "Public summary",
        ],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    "arguments,expected",
    [([], 0), (["-F", "message.txt"], 1), (["--cleanup=whitespace"], 1)],
)
@pytest.mark.parametrize(
    "configured,global_args",
    [
        (None, []),
        ("verbatim", []),
        ("whitespace", []),
        ("strip", ["-c", "commit.cleanup=verbatim"]),
    ],
)
def test_commit_msg_hook_uses_cleanup_with_localized_editor(
    tmp_path, arguments, expected, configured, global_args
):
    repo, env = _init_repo(tmp_path)
    if configured is not None:
        subprocess.run(
            ["git", "config", "commit.cleanup", configured],
            cwd=repo,
            env=env,
            check=True,
        )
        expected = 1
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    private_path = "scratchpad" + "/example.md"
    message = (
        "Public summary\n; Veuillez saisir le message de validation.\n"
        + f"; Voir {private_path}\n"
    )
    (repo / "message.txt").write_text(message)
    editor = repo / "editor.sh"
    editor.write_text('#!/bin/sh\ncat message.txt > "$1"\n')
    editor.chmod(0o755)
    result = subprocess.run(
        [
            "git",
            "-c",
            "user.name=Example",
            "-c",
            "user.email=example@example.com",
            "-c",
            "core.commentChar=;",
            *global_args,
            "commit",
            "--allow-empty",
            *arguments,
        ],
        cwd=repo,
        env={**env, "GIT_EDITOR": str(editor)},
        capture_output=True,
        text=True,
    )
    assert result.returncode == expected, result.stderr
    if expected:
        assert private_path in result.stderr


def test_pre_commit_hook_blocks_local_path_outside_ai_inputs(tmp_path):
    repo, env = _init_repo(tmp_path)
    _write_gate(repo)
    (repo / "scripts/check_local_paths.py").write_bytes(
        (ROOT / "scripts/check_local_paths.py").read_bytes()
    )
    assert _install(repo, env).returncode == 0
    private_path = "scratchpad" + "/example.md"
    (repo / "source.cpp").write_text(f"// {private_path}\n")
    subprocess.run(["git", "add", "source.cpp"], cwd=repo, env=env, check=True)
    result = subprocess.run(
        [str(_hook(repo))], cwd=repo, env=env, capture_output=True, text=True
    )
    assert result.returncode == 1
    assert f"source.cpp:1: {private_path}" in result.stdout


def _run_guard(
    repo: Path, env: dict[str, str], command: str, input_key: str = "command"
):
    payload = json.dumps({"tool_input": {input_key: command}})
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


def test_guard_accepts_codex_cmd_input(tmp_path):
    repo, env = _init_repo(tmp_path)
    env.update({"CODEX_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "git commit -m x", input_key="cmd")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_prefers_repository_pixi_python(tmp_path):
    repo, env = _init_repo(tmp_path)
    _write_gate(repo)
    pixi_python = repo / ".pixi" / "envs" / "default" / "bin" / "python"
    pixi_python.parent.mkdir(parents=True)
    pixi_python.symlink_to(sys.executable)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    incompatible = bin_dir / "python3"
    incompatible.write_text("#!/bin/sh\nexit 1\n")
    incompatible.chmod(0o755)
    env.update(
        {
            "CLAUDE_PROJECT_DIR": str(repo),
            "PATH": f"{bin_dir}{os.pathsep}{env['PATH']}",
        }
    )

    returncode, stderr = _run_guard(repo, env, "git commit --no-verify -m x")

    assert returncode == 0
    assert "direct-agent-gate" in stderr


def test_guard_normalizes_windows_python_crlf(tmp_path):
    repo, env = _init_repo(tmp_path)
    fake_python = tmp_path / "fake-python"
    fake_python.write_text(
        "#!/bin/sh\nprintf 'commit\\r\\n%s\\r\\n' \"$FAKE_REPO_ROOT\"\n"
    )
    fake_python.chmod(0o755)
    env.update(
        {
            "CLAUDE_PROJECT_DIR": str(repo),
            "DART_HOOK_DRY_RUN": "1",
            "DART_HOOK_PYTHON": str(fake_python),
            "FAKE_REPO_ROOT": str(repo),
        }
    )

    returncode, stderr = _run_guard(repo, env, "git commit --no-verify -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_fails_closed_for_unknown_injected_classifier_result(tmp_path):
    repo, env = _init_repo(tmp_path)
    fake_python = tmp_path / "fake-python"
    fake_python.write_text("#!/bin/sh\nprintf 'unexpected\\r\\n\\r\\n'\n")
    fake_python.chmod(0o755)
    env.update(
        {
            "CLAUDE_PROJECT_DIR": str(repo),
            "DART_HOOK_PYTHON": str(fake_python),
        }
    )

    returncode, stderr = _run_guard(repo, env, "git commit --no-verify -m x")

    assert returncode == 2
    assert "invalid commit-detection result" in stderr


@pytest.mark.parametrize(
    "command",
    [
        "git commit -m x",
        "/usr/bin/git commit -m x",
        "command /usr/local/bin/git commit -m x",
        "git -c user.name='DART Bot' commit -m x",
        "GIT commit -m x",
        "GIT.EXE commit -m x",
        "command git commit -m x",
        "command -- git commit -m x",
        "command -p git commit -m x",
        "exec -a label git commit -m x",
        "time -p git commit -m x",
        "nice -n 5 git commit -m x",
        "nohup -- git commit -m x",
        "/usr/bin/env git commit -m x",
        "/usr/bin/time -p git commit -m x",
        "/usr/bin/nice -n 5 git commit -m x",
        "/usr/bin/nohup -- git commit -m x",
        "env A=1 git commit -m x",
        "env DART_SKIP_HOOKS=0 git commit -m x",
        "env -u FOO git commit -m x",
        "env --unset=FOO git commit -m x",
        "env -C . git commit -m x",
        "env -i git commit -m x",
        "env --ignore-environment git commit -m x",
        "env -S 'git commit -m x'",
        "env --split-string='git commit -m x'",
        "(git commit -m x)",
        'FOO="a b" git commit -m x',
        "git com\\mit -m x",
        'git com"mit" -m x',
        "git 'com'mit -m x",
        "pixi run lint && git commit -m x",
    ],
)
def test_guard_detects_commit_forms(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)
    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command",
    [
        "if git diff --cached --quiet; then :; else git commit -m x; fi",
        "if git diff --cached --quiet; then git commit -m x; fi",
        "if git diff --cached --quiet\nthen git commit -m x\nfi",
        "while false; do git commit -m x; done",
        "if true; then (git commit -m x); fi",
        "if true; then { git commit -m x; }; fi",
    ],
)
def test_guard_detects_commits_inside_shell_conditionals(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)
    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command",
    [
        # `&` terminates the preceding command, so the trailing `git commit`
        # still runs and must route to the gate even when a non-git segment
        # precedes it. Splitting on `&` must not disturb `&&` handling.
        "true & git commit -m x",
        "false & git commit -m x",
        "a && b & git commit -m x",
        "git commit -m x &",
        "git commit -m x & true",
    ],
)
def test_guard_detects_commit_after_background_operator(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)
    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command",
    [
        "git log --grep commit",
        "echo commit",
        "DART_SKIP_HOOKS=1 git commit -m x",
        "env DART_SKIP_HOOKS=1 git commit -m x",
        'env DART_SKIP_HOOKS="1" git commit -m x',
        "env -S 'DART_SKIP_HOOKS=1 git commit -m x'",
        "env -S 'git log --grep commit'",
        "git -C /somewhere/else commit -m x",
        # A background operator must not turn a non-commit chain into a gate,
        # and must still honor a DART_SKIP_HOOKS bypass on the commit segment.
        "true & echo commit",
        "DART_SKIP_HOOKS=1 git commit -m x & true",
    ],
)
def test_guard_skips_non_commits_and_bypasses(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)
    assert returncode == 0
    assert "would run" not in stderr


def test_guard_respects_skip_assignment_inside_shell_conditionals(tmp_path):
    command = "if true; then DART_SKIP_HOOKS=1 git commit -m x; fi"

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
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


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
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_skips_git_c_commits_in_another_repo(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git -C {other} commit -m x")

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_routes_linked_worktree_commit_to_target_index(tmp_path):
    repo, env = _init_repo(tmp_path)
    subprocess.run(
        ["git", "-C", str(repo), "config", "user.email", "test@example.com"],
        check=True,
        env=env,
    )
    subprocess.run(
        ["git", "-C", str(repo), "config", "user.name", "DART Test"],
        check=True,
        env=env,
    )
    (repo / "README.md").write_text("base\n")
    subprocess.run(["git", "-C", str(repo), "add", "README.md"], check=True, env=env)
    subprocess.run(
        ["git", "-C", str(repo), "commit", "-q", "-m", "base"],
        check=True,
        env=env,
    )
    linked = tmp_path / "linked"
    subprocess.run(
        [
            "git",
            "-C",
            str(repo),
            "worktree",
            "add",
            "-q",
            "-b",
            "linked-test",
            str(linked),
        ],
        check=True,
        env=env,
    )
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, f"git -C {linked} commit --no-verify -m x"
    )

    assert returncode == 0
    assert "would run" in stderr
    assert f"in {linked}" in stderr


def test_guard_skips_env_c_commits_in_another_repo(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"env -C {other} git commit -m x")

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_skips_env_split_commits_in_another_repo(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"env -S 'git -C {other} commit -m x'")

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


@pytest.mark.parametrize(
    "command",
    [
        "cd docs && git -C .. commit -m x",
        "cd docs; git -C .. commit -m x",
        "cd docs\ngit -C .. commit -m x",
        "cd docs && env -C .. git commit -m x",
    ],
)
def test_guard_preserves_shell_cwd_for_relative_commit_paths(tmp_path, command):
    repo, env = _init_repo(tmp_path)
    (repo / "docs").mkdir()
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_does_not_preserve_failed_cd_cwd_for_or_chain(tmp_path):
    repo, env = _init_repo(tmp_path)
    (repo / "docs").mkdir()
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"cd docs || git -C {other} commit -m x")

    assert returncode == 0
    assert "would run" not in stderr


@pytest.mark.parametrize(
    "command_template",
    [
        "false && cd {other}; git commit -m x",
        "true || cd {other}; git commit -m x",
    ],
)
def test_guard_does_not_carry_cwd_from_skipped_conditional_cd(
    tmp_path, command_template
):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, command_template.format(other=other))

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command_template",
    [
        "true && cd {other}; git commit -m x",
        "false || cd {other}; git commit -m x",
    ],
)
def test_guard_carries_cwd_from_executed_conditional_cd(tmp_path, command_template):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, command_template.format(other=other))

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_keeps_uncertain_conditional_cd_cwd_fail_closed(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        other, env, f"test -d {repo} && cd {repo}; git commit -m x"
    )

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_treats_unparsed_cd_status_as_uncertain(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, f"cd -P . || cd {other}; git commit -m x"
    )

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_does_not_carry_pipeline_cd_cwd(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"true | cd {other}; git commit -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_does_not_carry_multisegment_subshell_cd_cwd(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, f"(true; cd {other}; true); git commit -m x"
    )

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_does_not_carry_command_substitution_cd_cwd(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, f"echo $(true; cd {other}; pwd); git commit -m x"
    )

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_does_not_use_command_substitution_status_for_outer_chain(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = f"echo $(true; false) && cd {repo}; git commit -m x"
    returncode, stderr = _run_guard(other, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_does_not_carry_backtick_command_substitution_cd_cwd(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = f"echo `true; cd {other}; pwd`; git commit -m x"
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command",
    (
        "echo $(git commit -m x)",
        "value=$(git commit -m x)",
        "echo `git commit -m x`",
    ),
)
def test_guard_detects_embedded_command_substitution_commit(tmp_path, command):
    repo, env = _init_repo(tmp_path)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_preserves_outer_git_command_around_substitution(tmp_path):
    repo, env = _init_repo(tmp_path)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, 'git -C "$(pwd)" commit -m x')

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize("definition", ("change_dir()", "function change_dir"))
def test_guard_does_not_carry_function_definition_body_cwd(tmp_path, definition):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = f"{definition} {{ true; cd {other}; }}; git commit -m x"
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_does_not_execute_function_definition_body_commit(tmp_path):
    repo, env = _init_repo(tmp_path)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = "commit_later() { git commit -m x; }"
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_does_not_treat_path_qualified_builtin_as_shell_builtin(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = f"/usr/bin/builtin cd {other}; git commit -m x"
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_carries_brace_group_cd_cwd(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, f"{{ true; cd {other}; true; }}; git commit -m x"
    )

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_ignores_quoted_shell_separators(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = f"echo 'text; cd {other}; more'; git commit -m x"
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize("wrapper", ("env", "nice", "nohup", "exec"))
def test_guard_does_not_carry_external_wrapper_cd_cwd(tmp_path, wrapper):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"{wrapper} cd {other}; git commit -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_tracks_builtin_cd_from_foreign_repo(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(other, env, f"builtin cd {repo}; git commit -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_poison_cwd_for_builtin_eval(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        other, env, f"builtin eval 'cd {repo}'; git commit -m x"
    )

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command_template",
    (
        "eval 'cd {repo}'; git commit -m x",
        "source change-dir.sh; git commit -m x",
        ". change-dir.sh; git commit -m x",
    ),
)
def test_guard_poison_cwd_for_opaque_parent_shell_mutators(tmp_path, command_template):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(other, env, command_template.format(repo=repo))

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command_template",
    [
        "cd {missing}; git commit -m x",
        "cd {missing}\ngit commit -m x",
        "cd {missing} && true; git commit -m x",
    ],
)
def test_guard_does_not_preserve_missing_cd_cwd(tmp_path, command_template):
    repo, env = _init_repo(tmp_path)
    missing = tmp_path / "missing"
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = command_template.format(missing=missing)
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_preserves_conditional_cd_cwd_for_relative_commit_paths(tmp_path):
    repo, env = _init_repo(tmp_path)
    (repo / "docs").mkdir()
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, "if cd docs; then git -C .. commit -m x; fi"
    )

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_skips_conditional_cd_commit_in_another_repo(tmp_path):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(
        repo, env, f"if cd {other}; then git commit -m x; fi"
    )

    assert returncode == 0
    assert "would run" not in stderr


@pytest.mark.parametrize(
    "command_template",
    [
        "if false; then cd {other}; fi; git commit -m x",
        "if false\nthen cd {other}\nfi\ngit commit -m x",
    ],
)
def test_guard_does_not_preserve_conditional_branch_cd_cwd(tmp_path, command_template):
    repo, env = _init_repo(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    subprocess.run(["git", "init", "-q", str(other)], check=True, env=env)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = command_template.format(other=other)
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command",
    [
        "cat <<'EOF'\ngit commit -m example\nEOF",
        'cat <<"EOF"\ngit commit -m example\nEOF',
        "cat <<-EOF\n\tgit commit -m example\n\tEOF",
    ],
)
def test_guard_ignores_git_commit_inside_heredoc_body(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)

    assert returncode == 0
    assert "would run" not in stderr


def test_guard_detects_git_commit_after_heredoc_body(tmp_path):
    command = "cat <<'EOF'\ngit commit -m example\nEOF\ngit commit -m real"

    returncode, stderr = _guard_verdict(tmp_path, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command",
    [
        'echo "<<EOF"\ngit commit -m x',
        "echo '<<EOF'\ngit commit -m x",
        'printf "%s\\n" "<<-EOF"\ngit commit -m x',
    ],
)
def test_guard_detects_git_commit_after_quoted_heredoc_marker_text(tmp_path, command):
    returncode, stderr = _guard_verdict(tmp_path, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_runs_when_only_foreign_executable_hook_installed(tmp_path):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\nexit 0\n")
    hook.chmod(0o755)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "git commit -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "body",
    [
        "#!/bin/sh\n# DART-MANAGED-HOOK v1\npixi run check-lint-quick\n",
        "#!/bin/sh\n# DART-MANAGED-HOOK v7\npixi run check-lint-quick\n",
        (
            "#!/bin/sh\n# DART-MANAGED-HOOK v50\n"
            'if ! "$python_cmd" scripts/check_agent_hook.py --profile staged; then\n'
            "    exit 1\nfi\n"
        ),
    ],
)
def test_guard_runs_when_dart_managed_hook_is_stale_or_incomplete(tmp_path, body):
    repo, env = _init_repo(tmp_path)
    hook = _hook(repo)
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text(body)
    hook.chmod(0o755)
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "git commit -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_stands_down_when_dart_managed_executable_hook_installed(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "git commit -m x")

    assert returncode == 0
    assert stderr == ""


def test_guard_stands_down_for_env_split_with_dart_managed_hook(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "env -S 'git commit -m x'")

    assert returncode == 0
    assert stderr == ""


@pytest.mark.parametrize(
    "command",
    [
        "git -c user.name=DART commit -m x",
        "git -c core.hooksPathSuffix=/tmp/empty-hooks commit -m x",
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


@pytest.mark.parametrize(
    "command",
    [
        (
            "GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=user.name "
            "GIT_CONFIG_VALUE_0=DART git commit -m x"
        ),
        (
            "env GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=user.name "
            "GIT_CONFIG_VALUE_0=DART git commit -m x"
        ),
    ],
)
def test_guard_stands_down_for_unrelated_env_git_config_with_managed_hook(
    tmp_path, command
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert stderr == ""


@pytest.mark.parametrize(
    "args",
    [
        "--no-verify -m x",
        "-n -m x",
        "-nm x",
        "-qnm x",
    ],
)
def test_guard_runs_for_no_verify_even_with_dart_managed_hook(tmp_path, args):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git commit {args}")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_runs_for_env_split_no_verify_even_with_dart_managed_hook(
    tmp_path,
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "env -S 'git commit --no-verify -m x'")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


def test_guard_runs_for_path_git_no_verify_even_with_dart_managed_hook(tmp_path):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, "/usr/bin/git commit --no-verify -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "args",
    [
        "-qm x",
        "-uno -m x",
    ],
)
def test_guard_stands_down_for_combined_flags_without_no_verify(tmp_path, args):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git commit {args}")

    assert returncode == 0
    assert stderr == ""


@pytest.mark.parametrize(
    "option",
    [
        "-c core.hooksPath=/tmp/empty",
        "-c core.hookspath=/tmp/empty",
        "-c Core.HOOKSPATH=/tmp/empty",
    ],
)
def test_guard_runs_for_core_hookspath_override_even_with_dart_managed_hook(
    tmp_path, option
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, f"git {option} commit -m x")

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "option",
    [
        "--config-env=core.hooksPath=EMPTY_HOOKS",
        "--config-env=core.hookspath=EMPTY_HOOKS",
        "--config-env=Core.HOOKSPATH=EMPTY_HOOKS",
        "--config-env core.hooksPath=EMPTY_HOOKS",
        "--config-env core.hookspath=EMPTY_HOOKS",
        "--config-env Core.HOOKSPATH=EMPTY_HOOKS",
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
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command",
    [
        (
            "GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=core.hooksPath "
            "GIT_CONFIG_VALUE_0=/tmp/empty-hooks git commit -m x"
        ),
        (
            "GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=Core.HOOKSPATH "
            "GIT_CONFIG_VALUE_0=/tmp/empty-hooks git commit -m x"
        ),
        (
            "GIT_CONFIG_COUNT=2 GIT_CONFIG_KEY_0=user.name "
            "GIT_CONFIG_VALUE_0=DART GIT_CONFIG_KEY_1=core.hooksPath "
            "GIT_CONFIG_VALUE_1=/tmp/empty-hooks git commit -m x"
        ),
        (
            "env GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=core.hooksPath "
            "GIT_CONFIG_VALUE_0=/tmp/empty-hooks git commit -m x"
        ),
        (
            "env -S 'GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=core.hooksPath "
            "GIT_CONFIG_VALUE_0=/tmp/empty-hooks git commit -m x'"
        ),
    ],
)
def test_guard_runs_for_env_hookspath_override_even_with_dart_managed_hook(
    tmp_path, command
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr


@pytest.mark.parametrize(
    "command_template",
    [
        "GIT_CONFIG_GLOBAL={config} git commit -m x",
        "GIT_CONFIG_SYSTEM={config} git commit -m x",
        "env GIT_CONFIG_GLOBAL={config} git commit -m x",
        "env -S 'GIT_CONFIG_GLOBAL={config} git commit -m x'",
        "HOME={home} git commit -m x",
        "env HOME={home} git commit -m x",
        "XDG_CONFIG_HOME={xdg} git commit -m x",
    ],
)
def test_guard_runs_for_config_file_env_even_with_dart_managed_hook(
    tmp_path, command_template
):
    repo, env = _init_repo(tmp_path)
    assert _install(repo, env).returncode == 0
    config = tmp_path / "gitconfig"
    config.write_text("[core]\n\thooksPath = /tmp/empty-hooks\n")
    home = tmp_path / "home"
    home.mkdir()
    (home / ".gitconfig").write_text(config.read_text())
    xdg = tmp_path / "xdg"
    (xdg / "git").mkdir(parents=True)
    (xdg / "git" / "config").write_text(config.read_text())
    env.update({"CLAUDE_PROJECT_DIR": str(repo), "DART_HOOK_DRY_RUN": "1"})

    command = command_template.format(config=config, home=home, xdg=xdg)
    returncode, stderr = _run_guard(repo, env, command)

    assert returncode == 0
    assert "would run 'python3 scripts/check_agent_hook.py --profile staged'" in stderr
