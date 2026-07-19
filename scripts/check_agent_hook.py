#!/usr/bin/env python3
"""Run DART's fast commit-command and staged-file agent hook profiles.

The ``auto`` profile consumes a Codex/Claude PreToolUse JSON payload and
delegates commit detection to the release branch's well-tested POSIX guard.
On native Windows, ``scripts/pretool_guard_bridge.py`` locates Git Bash and
forwards the unchanged payload to that same guard. The ``staged`` profile is
the installed git-hook gate: it always runs
``git diff --cached --check`` and runs AI-infrastructure checks only when their
tracked inputs are staged. Neither profile configures, builds, or uses network.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

AI_PATH_PREFIXES = (
    ".agents/",
    ".claude/commands/",
    ".claude/hooks/",
    ".claude/settings.json",
    ".claude/skills/",
    ".codex/",
    ".opencode/command/",
    ".github/workflows/",
    ".gitignore",
    "AGENTS.md",
    "docs/",
    "pixi.toml",
    "python/tests/unit/gui/test_agent_capture.py",
    "python/tests/unit/gui/test_agent_debug_overlay.py",
    "scripts/check_agent_hook.py",
    "scripts/check_ai_infrastructure.py",
    "scripts/install_git_hooks.py",
    "scripts/pretool_guard_bridge.py",
    "scripts/setup_ai.py",
    "scripts/sync_ai_commands.py",
    "tests/test_ai_infrastructure.py",
    "tests/test_install_git_hooks.py",
    "tests/test_sync_ai_commands.py",
)


def repo_root() -> Path:
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
    )
    return Path(result.stdout.strip())


def is_ai_infrastructure_path(path: str) -> bool:
    """Return whether a staged path affects the deterministic AI checks."""
    return path.endswith("/AGENTS.md") or any(
        path == prefix or path.startswith(prefix) for prefix in AI_PATH_PREFIXES
    )


def is_related_worktree_ai_path(root: Path, path: str) -> bool:
    """Exclude unrelated personal/plugin skills from staged consistency checks."""
    if not path.startswith(".agents/skills/"):
        return is_ai_infrastructure_path(path)
    parts = Path(path).parts
    if len(parts) < 3 or parts[2] == ".dart-generated.json":
        return True
    skill_name = parts[2]
    if (root / ".claude" / "commands" / f"{skill_name}.md").is_file() or (
        root / ".claude" / "skills" / skill_name / "SKILL.md"
    ).is_file():
        return True
    manifest_path = root / ".agents" / "skills" / ".dart-generated.json"
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError):
        manifest = {}
    owned_paths = manifest.get("paths", []) if isinstance(manifest, dict) else []
    if isinstance(owned_paths, list) and f"{skill_name}/SKILL.md" in owned_paths:
        return True
    skill_path = root / ".agents" / "skills" / skill_name / "SKILL.md"
    try:
        content = skill_path.read_text()
    except OSError:
        return False
    return "<!-- AUTO-GENERATED FILE" in content and (
        "<!-- Sync script: scripts/sync_ai_commands.py -->" in content
    )


def run_auto(root: Path) -> int:
    guard = root / ".claude" / "hooks" / "pre-commit-guard.sh"
    if not guard.is_file():
        print(f"DART hook: missing {guard}", file=sys.stderr)
        return 2

    env = dict(os.environ)
    env["CLAUDE_PROJECT_DIR"] = str(root)
    env["CODEX_PROJECT_DIR"] = str(root)
    env["DART_HOOK_PYTHON"] = str(Path(sys.executable).resolve()).replace("\\", "/")
    result = subprocess.run(
        [str(guard)],
        input=sys.stdin.buffer.read(),
        cwd=root,
        env=env,
    )
    return result.returncode


def staged_paths(root: Path) -> list[str]:
    result = subprocess.run(
        [
            "git",
            "diff",
            "--cached",
            "--no-renames",
            "--name-only",
            "--diff-filter=ACMRD",
        ],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line]


def unstaged_paths(root: Path) -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--no-renames", "--name-only"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line]


def untracked_paths(root: Path) -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line]


@contextmanager
def staged_snapshot(root: Path) -> Iterator[Path]:
    """Materialize the Git index so checks cannot consume worktree-only files."""
    with tempfile.TemporaryDirectory(prefix="dart-agent-hook-") as directory:
        snapshot = Path(directory)
        result = subprocess.run(
            [
                "git",
                "checkout-index",
                "--all",
                "--force",
                "--prefix",
                f"{snapshot}/",
            ],
            cwd=root,
            capture_output=True,
            text=True,
        )
        if result.returncode:
            raise RuntimeError(
                result.stderr.strip() or "unable to materialize the staged snapshot"
            )
        yield snapshot


def run_checked(command: list[str], root: Path) -> int:
    result = subprocess.run(command, cwd=root)
    return result.returncode


def run_staged(root: Path) -> int:
    diff_check = subprocess.run(
        ["git", "diff", "--cached", "--check"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    if diff_check.returncode != 0:
        sys.stderr.write(diff_check.stdout)
        sys.stderr.write(diff_check.stderr)
        return diff_check.returncode

    paths = staged_paths(root)
    relevant = {path for path in paths if is_ai_infrastructure_path(path)}
    if not relevant:
        return 0

    worktree_only = set(unstaged_paths(root)) | set(untracked_paths(root))
    unstaged_relevant = {
        path for path in worktree_only if is_related_worktree_ai_path(root, path)
    }
    if unstaged_relevant:
        print(
            "DART staged hook: staged AI infrastructure cannot be validated "
            "while related infrastructure also has unstaged changes; stage the "
            "intended final content first:",
            file=sys.stderr,
        )
        for path in sorted(unstaged_relevant):
            print(f"  {path}", file=sys.stderr)
        return 2

    try:
        with staged_snapshot(root) as snapshot:
            checks = (
                [sys.executable, "scripts/sync_ai_commands.py", "--check"],
                [sys.executable, "scripts/check_ai_infrastructure.py", "--check"],
            )
            for command in checks:
                returncode = run_checked(command, snapshot)
                if returncode:
                    return returncode
    except RuntimeError as error:
        print(f"DART staged hook: {error}", file=sys.stderr)
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("auto", "staged"), default="staged")
    parser.add_argument("--repo-root", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()

    root = args.repo_root.resolve() if args.repo_root else repo_root()
    if args.profile == "auto":
        return run_auto(root)
    return run_staged(root)


if __name__ == "__main__":
    raise SystemExit(main())
