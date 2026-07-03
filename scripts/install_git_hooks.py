#!/usr/bin/env python3
"""Install DART's git ``pre-commit`` hook (the Tier-0 lint gate).

Idempotently writes ``<git-hooks-dir>/pre-commit`` so every ``git commit`` runs
``pixi run check-lint-quick`` first. Behaviour:

* The managed hook carries a sentinel line (``DART-MANAGED-HOOK``); re-running
  this installer detects it and rewrites the hook in place, so the command is
  safe to run any number of times.
* If a *foreign* (non-DART) ``pre-commit`` hook already exists it is preserved,
  not clobbered: it is moved to ``pre-commit.local`` and chained from the
  managed hook so its checks still run before the lint gate. If a
  ``pre-commit.local`` is already present the installer refuses with a clear
  message rather than lose an existing local hook.
* Worktrees are handled via ``git rev-parse --git-path hooks``, which resolves
  to the shared common hooks directory, so a single install covers all linked
  worktrees of the repository.
* Emergency bypass at commit time: ``DART_SKIP_HOOKS=1 git commit ...``.
* Verification aid: ``DART_HOOK_DRY_RUN=1`` makes the installed hook print the
  command it *would* run instead of running it (used by CI/tests to confirm
  wiring without invoking the full lint).

Runnable with plain ``python3`` — no third-party imports.
"""

from __future__ import annotations

import shutil
import stat
import subprocess
import sys
from pathlib import Path

SENTINEL = "DART-MANAGED-HOOK"
HOOK_VERSION = "1"

# POSIX sh hook body. Kept dependency-free; only assumes ``git`` and ``pixi`` are
# on PATH (both are already required to work in this repo).
HOOK_TEMPLATE = f"""\
#!/bin/sh
# DART pre-commit hook — installed by scripts/install_git_hooks.py
# {SENTINEL} v{HOOK_VERSION}  (sentinel line: do not edit; the installer keys on it)
#
# Runs the Tier-0 lint gate (`pixi run check-lint-quick`) before every commit.
# Emergency bypass: DART_SKIP_HOOKS=1 git commit ...

if [ "${{DART_SKIP_HOOKS:-0}}" = "1" ]; then
    echo "DART pre-commit: skipped (DART_SKIP_HOOKS=1)" >&2
    exit 0
fi

if [ -n "${{DART_HOOK_DRY_RUN:-}}" ]; then
    echo "DART pre-commit (dry run): would run 'pixi run check-lint-quick'" >&2
    exit 0
fi

# Chain to a foreign hook preserved at install time, if any.
hooks_dir=$(git rev-parse --git-path hooks)
if [ -x "$hooks_dir/pre-commit.local" ]; then
    "$hooks_dir/pre-commit.local" "$@" || exit $?
fi

repo_root=$(git rev-parse --show-toplevel) || exit 1
cd "$repo_root" || exit 1

echo "DART pre-commit: running Tier-0 lint gate (pixi run check-lint-quick)..." >&2
if ! pixi run check-lint-quick; then
    echo "" >&2
    echo "DART pre-commit: check-lint-quick FAILED — commit blocked." >&2
    echo "  Fix with: pixi run lint   (then re-stage and commit)" >&2
    echo "  Emergency bypass: DART_SKIP_HOOKS=1 git commit ..." >&2
    exit 1
fi
"""


def run_git(args: list[str]) -> str:
    """Run a git command from the current directory and return stripped stdout."""
    result = subprocess.run(
        ["git", *args],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        sys.exit(
            f"error: `git {' '.join(args)}` failed: {result.stderr.strip()}\n"
            "  (run this from inside the DART git repository)"
        )
    return result.stdout.strip()


def resolve_hooks_dir() -> Path:
    """Resolve the git hooks directory, honoring worktrees."""
    raw = Path(run_git(["rev-parse", "--git-path", "hooks"]))
    if not raw.is_absolute():
        raw = (Path.cwd() / raw).resolve()
    return raw


def write_hook(path: Path) -> None:
    path.write_text(HOOK_TEMPLATE)
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def main() -> int:
    hooks_dir = resolve_hooks_dir()
    hooks_dir.mkdir(parents=True, exist_ok=True)

    pre_commit = hooks_dir / "pre-commit"
    local = hooks_dir / "pre-commit.local"

    if pre_commit.exists():
        existing = pre_commit.read_text(errors="replace")
        if SENTINEL in existing:
            write_hook(pre_commit)
            print(
                f"DART pre-commit hook already installed; refreshed in place: {pre_commit}"
            )
            return 0

        # A foreign hook is present — preserve it rather than clobber.
        if local.exists():
            sys.exit(
                "error: refusing to overwrite an existing pre-commit hook.\n"
                f"  A foreign hook exists at {pre_commit} AND {local} is already\n"
                "  present, so the foreign hook cannot be backed up without loss.\n"
                "  Resolve manually: fold your hook logic into pre-commit.local,\n"
                "  remove pre-commit, then re-run `pixi run install-hooks`."
            )
        shutil.move(str(pre_commit), str(local))
        local.chmod(local.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        print(
            f"Preserved existing pre-commit hook as {local} (chained from the DART hook)."
        )

    write_hook(pre_commit)
    print(f"Installed DART pre-commit hook: {pre_commit}")
    print("  Runs `pixi run check-lint-quick` before every commit.")
    print("  Emergency bypass: DART_SKIP_HOOKS=1 git commit ...")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
