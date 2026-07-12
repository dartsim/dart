#!/usr/bin/env python3
"""Fast, non-interactive pre-commit check for staged DART changes."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

from ai_infrastructure import (
    PROFILES,
    detect_profile,
    format_errors,
    is_ai_infrastructure_path,
    is_related_worktree_ai_path,
    repository_root,
    run_checks,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=("auto", "staged", *PROFILES),
        default="auto",
    )
    parser.add_argument("--root", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args()


def _git(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True
    )


def staged_paths(root: Path) -> list[str]:
    result = _git(
        root,
        "diff",
        "--cached",
        "--no-renames",
        "--name-only",
        "--diff-filter=ACMRD",
        "-z",
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "unable to inspect staged paths")
    return sorted(path for path in result.stdout.split("\0") if path)


def unstaged_paths(root: Path) -> list[str]:
    result = _git(
        root,
        "diff",
        "--no-renames",
        "--name-only",
        "--diff-filter=ACMRD",
        "-z",
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "unable to inspect unstaged paths")
    return sorted(path for path in result.stdout.split("\0") if path)


def untracked_paths(root: Path) -> list[str]:
    result = _git(root, "ls-files", "--others", "--exclude-standard", "-z")
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "unable to inspect untracked paths")
    return sorted(path for path in result.stdout.split("\0") if path)


@contextmanager
def staged_snapshot(root: Path) -> Iterator[Path]:
    """Materialize the Git index so checks cannot consume worktree-only files."""
    with tempfile.TemporaryDirectory(prefix="dart-agent-hook-") as directory:
        snapshot = Path(directory)
        result = _git(
            root,
            "checkout-index",
            "--all",
            "--force",
            "--prefix",
            f"{snapshot}/",
        )
        if result.returncode != 0:
            raise RuntimeError(
                result.stderr.strip() or "unable to materialize the staged snapshot"
            )
        yield snapshot


def main() -> int:
    args = parse_args()
    root = repository_root(args.root)
    whitespace = _git(root, "diff", "--cached", "--check")
    if whitespace.returncode != 0:
        print(whitespace.stdout or whitespace.stderr, end="")
        print("DART agent hook FAILED: staged diff check failed")
        return 1
    try:
        paths = staged_paths(root)
    except RuntimeError as exc:
        print(f"DART agent hook FAILED: {exc}")
        return 1
    relevant = [path for path in paths if is_ai_infrastructure_path(path)]
    if not relevant:
        print(
            f"DART agent hook passed ({len(paths)} staged paths; no AI-infra changes)"
        )
        return 0
    try:
        worktree_only = set(unstaged_paths(root)) | set(untracked_paths(root))
    except RuntimeError as exc:
        print(f"DART agent hook FAILED: {exc}")
        return 1
    unstaged_relevant = sorted(
        path for path in worktree_only if is_related_worktree_ai_path(root, path)
    )
    if unstaged_relevant:
        print(
            "DART agent hook FAILED: staged AI infrastructure cannot be "
            "validated while related infrastructure has unstaged or untracked "
            "changes:"
        )
        for path in unstaged_relevant:
            print(f"  {path}")
        print("Stage the intended versions, then retry the commit.")
        return 1
    requested_profile = "auto" if args.profile == "staged" else args.profile
    profile = detect_profile(root, requested_profile)
    try:
        with staged_snapshot(root) as snapshot:
            errors = run_checks(snapshot, profile)
    except RuntimeError as exc:
        print(f"DART agent hook FAILED: {exc}")
        return 1
    if errors:
        print(format_errors(errors))
        print(f"DART agent hook FAILED ({len(errors)} errors, profile={profile})")
        return 1
    print(
        f"DART agent hook passed ({len(paths)} staged paths, "
        f"{len(relevant)} AI-infra paths, profile={profile})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
