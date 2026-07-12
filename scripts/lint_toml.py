#!/usr/bin/env python3
"""Format or check TOML files while keeping backups for write mode."""

from __future__ import annotations

import argparse
import pathlib
import shutil
import subprocess
import sys
import time
from typing import Iterable

SKIP_DIRS = {".git", ".pixi", "build", "external", "node_modules"}


def is_skipped_path(root: pathlib.Path, path: pathlib.Path) -> bool:
    relative = path.relative_to(root)
    return any(part in SKIP_DIRS for part in relative.parts) or relative.parts[:2] == (
        ".claude",
        "worktrees",
    )


def find_toml_files(root: pathlib.Path) -> list[pathlib.Path]:
    files = []
    for path in root.rglob("*.toml"):
        if is_skipped_path(root, path):
            continue
        files.append(path.relative_to(root))
    files.sort()
    return files


def backup_files(files: Iterable[pathlib.Path], root: pathlib.Path) -> None:
    timestamp = time.strftime("%Y%m%d%H%M%S")
    backup_dir = root / ".pixi" / "lint-toml-backups" / timestamp
    for path in files:
        src = root / path
        dest = backup_dir / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check TOML formatting without modifying files.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = pathlib.Path(__file__).resolve().parent.parent
    toml_files = find_toml_files(root)
    if not toml_files:
        return 0

    if not args.check:
        backup_files(toml_files, root)

    taplo_files = [str(root / path) for path in toml_files]
    command = ["taplo", "fmt"]
    if args.check:
        command.append("--check")
    command.extend(taplo_files)
    subprocess.run(command, check=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
