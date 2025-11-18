#!/usr/bin/env python3
"""Format TOML files while keeping backups."""
from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys
import time
from typing import Iterable

SKIP_DIRS = {".pixi", "build", "external", "node_modules"}


def find_toml_files(root: pathlib.Path) -> list[pathlib.Path]:
    files = []
    for path in root.rglob("*.toml"):
        if any(part in SKIP_DIRS for part in path.parts):
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


def main() -> int:
    root = pathlib.Path(__file__).resolve().parent.parent
    toml_files = find_toml_files(root)
    if not toml_files:
        return 0

    backup_files(toml_files, root)
    taplo_files = [str(root / path) for path in toml_files]
    subprocess.run(["taplo", "fmt", *taplo_files], check=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
