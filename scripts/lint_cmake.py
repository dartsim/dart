#!/usr/bin/env python3
"""Format CMake files with gersemi."""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys

SKIP_DIRS = {".deps", ".git", ".pixi", "build", "external", "node_modules"}


def find_cmake_files(root: pathlib.Path) -> list[pathlib.Path]:
    files = []
    for path in root.rglob("CMakeLists.txt"):
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        files.append(path.relative_to(root))
    for path in root.rglob("*.cmake"):
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        files.append(path.relative_to(root))
    files = sorted(set(files))
    return files


def run_gersemi(root: pathlib.Path, files: list[pathlib.Path], check: bool) -> int:
    if not files:
        return 0

    cmd = ["gersemi"]
    if check:
        cmd.append("--check")
    else:
        cmd.append("-i")
    cmd.extend(str(root / path) for path in files)
    result = subprocess.run(cmd, cwd=root)
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check formatting without applying changes.",
    )
    args = parser.parse_args()

    root = pathlib.Path(__file__).resolve().parent.parent
    cmake_files = find_cmake_files(root)
    return run_gersemi(root, cmake_files, args.check)


if __name__ == "__main__":
    sys.exit(main())
