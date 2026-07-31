#!/usr/bin/env python3
"""Format CMake files with gersemi."""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys

# ``patches`` holds FreeBSD port patch files (unified diffs) whose names end in
# ``.cmake`` (e.g. tools/freebsd/patches/patch-cmake_DARTMacros.cmake). They are
# not CMake sources, so gersemi cannot parse them; skip the directory.
SKIP_DIRS = {
    ".deps",
    ".git",
    ".pixi",
    "build",
    "external",
    "node_modules",
    "patches",
}
GERSEMI_BATCH_SIZE = 100


def find_cmake_files(root: pathlib.Path) -> list[pathlib.Path]:
    # Only ``CMakeLists.txt`` and ``*.cmake`` are formatted. ``*.cmake.in``
    # configure templates are intentionally excluded: they embed configure-time
    # placeholders (e.g. ``@PACKAGE_INIT@`` in DARTConfig.cmake.in, which expands
    # to multiple statements) that gersemi cannot parse, and reflowing lines
    # around ``@VAR@`` tokens could corrupt the template before ``configure_file``
    # substitution runs.
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

    base_cmd = ["gersemi", "--check" if check else "-i"]
    for start in range(0, len(files), GERSEMI_BATCH_SIZE):
        batch = files[start : start + GERSEMI_BATCH_SIZE]
        cmd = base_cmd + [str(root / path) for path in batch]
        result = subprocess.run(cmd, cwd=root)
        if result.returncode != 0:
            return result.returncode

    return 0


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
