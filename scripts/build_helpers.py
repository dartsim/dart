#!/usr/bin/env python3
"""
Shared helpers for pixi automation scripts.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Iterable


def _candidate_build_dirs(build_type: str) -> Iterable[Path]:
    """
    Yield potential build directories in preference order.
    """
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME") or "default"
    build_root = Path("build") / pixi_env / "cpp"
    yield build_root / build_type
    yield build_root


def get_build_dir(build_type: str) -> Path:
    """
    Return the most likely build directory for the current pixi environment.

    Prefers per-configuration subdirectories (Ninja single-config builds),
    but gracefully falls back to the generator root (Visual Studio multi-config).
    """
    for candidate in _candidate_build_dirs(build_type):
        if candidate.is_dir():
            if (candidate / "build.ninja").is_file():
                return candidate
            # Multi-config generators place build.ninja only at the root; make sure
            # we don't select the configuration subdirectory in that case.
            if not (candidate.parent / "build.ninja").is_file():
                return candidate
    # If nothing exists yet, return the first candidate so CMake can create it.
    return next(_candidate_build_dirs(build_type))


def run_cmake_build(build_dir: Path, build_type: str, target: str):
    """
    Invoke `cmake --build` for the provided target.
    """
    cmd = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        build_type,
        "--parallel",
        "--target",
        target,
    ]
    subprocess.check_call(cmd)


def ninja_target_exists(build_dir: Path, target: str) -> bool:
    """
    Detect whether the provided Ninja build directory contains a target.

    For non-Ninja generators, the build.ninja file does not exist, so we assume
    the target is present and let CMake perform the actual build.
    """
    ninja_file = build_dir / "build.ninja"
    if not ninja_file.is_file():
        return True

    pattern = f"build {target}:"
    with ninja_file.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if line.startswith(pattern):
                return True
    return False
