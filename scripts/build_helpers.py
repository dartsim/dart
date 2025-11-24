#!/usr/bin/env python3
"""
Shared helpers for pixi automation scripts.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path
from typing import Iterable, Optional, Set


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

    def _has_build_system_files(path: Path) -> bool:
        return any(
            (path / marker).is_file() for marker in ("build.ninja", "CMakeCache.txt")
        )

    first_candidate: Optional[Path] = None

    for candidate in _candidate_build_dirs(build_type):
        if first_candidate is None:
            first_candidate = candidate
        if candidate.is_dir() and _has_build_system_files(candidate):
            return candidate

    # Fall back to the first existing candidate (even if not configured yet)
    for candidate in _candidate_build_dirs(build_type):
        if candidate.exists():
            return candidate

    # Nothing exists yet; return the preferred directory so CMake can create it.
    return first_candidate if first_candidate is not None else Path()


def run_cmake_build(build_dir: Path, build_type: str, target: str):
    """
    Invoke `cmake --build` for the provided target.
    """
    raw_parallel = (
        os.environ.get("DART_PARALLEL_JOBS")
        or os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL")
        or "1"
    ).strip()
    try:
        jobs = int(raw_parallel)
    except ValueError:
        jobs = 1
    jobs = max(jobs, 1)

    cmd = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        build_type,
        "--target",
        target,
    ]
    cmd.extend(["--parallel", str(jobs)])

    subprocess.check_call(cmd)


def _locate_cache(build_dir: Path) -> Optional[Path]:
    """
    Return the nearest CMakeCache.txt for the build directory.
    """
    for candidate in (build_dir, build_dir.parent):
        cache_path = candidate / "CMakeCache.txt"
        if cache_path.is_file():
            return cache_path
    return None


def _cmake_generator(build_dir: Path) -> Optional[str]:
    cache = _locate_cache(build_dir)
    if not cache:
        return None

    prefix = "CMAKE_GENERATOR:"
    with cache.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if line.startswith(prefix):
                parts = line.split("=", maxsplit=1)
                if len(parts) == 2:
                    return parts[1].strip()
    return None


def _ninja_target_exists(build_dir: Path, target: str) -> Optional[bool]:
    ninja_file = build_dir / "build.ninja"
    if not ninja_file.is_file():
        return None

    pattern = f"build {target}:"
    with ninja_file.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if line.startswith(pattern):
                return True
    return False


_HELP_SKIP_PREFIXES = (
    "The following",
    "Build command",
    "Build options",
    "For more information",
    'Use "cmake --build"',
)

_HELP_LINE_PATTERN = re.compile(r"^(?:\*|\.\.\.|[-•])?\s*([A-Za-z0-9_./:+-]+)")


def _cmake_help_targets(build_dir: Path, build_type: str) -> Optional[Set[str]]:
    cmd = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        build_type,
        "--target",
        "help",
    ]
    result = subprocess.run(
        cmd, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    if result.returncode != 0:
        return None

    targets: Set[str] = set()
    combined = f"{result.stdout}\n{result.stderr}"
    for line in combined.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith(_HELP_SKIP_PREFIXES):
            continue
        match = _HELP_LINE_PATTERN.match(stripped)
        if not match:
            continue
        name = match.group(1).rstrip(":")
        if name:
            targets.add(name)
    return targets


def cmake_target_exists(build_dir: Path, build_type: str, target: str) -> bool:
    """
    Detect whether the requested target exists for the configured generator.

    For Ninja, the lookup reads build.ninja directly. Other generators fall back
    to `cmake --build <dir> --target help`, which lists available targets without
    actually building anything.
    """
    generator = _cmake_generator(build_dir)
    if generator and "Ninja" in generator:
        ninja_result = _ninja_target_exists(build_dir, target)
        if ninja_result is not None:
            return ninja_result

    help_targets = _cmake_help_targets(build_dir, build_type)
    if help_targets is None:
        return False

    return target in help_targets
