#!/usr/bin/env python3
"""Check that legacy collision engines stay out of runtime source paths."""

from __future__ import annotations

import re
import sys
from pathlib import Path

SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".ipp"}
LEGACY_ENGINES = ("fcl", "bullet", "ode")
PUBLIC_FACADE_HEADERS = {
    "bullet": {
        "All.hpp",
        "BulletCollisionDetector.hpp",
        "BulletCollisionGroup.hpp",
        "bullet.hpp",
        "bullet_collision_detector.hpp",
        "bullet_collision_group.hpp",
    },
    "fcl": {
        "All.hpp",
        "FCLCollisionDetector.hpp",
        "FCLCollisionGroup.hpp",
        "fcl.hpp",
        "fcl_collision_detector.hpp",
        "fcl_collision_group.hpp",
    },
    "ode": {
        "All.hpp",
        "OdeCollisionDetector.hpp",
        "OdeCollisionGroup.hpp",
        "ode.hpp",
        "ode_collision_detector.hpp",
        "ode_collision_group.hpp",
    },
}
REFERENCE_PREFIXES = tuple(
    f"dart/collision/{engine}/reference/" for engine in LEGACY_ENGINES
)
COMPAT_PREFIXES = tuple(f"dart/collision/{engine}/compat/" for engine in LEGACY_ENGINES)
FORBIDDEN_DART_REFERENCE_INCLUDES = tuple(
    f"dart/collision/{engine}/reference/" for engine in LEGACY_ENGINES
)
TEST_REFERENCE_INCLUDE_PREFIX = "dart/test/reference_collision/"
FORBIDDEN_EXTERNAL_INCLUDE_PREFIXES = (
    "fcl/",
    "ode/",
    "ccd/",
    "BulletCollision/",
    "BulletDynamics/",
    "BulletSoftBody/",
    "LinearMath/",
    "btBullet",
)

INCLUDE_PATTERN = re.compile(r"^\s*#\s*include\s*[<\"]([^>\"]+)[>\"]")
SKIPPED_SCAN_DIRS = {
    ".agents",
    ".claude",
    ".codex",
    ".git",
    ".opencode",
    ".pixi",
    ".pytest_cache",
    "__pycache__",
    "build",
    "dist",
    "install",
}


def is_source_file(path: Path) -> bool:
    return path.suffix.lower() in SOURCE_SUFFIXES


def is_reference_path(relative_path: str) -> bool:
    return relative_path.startswith(REFERENCE_PREFIXES)


def is_compat_path(relative_path: str) -> bool:
    return relative_path.startswith(COMPAT_PREFIXES)


def should_skip_path(path: Path) -> bool:
    return any(part in SKIPPED_SCAN_DIRS for part in path.parts)


def check_forbidden_sources(repo_root: Path) -> list[str]:
    failures: list[str] = []
    for engine in LEGACY_ENGINES:
        engine_dir = repo_root / "dart" / "collision" / engine
        if not engine_dir.exists():
            continue
        for path in sorted(engine_dir.rglob("*")):
            if not path.is_file() or path.suffix.lower() not in {".cc", ".cpp", ".cxx"}:
                continue
            relative_path = path.relative_to(repo_root).as_posix()
            if is_reference_path(relative_path):
                failures.append(
                    f"{relative_path}: reference collision implementation source "
                    "must live under tests/dart/test/reference_collision/"
                )
            elif not is_compat_path(relative_path):
                failures.append(
                    f"{relative_path}: legacy engine implementation source must "
                    "live under compat/"
                )
    return failures


def check_public_facade_headers(repo_root: Path) -> list[str]:
    failures: list[str] = []
    for engine in LEGACY_ENGINES:
        engine_dir = repo_root / "dart" / "collision" / engine
        if not engine_dir.exists():
            continue

        allowed_headers = PUBLIC_FACADE_HEADERS[engine]
        allowed_include_prefix = f"dart/collision/{engine}/compat/"
        for path in sorted(engine_dir.iterdir()):
            if not path.is_file() or not is_source_file(path):
                continue

            relative_path = path.relative_to(repo_root).as_posix()
            if path.name not in allowed_headers:
                failures.append(
                    f"{relative_path}: public legacy collision source must be "
                    "an explicit forwarding facade or move under compat/"
                )
                continue

            for line_number, line in enumerate(
                path.read_text(encoding="utf-8", errors="replace").splitlines(),
                start=1,
            ):
                stripped = line.strip()
                if (
                    not stripped
                    or stripped.startswith("//")
                    or stripped == "#pragma once"
                ):
                    continue

                match = INCLUDE_PATTERN.match(line)
                if match and match.group(1).startswith(allowed_include_prefix):
                    continue

                failures.append(
                    f"{relative_path}:{line_number}: public legacy collision "
                    "header must only forward to the native-backed compat "
                    f"facade under {allowed_include_prefix}"
                )
    return failures


def check_forbidden_includes(repo_root: Path) -> list[str]:
    failures: list[str] = []
    for path in sorted((repo_root / "dart").rglob("*")):
        if not path.is_file() or not is_source_file(path):
            continue
        relative_path = path.relative_to(repo_root).as_posix()
        if is_reference_path(relative_path):
            continue

        for line_number, line in enumerate(
            path.read_text(encoding="utf-8", errors="replace").splitlines(),
            start=1,
        ):
            match = INCLUDE_PATTERN.match(line)
            if not match:
                continue
            include = match.group(1)
            if include.startswith(FORBIDDEN_DART_REFERENCE_INCLUDES):
                failures.append(
                    f"{relative_path}:{line_number}: runtime source includes "
                    f"reference backend header {include}"
                )
            if include.startswith(FORBIDDEN_EXTERNAL_INCLUDE_PREFIXES):
                failures.append(
                    f"{relative_path}:{line_number}: runtime source includes "
                    f"legacy collision dependency header {include}"
                )

    for path in sorted(repo_root.rglob("*")):
        if not path.is_file() or not is_source_file(path) or should_skip_path(path):
            continue
        relative_path = path.relative_to(repo_root).as_posix()
        if relative_path.startswith("tests/"):
            continue

        for line_number, line in enumerate(
            path.read_text(encoding="utf-8", errors="replace").splitlines(),
            start=1,
        ):
            match = INCLUDE_PATTERN.match(line)
            if not match:
                continue
            include = match.group(1)
            if include.startswith(FORBIDDEN_DART_REFERENCE_INCLUDES):
                failures.append(
                    f"{relative_path}:{line_number}: non-test source includes "
                    f"old reference backend header {include}"
                )
            if include.startswith(TEST_REFERENCE_INCLUDE_PREFIX):
                failures.append(
                    f"{relative_path}:{line_number}: non-test source includes "
                    f"test-only reference backend header {include}"
                )
    return failures


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    failures = check_forbidden_sources(repo_root)
    failures.extend(check_public_facade_headers(repo_root))
    failures.extend(check_forbidden_includes(repo_root))

    if failures:
        print("Collision runtime isolation check failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("Collision runtime isolation check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
