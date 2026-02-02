#!/usr/bin/env python3
"""Build and run a C++ example with optional runtime arguments."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description=__doc__,
        add_help=False,  # Allow --help to pass through to the example.
    )
    parser.add_argument(
        "target",
        nargs="?",
        default=None,
        help="CMake target / example binary name (e.g., atlas_simbicon)",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use (default: Release)",
    )
    parser.add_argument(
        "--pixi-help",
        action="store_true",
        help="Show this help for the pixi wrapper.",
    )
    known, unknown = parser.parse_known_args(argv)

    if known.pixi_help or known.target is None:
        parser.print_help()
        sys.exit(0)

    return known, unknown


def _normalize_target(target: str) -> str:
    if target == "raylib_gui":
        print("NOTE: example `raylib_gui` was renamed to `raylib`.", file=sys.stderr)
        return "raylib"
    return target


def _is_raylib_target(target: str) -> bool:
    """Return True if *target* is a Raylib-backed example."""
    return target in {"raylib", "dart_raylib"} or target.startswith("raylib_")


def _resolve_build_and_binary(target: str) -> tuple[str, str]:
    if target in {"raylib", "dart_raylib"}:
        return "dart_raylib", "raylib"
    if target.startswith("raylib_"):
        return f"dart_{target}", target
    return target, target


def _cmake_cache_bool(build_dir: Path, option: str) -> bool | None:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        return None

    needle = f"{option}:BOOL="
    with cache_path.open("r", encoding="utf-8", errors="ignore") as cache:
        for line in cache:
            if not line.startswith(needle):
                continue
            value = line.strip().split("=", maxsplit=1)[-1].upper()
            if value in {"ON", "TRUE", "1"}:
                return True
            if value in {"OFF", "FALSE", "0"}:
                return False
            return None
    return None


def _ensure_target_requirements(
    build_dir: Path, target: str, env: dict[str, str]
) -> None:
    if not _is_raylib_target(target):
        return

    enabled = _cmake_cache_bool(build_dir, "DART_BUILD_GUI_RAYLIB")
    if enabled:
        return

    print(
        "Enabling experimental Raylib backend (DART_BUILD_GUI_RAYLIB=ON) for this build...",
        file=sys.stderr,
    )
    subprocess.run(
        [
            "cmake",
            "-S",
            ".",
            "-B",
            str(build_dir),
            "-DDART_BUILD_GUI_RAYLIB=ON",
        ],
        check=True,
        env=env,
    )


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return

    msg = (
        f"Build directory {build_dir} does not exist.\n"
        f"Run `pixi run config --build_type {build_type}` first, "
        "then re-run this command."
    )
    raise SystemExit(msg)


def run(target: str, build_type: str, run_args: list[str]) -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type

    ensure_build_exists(build_dir, build_type)

    target = _normalize_target(target)
    build_target, binary_name = _resolve_build_and_binary(target)

    env = os.environ.copy()
    env["BUILD_TYPE"] = build_type
    env["CMAKE_BUILD_DIR"] = str(build_dir)

    _ensure_target_requirements(build_dir, build_target, env)

    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(build_dir),
            "--target",
            build_target,
        ],
        check=True,
        env=env,
    )

    binary = build_dir / "bin" / binary_name
    if not binary.exists():
        raise SystemExit(f"Binary not found: {binary}")

    cmd = [str(binary), *run_args]
    subprocess.run(cmd, check=True, env=env)
    return 0


def main(argv: list[str]) -> int:
    args, run_args = parse_args(argv)
    return run(args.target, args.build_type, run_args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
