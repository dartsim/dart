#!/usr/bin/env python3
"""Build and run C++ unit tests with optional component filtering."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

COMPONENT_PREFIXES = {
    "simd": "UNIT_simd",
    "math": "UNIT_math",
    "common": "UNIT_common",
    "dynamics": "UNIT_dynamics",
    "collision": "UNIT_collision",
    "constraint": "UNIT_constraint",
    "utils": "UNIT_utils",
    "io": "UNIT_io",
    "gui": "UNIT_gui",
}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "component",
        nargs="?",
        default=None,
        help="Component to test (e.g., simd, math, dynamics). If omitted, runs all unit tests.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type (default: Release)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Verbose ctest output"
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available components",
    )
    return parser.parse_args(argv)


def _print_components() -> None:
    print("Available test components:")
    for name, prefix in sorted(COMPONENT_PREFIXES.items()):
        print(f"  {name} -> ctest -R {prefix}")
    print("\nUsage: pixi run test-unit <component>")
    print("       pixi run test-unit simd")
    print("       pixi run test-unit math --verbose")


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return
    msg = f"Build directory {build_dir} does not exist.\nRun `pixi run build` first."
    raise SystemExit(msg)


def run(component: str | None, build_type: str, verbose: bool) -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type

    ensure_build_exists(build_dir, build_type)

    env = os.environ.copy()
    env["BUILD_TYPE"] = build_type
    env["CMAKE_BUILD_DIR"] = str(build_dir)

    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(build_dir),
            "--target",
            "tests",
        ],
        check=True,
        env=env,
    )

    ctest_cmd = [
        "ctest",
        "--test-dir",
        str(build_dir),
        "--output-on-failure",
    ]

    if verbose:
        ctest_cmd.append("-V")

    if component:
        prefix = COMPONENT_PREFIXES.get(component.lower(), f"UNIT_{component}")
        ctest_cmd.extend(["-R", prefix])

    result = subprocess.run(ctest_cmd, env=env)
    return result.returncode


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.list:
        _print_components()
        return 0

    return run(args.component, args.build_type, args.verbose)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
