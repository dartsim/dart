#!/usr/bin/env python3
"""Build and run C++ unit tests with optional component filtering."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

try:
    from parallel_jobs import compute_load_limit, compute_parallel_jobs
except ImportError:  # pragma: no cover - defensive fallback

    def compute_parallel_jobs() -> int:
        return max(os.cpu_count() or 1, 1)

    def compute_load_limit() -> int | None:
        return os.cpu_count() or 1


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
ALL_UNIT_TESTS_REGEX = "^UNIT_"

# Tests excluded from the `--quick` (Tier 0) inner loop because they dominate
# the wall-clock. They still run in the default and full tiers. Anchored so it
# matches only UNIT_simulation_World, not e.g. UNIT_simulation_WorldConfig.
QUICK_EXCLUDE_REGEX = "^UNIT_simulation_World$"

# Default per-test timeout (seconds). The slowest unit test is ~13s; a generous
# ceiling catches a genuinely hung test without flaking healthy ones.
DEFAULT_TEST_TIMEOUT = 300


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
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Tier-0 inner loop: skip the long-pole unit tests for fast feedback",
    )
    parser.add_argument(
        "--jobs",
        "-j",
        type=int,
        default=None,
        help="Parallel ctest jobs (default: shared parallelism policy)",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=DEFAULT_TEST_TIMEOUT,
        help=f"Per-test timeout in seconds (default: {DEFAULT_TEST_TIMEOUT})",
    )
    parser.add_argument(
        "--rerun-failed",
        action="store_true",
        help="Only re-run tests that failed in the previous run",
    )
    return parser.parse_args(argv)


def _print_components() -> None:
    print("Available test components:")
    for name, prefix in sorted(COMPONENT_PREFIXES.items()):
        print(f"  {name} -> ctest -R {prefix}")
    print(f"\nNo component -> ctest -R {ALL_UNIT_TESTS_REGEX}")
    print("\nUsage: pixi run test-unit <component>")
    print("       pixi run test-unit simd")
    print("       pixi run test-unit math --verbose")


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return
    msg = f"Build directory {build_dir} does not exist.\nRun `pixi run build` first."
    raise SystemExit(msg)


def run(args: argparse.Namespace) -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / args.build_type

    ensure_build_exists(build_dir, args.build_type)

    env = os.environ.copy()
    env["BUILD_TYPE"] = args.build_type
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

    if args.verbose:
        ctest_cmd.append("-V")

    # Parallel, load-aware test execution with a per-test timeout. CTEST_*
    # environment variables still override these defaults when a caller sets them.
    jobs = args.jobs if args.jobs and args.jobs > 0 else compute_parallel_jobs()
    ctest_cmd += ["--parallel", str(jobs)]
    load_limit = compute_load_limit()
    if load_limit:
        ctest_cmd += ["--test-load", str(load_limit)]
    if args.timeout and args.timeout > 0:
        ctest_cmd += ["--timeout", str(args.timeout)]
    # --rerun-failed needs a prior run's LastTestsFailed.log and ignores -R/-E
    # selection. Degrade gracefully to a normal run when there is nothing to
    # re-run, and warn that it overrides component scope.
    rerun = args.rerun_failed
    if rerun:
        last_failed = build_dir / "Testing" / "Temporary" / "LastTestsFailed.log"
        if not last_failed.is_file():
            print(
                "warning: --rerun-failed found no prior failures; "
                "running the full selection instead.",
                file=sys.stderr,
            )
            rerun = False
        elif args.component:
            print(
                f"warning: --rerun-failed ignores the '{args.component}' "
                "component scope; re-running all previously failed tests.",
                file=sys.stderr,
            )

    if rerun:
        ctest_cmd.append("--rerun-failed")
    else:
        test_regex = ALL_UNIT_TESTS_REGEX
        if args.component:
            test_regex = COMPONENT_PREFIXES.get(
                args.component.lower(), f"UNIT_{args.component}"
            )
        ctest_cmd.extend(["-R", test_regex])
        if args.quick:
            ctest_cmd.extend(["-E", QUICK_EXCLUDE_REGEX])

    result = subprocess.run(ctest_cmd, env=env)
    return result.returncode


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.list:
        _print_components()
        return 0

    return run(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
