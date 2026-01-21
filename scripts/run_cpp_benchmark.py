#!/usr/bin/env python3
"""Build and run a C++ benchmark with optional runtime arguments."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

CANONICAL_BENCHMARKS = {
    "boxes": "bm_boxes",
    "kinematics": "bm_kinematics",
    "lcp_compare": "BM_LCP_COMPARE",
    "lcp_solver": "BM_LCPSOLVER",
    "lcp_solver_solvers": "BM_LCPSOLVER_SOLVERS",
    "row_swapping": "BM_ROW_SWAPPING",
    "dot_product": "BM_DOT_PRODUCT",
    "matrix_multiply": "BM_MATRIX_MULTIPLY",
    "simd": "bm_simd",
}

ALIASES = {
    **CANONICAL_BENCHMARKS,
    "bm_boxes": "bm_boxes",
    "bm_kinematics": "bm_kinematics",
    "bm_lcp_compare": "BM_LCP_COMPARE",
    "bm_lcpsolver": "BM_LCPSOLVER",
    "bm_lcpsolver_solvers": "BM_LCPSOLVER_SOLVERS",
    "bm_row_swapping": "BM_ROW_SWAPPING",
    "bm_dot_product": "BM_DOT_PRODUCT",
    "bm_matrix_multiply": "BM_MATRIX_MULTIPLY",
    "lcpsolver": "BM_LCPSOLVER",
    "lcp_solvers": "BM_LCPSOLVER_SOLVERS",
    "bm_simd": "bm_simd",
}


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description=__doc__,
        add_help=False,  # Allow --help to pass through to the benchmark.
    )
    parser.add_argument(
        "benchmark",
        nargs="?",
        default=None,
        help="Benchmark alias or CMake target (e.g., lcp_compare or BM_LCP_COMPARE)",
    )
    parser.add_argument(
        "--target",
        dest="benchmark",
        help="Alias for benchmark target name (legacy pixi usage).",
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

    if known.pixi_help or known.benchmark is None:
        parser.print_help()
        _print_known_benchmarks()
        sys.exit(0)

    return known, unknown


def _print_known_benchmarks() -> None:
    print("\nCommon benchmarks (alias -> CMake target):")
    for alias, target in sorted(CANONICAL_BENCHMARKS.items()):
        print(f"  {alias} -> {target}")
    print("  (pass --help through to the benchmark after `--`)")


def _normalize_key(name: str) -> str:
    return name.strip().lower().replace("-", "_")


def _resolve_target(benchmark: str) -> str:
    key = _normalize_key(benchmark)
    return ALIASES.get(key, benchmark)


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return

    msg = (
        f"Build directory {build_dir} does not exist.\n"
        f"Run `pixi run config --build_type {build_type}` first, "
        "then re-run this command."
    )
    raise SystemExit(msg)


def _find_binary(build_dir: Path, target: str) -> Path:
    candidates = [
        build_dir / "bin" / target,
        build_dir / "tests" / "benchmark" / target,
        build_dir / "tests" / "benchmark" / "lcpsolver" / target,
        build_dir / "tests" / "benchmark" / "collision" / target,
        build_dir / "tests" / "benchmark" / "dynamics" / target,
        build_dir / "tests" / "benchmark" / "integration" / target,
        build_dir / "tests" / "benchmark" / "unit" / target,
        build_dir / "tests" / "benchmark" / "simd" / target,
    ]

    for path in candidates:
        if path.exists():
            return path

    raise SystemExit(
        f"Binary not found for target '{target}'. "
        "Check the build output for the runtime directory."
    )


def run(benchmark: str, build_type: str, run_args: list[str]) -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type

    ensure_build_exists(build_dir, build_type)

    target = _resolve_target(benchmark)

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
            target,
        ],
        check=True,
        env=env,
    )

    binary = _find_binary(build_dir, target)
    subprocess.run([str(binary), *run_args], check=True, env=env)
    return 0


def main(argv: list[str]) -> int:
    args, run_args = parse_args(argv)
    return run(args.benchmark, args.build_type, run_args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
