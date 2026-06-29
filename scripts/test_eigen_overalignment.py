#!/usr/bin/env python3
"""Build and test DART with Eigen forced to a larger static alignment.

This check catches allocator, placement-new, and storage code that accidentally
assumes Eigen-backed types will never require more than 16 or 32 byte alignment.
It does not require AVX-512 runner hardware because it forces Eigen's alignment
contract through compile definitions instead of enabling CPU instructions.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path

try:
    from parallel_jobs import compute_parallel_jobs
except ImportError:  # pragma: no cover

    def compute_parallel_jobs() -> int:
        return os.cpu_count() or 1


from cmake_host_linker_flags import cmake_host_linker_flags

ROOT_DIR = Path(__file__).resolve().parent.parent


def int_env(name: str, default: int, *, minimum: int, description: str) -> int:
    value = os.environ.get(name)
    if not value:
        return default
    try:
        parsed = int(value)
    except ValueError as exc:
        raise SystemExit(f"{name} must be {description}") from exc
    if parsed < minimum:
        raise SystemExit(f"{name} must be {description}")
    return parsed


def positive_int_env(name: str, default: int) -> int:
    return int_env(name, default, minimum=1, description="a positive integer")


def non_negative_int_env(name: str, default: int) -> int:
    return int_env(name, default, minimum=0, description="a non-negative integer")


def positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--alignment",
        type=positive_int,
        default=positive_int_env("DART_EIGEN_ALIGNMENT_BYTES", 64),
        help="Eigen max/static alignment in bytes.",
    )
    parser.add_argument(
        "--build-type",
        default=os.environ.get("BUILD_TYPE", "Release"),
        help="CMake build type for the alignment build.",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        help=(
            "Dedicated CMake build directory. Defaults to "
            "build/$PIXI_ENVIRONMENT_NAME/cpp/eigen<alignment>-align."
        ),
    )
    parser.add_argument(
        "--target",
        action="append",
        default=None,
        help="CMake target to build. Can be provided multiple times.",
    )
    parser.add_argument(
        "--ctest-regex",
        help="Optional ctest -R regex. By default, all built non-simulation tests run.",
    )
    parser.add_argument(
        "--skip-tests",
        action="store_true",
        help="Only configure and build the requested target(s).",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove the alignment build directory before configuring.",
    )
    return parser.parse_args()


def resolve_build_dir(args: argparse.Namespace) -> Path:
    if args.build_dir is not None:
        return args.build_dir

    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    return ROOT_DIR / "build" / pixi_env / "cpp" / f"eigen{args.alignment}-align"


def resolve_parallel() -> int:
    value = os.environ.get("DART_PARALLEL_JOBS")
    if value:
        try:
            return max(1, int(value))
        except ValueError:
            pass
    return compute_parallel_jobs()


def run(cmd: list[str], *, cwd: Path = ROOT_DIR) -> None:
    print("+ " + " ".join(str(part) for part in cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def cmake_cxx_flags(alignment: int) -> str:
    flags = os.environ.get("CMAKE_CXX_FLAGS", "").strip()
    alignment_flags = (
        f"-DEIGEN_MAX_ALIGN_BYTES={alignment} "
        f"-DEIGEN_MAX_STATIC_ALIGN_BYTES={alignment}"
    )
    return f"{flags} {alignment_flags}".strip()


def compiler_launcher_defs() -> list[str]:
    defs: list[str] = []
    for language in ("C", "CXX"):
        value = os.environ.get(f"CMAKE_{language}_COMPILER_LAUNCHER")
        if value:
            defs.append(f"-DCMAKE_{language}_COMPILER_LAUNCHER={value}")
    return defs


def configure(build_dir: Path, args: argparse.Namespace) -> None:
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if not conda_prefix:
        raise SystemExit("CONDA_PREFIX is required; run through pixi.")

    cmake_args = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        str(build_dir),
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={args.build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags(args.alignment)}",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_TESTS=ON",
        "-DDART_BUILD_EXAMPLES=OFF",
        "-DDART_BUILD_TUTORIALS=OFF",
        "-DDART_BUILD_GUI=OFF",
        "-DDART_BUILD_COLLISION_REFERENCE_TESTS=OFF",
        "-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF",
        "-DDART_BUILD_PROFILE=OFF",
        "-DDART_PROFILE_BUILTIN=OFF",
        "-DDART_PROFILE_TRACY=OFF",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_TRACY=ON",
        f"-DDART_VERBOSE={os.environ.get('DART_VERBOSE', 'OFF')}",
    ]
    cmake_args.extend(cmake_host_linker_flags())
    cmake_args.extend(compiler_launcher_defs())
    run(cmake_args)


def build(build_dir: Path, targets: list[str], parallel: int) -> None:
    for target in targets:
        run(
            [
                "cmake",
                "--build",
                str(build_dir),
                "--parallel",
                str(parallel),
                "--target",
                target,
            ]
        )


def test(build_dir: Path, args: argparse.Namespace, parallel: int) -> None:
    ctest_parallel = non_negative_int_env("CTEST_PARALLEL_LEVEL", parallel)
    cmd = [
        "ctest",
        "--test-dir",
        str(build_dir),
        "--output-on-failure",
        "--parallel",
        str(ctest_parallel),
        "--no-tests=error",
        "-LE",
        "simulation",
    ]
    if args.ctest_regex:
        cmd.extend(["-R", args.ctest_regex])
    run(cmd)


def main() -> int:
    args = parse_args()
    build_dir = resolve_build_dir(args)
    targets = args.target or ["tests"]
    parallel = resolve_parallel()

    if args.clean and build_dir.exists():
        shutil.rmtree(build_dir)

    configure(build_dir, args)
    build(build_dir, targets, parallel)
    if not args.skip_tests:
        test(build_dir, args, parallel)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
