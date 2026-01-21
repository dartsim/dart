#!/usr/bin/env python3
"""Build and run SIMD benchmarks with drjit comparison."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    build_dir = Path("build") / env_name / "cpp" / "Release-simd-drjit"

    nanobind_dir = subprocess.check_output(
        [sys.executable, "-m", "nanobind", "--cmake_dir"], text=True
    ).strip()

    print("=== Configuring with DART_SIMD_BENCHMARK_DRJIT=ON (fetches drjit) ===")
    cmake_args = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        str(build_dir),
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_EXAMPLES=OFF",
        "-DDART_BUILD_TUTORIALS=OFF",
        "-DDART_BUILD_TESTS=ON",
        "-DDART_BUILD_GUI=OFF",
        "-DDART_BUILD_COLLISION_BULLET=OFF",
        "-DDART_BUILD_COLLISION_ODE=OFF",
        "-DDART_BUILD_SIMULATION_EXPERIMENTAL=OFF",
        "-DDART_SIMD_BENCHMARK_DRJIT=ON",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        f"-Dnanobind_DIR={nanobind_dir}",
    ]
    result = subprocess.run(cmake_args)
    if result.returncode != 0:
        return result.returncode

    print("\n=== Building SIMD benchmarks ===")
    build_args = [
        "cmake",
        "--build",
        str(build_dir),
        "--target",
        "bm_simd",
        "bm_simd_vs_drjit",
    ]
    result = subprocess.run(build_args)
    if result.returncode != 0:
        return result.returncode

    print("\n=== Running dart::simd benchmark ===")
    bm_simd = build_dir / "bin" / "bm_simd"
    result = subprocess.run([str(bm_simd)])
    if result.returncode != 0:
        return result.returncode

    print("\n=== Running dart::simd vs drjit comparison ===")
    bm_vs_drjit = build_dir / "bin" / "bm_simd_vs_drjit"
    result = subprocess.run([str(bm_vs_drjit)])
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
