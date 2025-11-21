#!/usr/bin/env python3
"""Helper to invoke cmake --build with consistent options."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

try:
    from parallel_jobs import compute_parallel_jobs
except ImportError:  # pragma: no cover

    def compute_parallel_jobs() -> int:
        cpus = os.cpu_count() or 1
        if cpus <= 8:
            jobs = cpus
        else:
            jobs = max(1, (cpus * 3) // 4)
        # Avoid exhausting process limits on beefy self-hosted runners.
        return min(jobs, 32)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-dir",
        help=(
            "Path to the CMake build tree. When omitted, falls back to "
            "CMAKE_BUILD_DIR or build/$PIXI_ENVIRONMENT_NAME/cpp/$BUILD_TYPE."
        ),
    )
    parser.add_argument(
        "--config",
        help="Multi-config generator configuration (e.g., Release)",
    )
    parser.add_argument(
        "--parallel",
        type=int,
        help="Override the parallel job count",
    )
    parser.add_argument(
        "--target",
        action="append",
        default=[],
        help="Target(s) to build. Can be provided multiple times.",
    )
    return parser.parse_args()


def resolve_parallel(explicit: int | None) -> int:
    if explicit:
        return explicit
    env_value = os.environ.get("DART_PARALLEL_JOBS")
    if env_value:
        try:
            return int(env_value)
        except ValueError:
            pass
    return compute_parallel_jobs()


def resolve_build_dir(value: str | None) -> Path:
    """Determine the build directory from CLI args or environment."""
    if value:
        return Path(value)
    env_default = os.environ.get("CMAKE_BUILD_DIR")
    if env_default:
        return Path(env_default)
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME")
    build_type = os.environ.get("BUILD_TYPE")
    if pixi_env and build_type:
        return Path("build") / pixi_env / "cpp" / build_type
    raise SystemExit(
        "--build-dir is required. Provide it via CLI, set CMAKE_BUILD_DIR, "
        "or ensure PIXI_ENVIRONMENT_NAME and BUILD_TYPE are defined."
    )


def run_build(
    build_dir: Path, config: str | None, parallel: int, target: str | None
) -> None:
    cmd = ["cmake", "--build", str(build_dir)]
    if config:
        cmd += ["--config", config]
    cmd += ["--parallel", str(parallel)]
    if target:
        cmd += ["--target", target]
    subprocess.run(cmd, check=True)


def main() -> int:
    args = parse_args()
    build_dir = resolve_build_dir(args.build_dir)
    parallel = resolve_parallel(args.parallel)
    targets = args.target or [None]
    for target in targets:
        run_build(build_dir, args.config, parallel, target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
