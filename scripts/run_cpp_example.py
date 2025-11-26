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

    binary = build_dir / "bin" / target
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
