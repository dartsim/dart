#!/usr/bin/env python3
"""
Helper invoked by pixi tasks to build optional targets before installation.

This script mirrors the install logic used in CI workflows, ensuring we build
the configured `dartpy` target (when enabled) prior to running `cmake --build`
for installation. Keeping this logic in Python avoids shell-syntax nuances
across the various shells pixi may use (bash, PowerShell, deno_task_shell).
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import List


def run_cmake_build(build_dir: str, build_type: str, target: str):
    """Invoke `cmake --build` for the requested target."""
    cmd: List[str] = [
        "cmake",
        "--build",
        build_dir,
        "--config",
        build_type,
        "--parallel",
        "--target",
        target,
    ]
    subprocess.check_call(cmd)


def main(argv: List[str]) -> int:
    dartpy_flag = argv[1] if len(argv) > 1 else "ON"
    build_type = os.environ.get("BUILD_TYPE", "Release")
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME") or "default"

    # Windows multi-config generators place configuration under a sub-folder.
    build_dir = os.path.join("build", pixi_env, "cpp", build_type)
    if not os.path.isdir(build_dir):
        # Fallback to flat layout (Unix single-config builds).
        build_dir = os.path.join("build", pixi_env, "cpp")

    if dartpy_flag.upper() != "OFF":
        if ninja_target_exists(build_dir, "dartpy"):
            run_cmake_build(build_dir, build_type, "dartpy")
        else:
            print(
                "Skipping dartpy build during install because the 'dartpy' target "
                "was not generated in this configuration."
            )

    run_cmake_build(build_dir, build_type, "install")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))


def ninja_target_exists(build_dir: str, target: str) -> bool:
    """Detect whether the build.ninja file contains a target."""
    ninja_file = Path(build_dir) / "build.ninja"
    if not ninja_file.is_file():
        # Not a Ninja generator (e.g., Visual Studio); assume target exists.
        return True

    pattern = f"build {target}:"
    with ninja_file.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if line.startswith(pattern):
                return True
    return False
