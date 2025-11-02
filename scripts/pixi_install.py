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

    build_dir = os.path.join("build", pixi_env, "cpp")

    if dartpy_flag.upper() != "OFF":
        run_cmake_build(build_dir, build_type, "dartpy")

    run_cmake_build(build_dir, build_type, "install")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
