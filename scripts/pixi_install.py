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
import sys
from typing import List

from build_helpers import get_build_dir, ninja_target_exists, run_cmake_build


def main(argv: List[str]) -> int:
    dartpy_flag = argv[1] if len(argv) > 1 else "ON"
    build_type = os.environ.get("BUILD_TYPE", "Release")
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME") or "default"

    build_dir = get_build_dir(build_type)

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
