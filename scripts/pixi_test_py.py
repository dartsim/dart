#!/usr/bin/env python3
"""
Helper invoked by pixi to run python tests only when the pytest target exists.
"""

from __future__ import annotations

import os
import sys

from build_helpers import get_build_dir, ninja_target_exists, run_cmake_build


def main() -> int:
    build_type = os.environ.get("BUILD_TYPE", "Release")
    build_dir = get_build_dir(build_type)

    if not ninja_target_exists(build_dir, "pytest"):
        print(
            "Skipping python tests because the 'pytest' target "
            "was not generated in this configuration."
        )
        return 0

    run_cmake_build(build_dir, build_type, "pytest")
    return 0


if __name__ == "__main__":
    sys.exit(main())
