#!/usr/bin/env python3
"""Run the Pixi Python build while preserving caller CMake arguments."""

from __future__ import annotations

import os
import subprocess
import sys


def with_cmake_args(environ: dict[str, str], cmake_args: list[str]) -> dict[str, str]:
    """Return an environment with Pixi CMake options prepended."""
    updated = dict(environ)
    extra_args = " ".join(cmake_args).strip()
    existing = updated.get("CMAKE_ARGS", "").strip()
    combined = f"{extra_args} {existing}".strip()
    if combined:
        updated["CMAKE_ARGS"] = combined
    else:
        updated.pop("CMAKE_ARGS", None)
    return updated


def main() -> int:
    env = with_cmake_args(dict(os.environ), sys.argv[1:])
    return subprocess.call(
        [sys.executable, "-m", "pip", "install", ".", "-vv"],
        env=env,
    )


if __name__ == "__main__":
    raise SystemExit(main())
