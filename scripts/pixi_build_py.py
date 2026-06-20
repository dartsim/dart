#!/usr/bin/env python3
"""Run the Pixi Python build while preserving caller CMake arguments."""

from __future__ import annotations

import os
import subprocess
import sys

SYSTEM_IMGUI_CMAKE_ARG = "-DDART_USE_SYSTEM_IMGUI=ON"


def with_system_imgui_cmake_args(environ: dict[str, str]) -> dict[str, str]:
    """Return an environment with Pixi's default ImGui CMake option prepended."""
    updated = dict(environ)
    existing = updated.get("CMAKE_ARGS", "").strip()
    updated["CMAKE_ARGS"] = (
        f"{SYSTEM_IMGUI_CMAKE_ARG} {existing}" if existing else SYSTEM_IMGUI_CMAKE_ARG
    )
    return updated


def main() -> int:
    env = with_system_imgui_cmake_args(dict(os.environ))
    return subprocess.call(
        [sys.executable, "-m", "pip", "install", ".", "-vv"],
        env=env,
    )


if __name__ == "__main__":
    raise SystemExit(main())
