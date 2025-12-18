#!/usr/bin/env python3
from __future__ import annotations

import os
import shlex
import subprocess
import sys
from pathlib import Path


def _is_false(value: str | None) -> bool:
    if value is None:
        return False
    return value.strip().lower() in {"0", "false", "off", "no"}


def _is_sccache_launcher(value: str | None) -> bool:
    if not value:
        return False
    launcher_name = Path(value).name.lower()
    return launcher_name in {"sccache", "sccache.exe"}


def main(argv: list[str]) -> int:
    use_system_imgui = argv[1].upper() if len(argv) > 1 else "ON"
    if use_system_imgui not in {"ON", "OFF"}:
        print(
            f"Invalid use_system_imgui value '{use_system_imgui}'. Expected ON/OFF.",
            file=sys.stderr,
        )
        return 2

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if not conda_prefix:
        print(
            "CONDA_PREFIX is not set; wheel build must run inside a pixi env.",
            file=sys.stderr,
        )
        return 2

    disable_compiler_cache = os.environ.get(
        "DART_DISABLE_COMPILER_CACHE", "OFF"
    ).upper()
    if _is_false(os.environ.get("SCCACHE_GHA_ENABLED")):
        disable_compiler_cache = "ON"

    if disable_compiler_cache == "ON":
        os.environ.pop("DART_COMPILER_CACHE", None)
        for launcher_var in (
            "CMAKE_C_COMPILER_LAUNCHER",
            "CMAKE_CXX_COMPILER_LAUNCHER",
        ):
            if _is_sccache_launcher(os.environ.get(launcher_var)):
                os.environ.pop(launcher_var, None)

    cmake_args = [
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        "-DDART_BUILD_DARTPY=ON",
        "-DDART_BUILD_GUI=ON",
        "-DDART_ENABLE_SIMD=OFF",
        "-DDART_BUILD_WHEELS=ON",
        "-DDART_TREAT_WARNINGS_AS_ERRORS=OFF",
        "-DDART_BUILD_TESTS=OFF",
        "-DDART_BUILD_EXAMPLES=OFF",
        "-DDART_BUILD_TUTORIALS=OFF",
        "-DBUILD_SHARED_LIBS=OFF",
        f"-DDART_USE_SYSTEM_IMGUI={use_system_imgui}",
        f"-DDART_DISABLE_COMPILER_CACHE={disable_compiler_cache}",
    ]

    os.environ["DART_DISABLE_COMPILER_CACHE"] = disable_compiler_cache
    os.environ["CMAKE_ARGS"] = " ".join(cmake_args)

    Path("dist").mkdir(parents=True, exist_ok=True)

    cmd = ["pip", "wheel", ".", "--no-deps", "-w", "dist/", "-v"]
    print(f"Running: {' '.join(shlex.quote(part) for part in cmd)}")
    print(f"CMAKE_ARGS={os.environ['CMAKE_ARGS']}")
    subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
