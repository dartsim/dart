#!/usr/bin/env python3
"""Build a dartpy wheel from the active Pixi environment."""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
from pathlib import Path

from cmake_host_linker_flags import cmake_host_linker_flags


def _cmake_path(path: Path) -> str:
    return path.resolve().as_posix()


def _cmake_prefix_path(conda_prefix: str) -> str:
    prefix = Path(conda_prefix)
    entries = [prefix]
    if sys.platform == "win32":
        entries.append(prefix / "Library")
    return ";".join(_cmake_path(entry) for entry in entries)


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
            "CONDA_PREFIX is not set; wheel build must run inside a Pixi env.",
            file=sys.stderr,
        )
        return 2

    if sys.platform != "win32" and not os.environ.get("CMAKE_GENERATOR"):
        os.environ["CMAKE_GENERATOR"] = "Ninja"

    if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
        os.environ["CMAKE_BUILD_PARALLEL_LEVEL"] = str(min(os.cpu_count() or 1, 8))

    cmake_args = [
        f"-DCMAKE_PREFIX_PATH={_cmake_prefix_path(conda_prefix)}",
        "-DDART_BUILD_DARTPY=ON",
        "-DDART_ENABLE_SIMD=OFF",
        "-DDART_BUILD_WHEELS=ON",
        "-DDART_TREAT_WARNINGS_AS_ERRORS=OFF",
        "-DBUILD_SHARED_LIBS=OFF",
        f"-DDART_USE_SYSTEM_IMGUI={use_system_imgui}",
    ]
    cmake_args.extend(cmake_host_linker_flags())

    os.environ["CMAKE_ARGS"] = shlex.join(cmake_args)

    Path("dist").mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        "-m",
        "pip",
        "wheel",
        ".",
        "--no-build-isolation",
        "--no-deps",
        "-w",
        "dist/",
        "-v",
    ]
    print(f"Running: {' '.join(shlex.quote(part) for part in cmd)}")
    print(f"CMAKE_ARGS={os.environ['CMAKE_ARGS']}")
    subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
