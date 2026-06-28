#!/usr/bin/env python3
"""Emit CMake linker flags for host compilers running inside Pixi envs."""

from __future__ import annotations

import os
import shlex
import shutil
import sys
from pathlib import Path

_LINKER_FLAG_VARS = (
    "CMAKE_EXE_LINKER_FLAGS",
    "CMAKE_SHARED_LINKER_FLAGS",
    "CMAKE_MODULE_LINKER_FLAGS",
)


def _resolve_compiler() -> Path | None:
    value = os.environ.get("CXX") or "c++"
    resolved = shutil.which(value)
    if resolved is None:
        return None
    return Path(resolved).resolve()


def _inside(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _linker_flag_value(name: str) -> str:
    values = [
        os.environ.get(name, ""),
        os.environ.get("LDFLAGS", ""),
        "-B/usr/bin",
    ]
    return " ".join(value for value in values if value)


def cmake_host_linker_flags() -> list[str]:
    if sys.platform != "linux":
        return []

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if not conda_prefix:
        return []

    compiler = _resolve_compiler()
    if compiler is None:
        return []

    # CMake defaults to the host compiler in DART's Linux Pixi environments.
    # A refreshed lockfile can still put Conda binutils earlier on PATH. When
    # that happens, GCC's collect2 may select the Pixi linker and then mix the
    # Conda sysroot with host startup/libc objects. Keep the host toolchain
    # coherent by making GCC search /usr/bin for linker subprograms first.
    if not str(compiler).startswith(("/usr/bin/", "/bin/")):
        return []

    linker = shutil.which("ld")
    if linker is None:
        return []

    if not _inside(Path(linker).resolve(), Path(conda_prefix).resolve()):
        return []

    if not Path("/usr/bin/ld").is_file():
        return []

    return [f"-D{name}:STRING={_linker_flag_value(name)}" for name in _LINKER_FLAG_VARS]


def main() -> int:
    print(shlex.join(cmake_host_linker_flags()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
