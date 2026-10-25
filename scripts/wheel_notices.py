#!/usr/bin/env python3
"""Add the Linux wheel image's third-party notices to repaired wheels.

docker/wheels/Dockerfile.manylinux_2_28 copies the license files of the libraries
that the wheel bundles or compiles in, and of the third-party code inside assimp
and ODE, to one directory. This script adds those files to each wheel's
dist-info/licenses/python/dartpy/licenses/, next to the curl and OpenSSL notices
that pyproject.toml ships, with License-File headers and a new RECORD.
scripts/verify_wheel.py checks that every bundled library has its notices.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

DESTINATION = "python/dartpy/licenses/"


def _backend():
    path = Path(__file__).resolve().parent / "pep517/dartpy_backend.py"
    spec = importlib.util.spec_from_file_location("dartpy_backend", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print("Usage: python scripts/wheel_notices.py <notice_dir> <wheel> [...]")
        return 1
    notices = {
        f"{DESTINATION}{path.name}": path.read_bytes()
        for path in sorted(Path(argv[1]).iterdir())
    }
    if not notices:
        print(f"ERROR: {argv[1]} has no notices.")
        return 1
    add_license_files = _backend().add_license_files
    for wheel in argv[2:]:
        for name in add_license_files(Path(wheel), notices):
            print(f"Added {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
