#!/usr/bin/env python3
"""Verify that dartpy wheels contain the compiled extension module."""

from __future__ import annotations

import glob
import sys
import zipfile
from pathlib import Path


def _expand_wheels(patterns: list[str]) -> list[Path]:
    wheel_paths: list[Path] = []
    for pattern in patterns:
        matches = sorted(Path(path) for path in glob.glob(pattern))
        if matches:
            wheel_paths.extend(matches)
        else:
            wheel_paths.append(Path(pattern))
    return wheel_paths


def verify_wheel(wheel_path: Path) -> bool:
    print(f"Verifying wheel: {wheel_path}")

    if not wheel_path.exists():
        print(f"ERROR: Wheel file not found: {wheel_path}")
        return False

    with zipfile.ZipFile(wheel_path, "r") as wheel:
        file_list = wheel.namelist()
        extensions = [
            name
            for name in file_list
            if "dartpy" in name.lower()
            and (name.endswith(".so") or name.endswith(".pyd") or ".so." in name)
        ]
        non_metadata_files = [
            name
            for name in file_list
            if ".dist-info/" not in name and not name.endswith(".dist-info")
        ]

    if not extensions:
        print("ERROR: No dartpy extension module found in wheel.")
        return False

    if not non_metadata_files:
        print("ERROR: Wheel contains only metadata files.")
        return False

    for extension in extensions:
        print(f"Found extension: {extension}")
    print(f"Wheel contains {len(non_metadata_files)} non-metadata files.")
    return True


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("Usage: python scripts/verify_wheel.py <wheel_file_or_glob> [...]")
        return 1

    wheel_paths = _expand_wheels(argv[1:])
    if not wheel_paths:
        print("ERROR: No wheels found.")
        return 1

    ok = True
    for wheel_path in wheel_paths:
        ok = verify_wheel(wheel_path) and ok

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
