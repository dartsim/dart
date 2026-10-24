#!/usr/bin/env python3
"""Verify the layout of dartpy wheels: the package, its extension and licenses."""

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
    names = set(file_list)
    extensions = [
        name
        for name in file_list
        if name.startswith("dartpy/_core.") and name.endswith((".so", ".pyd"))
    ]
    non_metadata_files = [
        name
        for name in file_list
        if ".dist-info/" not in name and not name.endswith(".dist-info")
    ]
    dist_info = next(
        (name.partition("/")[0] for name in file_list if name.endswith("/METADATA")),
        "",
    )

    ok = True
    if "-cp312-abi3-" not in wheel_path.name:
        print("ERROR: Expected a cp312-abi3 wheel (DART_DARTPY_STABLE_ABI).")
        ok = False
    if not extensions:
        print("ERROR: No dartpy._core extension module found in wheel.")
        ok = False
    for required in (
        "dartpy/__init__.py",
        "dartpy/py.typed",
        "dartpy/experimental/__init__.py",
        f"{dist_info}/licenses/LICENSE",
        f"{dist_info}/licenses/filament/LICENSE",
    ):
        if required not in names:
            print(f"ERROR: Wheel is missing {required}.")
            ok = False
    # auditwheel and delvewheel vendor shared libraries into dartpy.libs/;
    # delocate uses dartpy/.dylibs/ inside the package.
    stray = sorted(
        name
        for name in file_list
        if not name.startswith(("dartpy/", "dartpy.libs/", f"{dist_info}/"))
        or name.startswith("dartpy/_filament_licenses/")
    )
    if stray:
        print(
            "ERROR: Wheel contains files outside the dartpy package: "
            + ", ".join(stray)
        )
        ok = False
    if not non_metadata_files:
        print("ERROR: Wheel contains only metadata files.")
        ok = False
    if not ok:
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
