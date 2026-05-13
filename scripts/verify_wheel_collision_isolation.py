#!/usr/bin/env python3
"""Verify dartpy wheels do not bundle legacy collision runtime artifacts."""

from __future__ import annotations

import argparse
import fnmatch
import glob
import sys
import zipfile
from pathlib import Path

FORBIDDEN_BASENAME_PATTERNS = (
    "dart-collision-fcl*",
    "dart-collision-bullet*",
    "dart-collision-ode*",
    "dart-collision-reference-fcl*",
    "dart-collision-reference-bullet*",
    "dart-collision-reference-ode*",
    "dart_collision-fclcomponent.cmake",
    "dart_collision-bulletcomponent.cmake",
    "dart_collision-odecomponent.cmake",
    "dart_collision-reference-fclcomponent.cmake",
    "dart_collision-reference-bulletcomponent.cmake",
    "dart_collision-reference-odecomponent.cmake",
    "libdart-collision-fcl*",
    "libdart-collision-bullet*",
    "libdart-collision-ode*",
    "libdart-collision-reference-fcl*",
    "libdart-collision-reference-bullet*",
    "libdart-collision-reference-ode*",
    "libfcl*.so*",
    "libfcl*.dylib",
    "fcl*.dll",
    "libccd*.so*",
    "libccd*.dylib",
    "ccd*.dll",
    "libode*.so*",
    "libode*.dylib",
    "ode*.dll",
    "libbullet*.so*",
    "libbullet*.dylib",
    "bullet*.dll",
    "liblinearmath*.so*",
    "liblinearmath*.dylib",
    "linearmath*.dll",
)


def expand_wheels(patterns: list[str]) -> list[Path]:
    wheels: list[Path] = []
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            wheels.extend(Path(match) for match in matches)
        else:
            wheels.append(Path(pattern))
    return wheels


def check_wheel(wheel_path: Path) -> list[str]:
    failures: list[str] = []
    if not wheel_path.exists():
        return [f"{wheel_path}: file does not exist"]

    with zipfile.ZipFile(wheel_path, "r") as wheel:
        names = wheel.namelist()

    print(f"Checking collision isolation: {wheel_path}")
    print(f"  inspected {len(names)} wheel entries")

    for entry in names:
        basename = entry.rsplit("/", 1)[-1].lower()
        for pattern in FORBIDDEN_BASENAME_PATTERNS:
            if fnmatch.fnmatchcase(basename, pattern):
                failures.append(f"{wheel_path}: forbidden wheel entry: {entry}")
                break

    if not failures:
        print("  ok: no legacy collision runtime artifacts found")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Verify dartpy wheels do not include legacy collision runtime "
            "libraries, reference collision libraries, or old collision "
            "component exports."
        )
    )
    parser.add_argument("wheels", nargs="+", help="Wheel path or glob pattern")
    args = parser.parse_args()

    failures: list[str] = []
    for wheel_path in expand_wheels(args.wheels):
        failures.extend(check_wheel(wheel_path))

    if failures:
        print("Wheel collision isolation check failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("Wheel collision isolation check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
