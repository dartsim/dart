#!/usr/bin/env python3
"""Validate gz-physics' DART version requirement for DART 7 package config.

DART 7's generated DARTConfigVersion.cmake intentionally satisfies legacy
DART 6.10+ find_package requests used by pinned gz-physics checkouts. This
script keeps the gazebo pixi workflow shape intact while avoiding local
gz-physics source mutation.
"""

import re
import sys
from pathlib import Path

MIN_COMPATIBLE_VERSION = (6, 10)
MAX_COMPATIBLE_VERSION = (7, 0)


def parse_major_minor(version: str) -> tuple[int, int] | None:
    parts = version.split(".")
    if len(parts) < 2:
        return None

    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        return None


def validate_gz_physics_cmake(cmake_file: Path) -> bool:
    """Return True when gz-physics asks for a DART version DART 7 supports."""
    if not cmake_file.exists():
        print(f"Error: CMakeLists.txt not found at {cmake_file}", file=sys.stderr)
        return False

    content = cmake_file.read_text()

    dart_version_pattern = re.compile(
        r"(gz_find_package\(\s*DART\b.*?\bVERSION\s+)(\d+\.\d+)(\b)",
        re.DOTALL,
    )
    match = dart_version_pattern.search(content)
    if not match:
        print(
            f"Error: Failed to locate DART version requirement in {cmake_file}",
            file=sys.stderr,
        )
        return False

    current_version = match.group(2)
    parsed_version = parse_major_minor(current_version)
    if parsed_version is None:
        print(
            f"Error: Unsupported DART VERSION {current_version} in {cmake_file}",
            file=sys.stderr,
        )
        return False

    if parsed_version < MIN_COMPATIBLE_VERSION:
        print(
            "Error: gz-physics requires DART VERSION "
            f"{current_version}, below DART 7's compatibility floor",
            file=sys.stderr,
        )
        return False

    if parsed_version > MAX_COMPATIBLE_VERSION:
        print(
            "Error: gz-physics requires DART VERSION "
            f"{current_version}, above the DART 7 package version",
            file=sys.stderr,
        )
        return False

    print(
        f"✓ DART VERSION {current_version} is compatible with DART 7 " "package config"
    )
    return True


def main() -> None:
    repo_root = Path(__file__).parent.parent
    cmake_file = repo_root / ".deps" / "gz-physics" / "CMakeLists.txt"

    success = validate_gz_physics_cmake(cmake_file)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
