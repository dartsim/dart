#!/usr/bin/env python3
"""
Patch gz-physics CMakeLists.txt to accept DART 7.0.

This script intentionally limits its scope to the CMake version requirement only.
For DART 7.0, we want to preserve backward compatibility for the APIs currently
used by Gazebo/gz-physics, so we should not carry local source patches here. If
gz-physics fails to build against DART 7, fix the compatibility in DART or
upstream in gz-physics instead of modifying their sources in this repository.
"""

import re
import sys
from pathlib import Path

OLD_DART_VERSION = "6.10"
NEW_DART_VERSION = "7.0"


def patch_gz_physics_cmake(
    cmake_file: Path, old_version: str, new_version: str
) -> bool:
    """Update the DART version requirement in gz-physics' top-level CMakeLists.txt.

    Returns:
        True if patching succeeded or is already applied, False otherwise.
    """
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
    if current_version == new_version:
        print(f"✓ Patch already applied (DART VERSION {new_version} found)")
        return True

    if current_version != old_version:
        print(
            f"Warning: Expected DART VERSION {old_version} but found {current_version} in {cmake_file}",
            file=sys.stderr,
        )
        print("File may have been updated upstream", file=sys.stderr)
        return False

    new_content = dart_version_pattern.sub(
        rf"\g<1>{new_version}\g<3>", content, count=1
    )

    backup_file = cmake_file.with_suffix(".txt.bak")
    backup_file.write_text(content)
    cmake_file.write_text(new_content)

    print(f"✓ Successfully patched {cmake_file}")
    print(f"  Changed: DART VERSION {old_version} → {new_version}")
    print(f"  Backup saved to: {backup_file}")
    return True


def main() -> None:
    repo_root = Path(__file__).parent.parent
    cmake_file = repo_root / ".deps" / "gz-physics" / "CMakeLists.txt"

    success = patch_gz_physics_cmake(cmake_file, OLD_DART_VERSION, NEW_DART_VERSION)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
