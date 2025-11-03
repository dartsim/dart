#!/usr/bin/env python3
"""
Patch gz-physics CMakeLists.txt to update DART version requirement.

This script updates the DART version requirement in gz-physics from 6.10 to 7.0.
Note: DART 7.0 will have API breaking changes. This patch may need updates when
gz-physics officially supports DART 7.x or when breaking changes are finalized.
"""

import sys
from pathlib import Path


def patch_gz_physics_cmake(
    cmake_file: Path, old_version: str, new_version: str
) -> bool:
    """
    Patch the gz-physics CMakeLists.txt file to update DART version.

    Args:
        cmake_file: Path to the CMakeLists.txt file
        old_version: Old DART version string (e.g., "6.10")
        new_version: New DART version string (e.g., "7.0")

    Returns:
        True if patching succeeded, False otherwise
    """
    if not cmake_file.exists():
        print(f"Error: CMakeLists.txt not found at {cmake_file}", file=sys.stderr)
        return False

    # Read the file
    content = cmake_file.read_text()

    # Check if patch is already applied
    if f"VERSION {new_version}" in content:
        print(f"✓ Patch already applied (VERSION {new_version} found)")
        return True

    # Check if old version exists
    old_pattern = f"VERSION {old_version}"
    if old_pattern not in content:
        print(
            f"Warning: Expected pattern '{old_pattern}' not found in {cmake_file}",
            file=sys.stderr,
        )
        print("File may have been updated upstream", file=sys.stderr)
        return False

    # Apply the patch
    new_content = content.replace(old_pattern, f"VERSION {new_version}")

    # Write the patched content
    cmake_file.write_text(new_content)

    # Create a backup
    backup_file = cmake_file.with_suffix(".txt.bak")
    backup_file.write_text(content)

    print(f"✓ Successfully patched {cmake_file}")
    print(f"  Changed: VERSION {old_version} → VERSION {new_version}")
    print(f"  Backup saved to: {backup_file}")

    return True


def main():
    """Main entry point for the script."""
    # Get the gz-physics CMakeLists.txt path
    repo_root = Path(__file__).parent.parent
    cmake_file = repo_root / ".deps" / "gz-physics" / "CMakeLists.txt"

    # Patch versions
    old_version = "6.10"
    new_version = "7.0"

    # Apply the patch
    success = patch_gz_physics_cmake(cmake_file, old_version, new_version)

    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
