#!/usr/bin/env python3
"""
Patch gz-physics CMakeLists.txt to update the required DART version.

This is intentionally minimal to keep Gazebo integration testing stable:
- Only updates `VERSION 6.10` -> `VERSION 7.0` in gz-physics.
- Ensures gz-physics tests prefer its vendored gtest headers to avoid ABI
  mismatches with system gtest.
"""

from __future__ import annotations

import sys
from pathlib import Path


def patch_gz_physics_cmake(
    cmake_file: Path, old_version: str, new_version: str
) -> bool:
    if not cmake_file.is_file():
        print(f"Error: CMakeLists.txt not found at {cmake_file}", file=sys.stderr)
        return False

    content = cmake_file.read_text(encoding="utf-8")

    new_pattern = f"VERSION {new_version}"
    if new_pattern in content:
        print(f"✓ Patch already applied ({new_pattern} found)")
        return True

    old_pattern = f"VERSION {old_version}"
    if old_pattern not in content:
        print(
            f"Error: Expected pattern '{old_pattern}' not found in {cmake_file}",
            file=sys.stderr,
        )
        return False

    cmake_file.write_text(content.replace(old_pattern, new_pattern), encoding="utf-8")
    print(f"✓ Patched {cmake_file}: {old_pattern} → {new_pattern}")
    return True


def patch_gz_physics_test_gtest_includes(test_cmake: Path) -> bool:
    if not test_cmake.is_file():
        print(f"Error: test/CMakeLists.txt not found at {test_cmake}", file=sys.stderr)
        return False

    content = test_cmake.read_text(encoding="utf-8")

    marker = (
        "target_include_directories(gz-physics-test BEFORE INTERFACE "
        "${CMAKE_CURRENT_SOURCE_DIR}/gtest_vendor/include)"
    )
    if marker in content:
        print(f"✓ Patch already applied ({test_cmake.name}: gtest include override)")
        return True

    anchor = (
        "target_include_directories(gz-physics-test INTERFACE "
        "${CMAKE_CURRENT_SOURCE_DIR}/include)"
    )
    if anchor not in content:
        print(
            f"Error: Expected anchor '{anchor}' not found in {test_cmake}",
            file=sys.stderr,
        )
        return False

    content = content.replace(anchor, f"{anchor}\n{marker}", 1)
    test_cmake.write_text(content, encoding="utf-8")
    print(f"✓ Patched {test_cmake}: ensure vendored gtest headers win")
    return True


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    gz_root = repo_root / ".deps" / "gz-physics"
    cmake_file = gz_root / "CMakeLists.txt"
    test_cmake = gz_root / "test" / "CMakeLists.txt"

    ok = patch_gz_physics_cmake(cmake_file, old_version="6.10", new_version="7.0")
    ok = patch_gz_physics_test_gtest_includes(test_cmake) and ok

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
