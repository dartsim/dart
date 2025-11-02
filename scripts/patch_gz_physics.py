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


def ensure_gtest_include_before(cmake_file: Path) -> bool:
    """
    Ensure the gtest vendor target includes its headers before conda headers.
    This avoids ABI mismatches with conda's newer gtest package.
    """
    if not cmake_file.exists():
        print(f"Error: gtest CMakeLists not found at {cmake_file}", file=sys.stderr)
        return False

    content = cmake_file.read_text()
    before_snippet = (
        "target_include_directories(gtest\n"
        "  SYSTEM BEFORE PUBLIC\n"
        "    ${CMAKE_CURRENT_SOURCE_DIR}/include\n"
        "  PRIVATE\n"
        "    ${CMAKE_CURRENT_SOURCE_DIR}\n"
        "    ${CMAKE_CURRENT_SOURCE_DIR}/src\n"
        ")\n"
    )

    if "SYSTEM BEFORE PUBLIC" in content:
        print("✓ gtest include ordering already adjusted")
        return True

    old_snippet = (
        "target_include_directories(gtest \n"
        "  SYSTEM PUBLIC\n"
        "  ${CMAKE_CURRENT_SOURCE_DIR}/include\n"
        "  PRIVATE\n"
        "  ${CMAKE_CURRENT_SOURCE_DIR}\n"
        "  ${CMAKE_CURRENT_SOURCE_DIR}/src\n"
        ")\n"
    )

    if old_snippet not in content:
        print(
            "Warning: Expected gtest include block not found; no changes applied",
            file=sys.stderr,
        )
        return False

    cmake_file.write_text(content.replace(old_snippet, before_snippet))
    print(f"✓ Updated gtest include ordering in {cmake_file}")
    return True


def prefer_system_gtest(test_cmake: Path) -> bool:
    """
    Teach gz-physics to use system-provided GTest when available to avoid
    ABI mismatches with conda packages.
    """
    if not test_cmake.exists():
        print(f"Error: test CMakeLists not found at {test_cmake}", file=sys.stderr)
        return False

    content = test_cmake.read_text()
    if "find_package(GTest" in content:
        print("✓ System GTest preference already present")
        return True

    replacement = """find_package(GTest QUIET)

if (TARGET GTest::gtest AND TARGET GTest::gtest_main)
  get_target_property(_gtest_location GTest::gtest IMPORTED_LOCATION)
  if (NOT _gtest_location)
    get_target_property(_gtest_location GTest::gtest IMPORTED_LOCATION_RELEASE)
  endif()
  message(STATUS "Detected GTest::gtest at '${_gtest_location}'")
  if (_gtest_location)
    get_filename_component(_gtest_libdir \"${_gtest_location}\" DIRECTORY)
    if (_gtest_libdir)
      link_directories(${_gtest_libdir})
    endif()
  endif()

  add_library(gtest INTERFACE IMPORTED)
  if (_gtest_location)
    target_link_libraries(gtest INTERFACE GTest::gtest "${_gtest_location}")
  else()
    target_link_libraries(gtest INTERFACE GTest::gtest)
  endif()

  add_library(gtest_main INTERFACE IMPORTED)
  get_target_property(_gtest_main_location GTest::gtest_main IMPORTED_LOCATION)
  if (NOT _gtest_main_location)
    get_target_property(_gtest_main_location GTest::gtest_main IMPORTED_LOCATION_RELEASE)
  endif()
  message(STATUS "Detected GTest::gtest_main at '${_gtest_main_location}'")
  if (_gtest_main_location)
    target_link_libraries(gtest_main INTERFACE GTest::gtest_main gtest "${_gtest_main_location}")
  else()
    target_link_libraries(gtest_main INTERFACE GTest::gtest_main gtest)
  endif()
else()
  add_subdirectory(gtest_vendor)
endif()
"""

    if "add_subdirectory(gtest_vendor)" not in content:
        print(
            "Warning: Expected gtest vendor subdirectory not found; no changes applied",
            file=sys.stderr,
        )
        return False

    test_cmake.write_text(
        content.replace("add_subdirectory(gtest_vendor)\n", replacement)
    )
    print(f"✓ Updated {test_cmake} to prefer system GTest")
    return True


def main():
    """Main entry point for the script."""
    # Get the gz-physics CMakeLists.txt path
    repo_root = Path(__file__).parent.parent
    cmake_file = repo_root / ".deps" / "gz-physics" / "CMakeLists.txt"
    gtest_vendor_cmake = (
        repo_root / ".deps" / "gz-physics" / "test" / "gtest_vendor" / "CMakeLists.txt"
    )
    test_cmake = repo_root / ".deps" / "gz-physics" / "test" / "CMakeLists.txt"

    # Patch versions
    old_version = "6.10"
    new_version = "7.0"

    # Apply the patch
    cmake_success = patch_gz_physics_cmake(cmake_file, old_version, new_version)
    gtest_include_success = ensure_gtest_include_before(gtest_vendor_cmake)
    gtest_system_success = prefer_system_gtest(test_cmake)

    # Exit with appropriate code
    sys.exit(
        0 if cmake_success and gtest_include_success and gtest_system_success else 1
    )


if __name__ == "__main__":
    main()
