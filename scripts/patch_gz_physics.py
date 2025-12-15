#!/usr/bin/env python3
"""
Patch gz-physics CMakeLists.txt to accept DART 7.0.

This script intentionally limits its scope to:
1) Bumping the DART CMake package version requirement in gz-physics.
2) Applying a small GoogleTest shim needed by the gz-physics test build.

For DART 7.0, we want to preserve backward compatibility for the APIs currently
used by Gazebo/gz-physics, so we should not carry local source patches here
besides what is required to keep this integration test runnable. If gz-physics
fails to build against DART 7, fix the compatibility in DART or upstream in
gz-physics instead of modifying their sources in this repository.
"""

import re
import sys
from pathlib import Path

OLD_DART_VERSION = "6.10"
NEW_DART_VERSION = "7.0"

_GTEST_SHIM_SOURCE = """\
#include "gtest/gtest.h"

namespace testing {
namespace internal {

TestInfo* MakeAndRegisterTestInfo(
    std::string test_suite_name, const char* name, const char* type_param,
    const char* value_param, CodeLocation code_location,
    TypeId fixture_class_id, SetUpTestSuiteFunc set_up_tc,
    TearDownTestSuiteFunc tear_down_tc, TestFactoryBase* factory) {
  return MakeAndRegisterTestInfo(test_suite_name.c_str(), name, type_param,
                                 value_param, code_location, fixture_class_id,
                                 set_up_tc, tear_down_tc, factory);
}

}  // namespace internal
}  // namespace testing
"""


def patch_gz_physics_cmake(
    cmake_file: Path, old_version: str, new_version: str
) -> bool:
    """
    Update the DART version requirement in gz-physics' top-level CMakeLists.txt.

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


def patch_gz_physics_gtest_vendor(gtest_vendor_dir: Path) -> bool:
    """
    Patch gz-physics' vendored gtest to provide the std::string overload of
    MakeAndRegisterTestInfo.

    Some gz-physics test targets can accidentally pick up system gtest headers
    (via transitive dependencies) while linking against the vendored gtest
    static library. Newer gtest headers reference the std::string overload,
    which older vendored gtest releases do not define, resulting in link errors.
    """
    cmake_file = gtest_vendor_dir / "CMakeLists.txt"
    shim_source = gtest_vendor_dir / "src" / "dart_make_and_register_shim.cc"

    if not cmake_file.exists():
        print(
            f"Error: gtest_vendor CMakeLists.txt not found at {cmake_file}",
            file=sys.stderr,
        )
        return False

    shim_source.parent.mkdir(parents=True, exist_ok=True)
    if not shim_source.exists() or shim_source.read_text() != _GTEST_SHIM_SOURCE:
        shim_source.write_text(_GTEST_SHIM_SOURCE)
        print(f"✓ Wrote gtest shim source: {shim_source}")
    else:
        print(f"✓ gtest shim source already present: {shim_source}")

    cmake_text = cmake_file.read_text()
    shim_entry = "${CMAKE_CURRENT_SOURCE_DIR}/src/dart_make_and_register_shim.cc"
    if shim_entry in cmake_text:
        print(f"✓ gtest CMake already references shim: {cmake_file}")
        return True

    old_line = "add_library(gtest STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/gtest-all.cc)"
    if old_line not in cmake_text:
        print(
            f"Error: Failed to locate gtest add_library line in {cmake_file}",
            file=sys.stderr,
        )
        return False

    new_block = (
        "add_library(gtest STATIC\n"
        "  ${CMAKE_CURRENT_SOURCE_DIR}/src/gtest-all.cc\n"
        "  ${CMAKE_CURRENT_SOURCE_DIR}/src/dart_make_and_register_shim.cc\n"
        ")"
    )
    cmake_file.write_text(cmake_text.replace(old_line, new_block, 1))
    print(f"✓ Updated gtest CMakeLists to include shim: {cmake_file}")
    return True


def main() -> None:
    repo_root = Path(__file__).parent.parent
    cmake_file = repo_root / ".deps" / "gz-physics" / "CMakeLists.txt"
    gtest_vendor_dir = repo_root / ".deps" / "gz-physics" / "test" / "gtest_vendor"

    success = patch_gz_physics_cmake(cmake_file, OLD_DART_VERSION, NEW_DART_VERSION)
    if success:
        success = patch_gz_physics_gtest_vendor(gtest_vendor_dir)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
