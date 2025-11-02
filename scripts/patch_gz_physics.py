#!/usr/bin/env python3
"""
Patch gz-physics so its DART plugin can build against this checkout.

This script:
1. Updates the DART version requirement in gz-physics from 6.10 to 7.0
2. Adjusts the DART component list requested by gz-physics
3. Replaces vendored GoogleTest with system GoogleTest
4. Patches CustomMeshShape to use MeshShape::setMesh with explicit ownership
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

    content = cmake_file.read_text()

    if f"VERSION {new_version}" in content:
        print(f"✓ DART version patch already applied (VERSION {new_version} found)")
        return True

    old_pattern = f"VERSION {old_version}"
    if old_pattern not in content:
        print(
            f"Warning: Expected pattern '{old_pattern}' not found in {cmake_file}",
            file=sys.stderr,
        )
        print("File may have been updated upstream", file=sys.stderr)
        return False

    new_content = content.replace(old_pattern, f"VERSION {new_version}")
    cmake_file.write_text(new_content)

    backup_file = cmake_file.with_suffix(".txt.bak")
    backup_file.write_text(content)

    print(f"✓ Successfully patched {cmake_file}")
    print(f"  Changed: VERSION {old_version} → VERSION {new_version}")
    print(f"  Backup saved to: {backup_file}")

    return True


def patch_dart_component_requirements(cmake_file: Path) -> bool:
    """Update the DART find_package block to match the embedded collision backends."""
    if not cmake_file.exists():
        print(f"Error: CMakeLists.txt not found at {cmake_file}", file=sys.stderr)
        return False

    content = cmake_file.read_text()
    old_block = """gz_find_package(DART
  COMPONENTS
    collision-bullet
    collision-ode
    utils
    utils-urdf
  CONFIG
  VERSION 7.0
  REQUIRED_BY dartsim
  PKGCONFIG dart
  PKGCONFIG_VER_COMPARISON >=)
"""

    new_block = """gz_find_package(DART
  COMPONENTS
    utils
    utils-urdf
  CONFIG
  VERSION 7.0
  REQUIRED_BY dartsim
  PKGCONFIG dart
  PKGCONFIG_VER_COMPARISON >=)
"""

    if old_block not in content:
        if new_block in content:
            print("✓ gz-physics DART components already patched")
            return True
        print(
            "Error: Failed to locate gz-physics DART find_package block to patch",
            file=sys.stderr,
        )
        return False

    cmake_file.write_text(content.replace(old_block, new_block, 1))
    print("✓ Updated gz-physics DART component requirements")
    return True


def patch_gtest_vendor(gtest_vendor_cmake: Path) -> bool:
    """
    Patch gz-physics gtest_vendor to use system GoogleTest instead of vendored one.

    Args:
        gtest_vendor_cmake: Path to test/gtest_vendor/CMakeLists.txt

    Returns:
        True if patching succeeded, False otherwise
    """
    if not gtest_vendor_cmake.exists():
        print(f"Error: {gtest_vendor_cmake} not found", file=sys.stderr)
        return False

    content = gtest_vendor_cmake.read_text()

    if "find_package(GTest REQUIRED)" in content:
        print("✓ GoogleTest patch already applied")
        return True

    new_content = """# Use system GoogleTest instead of vendored source
find_package(GTest REQUIRED)

# Create interface libraries that wrap GTest:: targets
# This matches the names expected by the test build system
if(NOT TARGET gtest)
    add_library(gtest INTERFACE)
    target_link_libraries(gtest INTERFACE GTest::gtest)
endif()

if(NOT TARGET gtest_main)
    add_library(gtest_main INTERFACE)
    target_link_libraries(gtest_main INTERFACE GTest::gtest_main)
endif()
"""

    gtest_vendor_cmake.write_text(new_content)

    backup_file = gtest_vendor_cmake.with_suffix(".txt.bak")
    backup_file.write_text(content)

    print(f"✓ Successfully patched {gtest_vendor_cmake}")
    print("  Replaced vendored GoogleTest with system GoogleTest")
    print(f"  Backup saved to: {backup_file}")

    return True


def patch_custom_mesh_shape(mesh_shape_path: Path) -> bool:
    """
    Update CustomMeshShape to use MeshShape::setMesh with explicit ownership now
    that MeshShape stores meshes in shared_ptr.
    """
    if not mesh_shape_path.exists():
        print(f"Error: CustomMeshShape not found at {mesh_shape_path}", file=sys.stderr)
        return False

    text = mesh_shape_path.read_text()
    include_snippet = "#include <gz/common/SubMesh.hh>\n"
    uri_include = "#include <dart/common/Uri.hpp>\n"
    if include_snippet in text and uri_include not in text:
        text = text.replace(include_snippet, include_snippet + "\n" + uri_include, 1)

    old_tail = (
        "  this->mMesh = scene;\n"
        "  this->mIsBoundingBoxDirty = true;\n"
        "  this->mIsVolumeDirty = true;\n"
        "}\n"
    )
    new_tail = (
        "  this->setMesh(\n"
        "      scene,\n"
        "      dart::dynamics::MeshShape::MeshOwnership::Manual,\n"
        "      dart::common::Uri(),\n"
        "      nullptr);\n"
        "  this->mIsBoundingBoxDirty = true;\n"
        "  this->mIsVolumeDirty = true;\n"
        "}\n"
    )
    if old_tail not in text:
        if new_tail in text:
            print("✓ CustomMeshShape already patched")
            return True
        print(
            "Error: Failed to locate CustomMeshShape mesh assignment block",
            file=sys.stderr,
        )
        return False

    mesh_shape_path.write_text(text.replace(old_tail, new_tail, 1))
    print(f"✓ Patched CustomMeshShape to use MeshShape::setMesh in {mesh_shape_path}")
    return True


def main() -> None:
    repo_root = Path(__file__).parent.parent
    gz_physics_root = repo_root / ".deps" / "gz-physics"
    cmake_file = gz_physics_root / "CMakeLists.txt"
    gtest_vendor_cmake = gz_physics_root / "test" / "gtest_vendor" / "CMakeLists.txt"
    custom_mesh_shape = gz_physics_root / "dartsim" / "src" / "CustomMeshShape.cc"

    old_version = "6.10"
    new_version = "7.0"

    success = True
    success = patch_gz_physics_cmake(cmake_file, old_version, new_version) and success
    success = patch_dart_component_requirements(cmake_file) and success
    success = patch_gtest_vendor(gtest_vendor_cmake) and success
    success = patch_custom_mesh_shape(custom_mesh_shape) and success

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
