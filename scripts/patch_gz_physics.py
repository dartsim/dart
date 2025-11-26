#!/usr/bin/env python3
"""
Patch gz-physics CMakeLists.txt to update DART version requirement.

This script updates the DART version requirement in gz-physics from 6.10 to 7.0.
Note: DART 7.0 will have API breaking changes. This patch may need updates when
gz-physics officially supports DART 7.x or when breaking changes are finalized.
"""

import re
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


def inject_make_and_register_overload(gtest_root: Path) -> bool:
    """
    Ensure the vendored gtest provides the std::string MakeAndRegisterTestInfo overload.

    Returns:
        True if the overload is present or successfully added, False otherwise.
    """

    internal_header = gtest_root / "include" / "gtest" / "internal" / "gtest-internal.h"
    public_header = gtest_root / "include" / "gtest" / "gtest.h"
    source_file = gtest_root / "src" / "gtest.cc"
    port_header = gtest_root / "include" / "gtest" / "internal" / "gtest-port.h"

    for path in (internal_header, public_header, source_file, port_header):
        if not path.exists():
            print(f"Error: Expected gtest file missing: {path}", file=sys.stderr)
            return False

    # Ensure GTEST_HAS_STD_STRING is defined so the shim is always compiled
    port_text = port_header.read_text()
    if "define GTEST_HAS_STD_STRING" not in port_text:
        guard = "#define GOOGLETEST_INCLUDE_GTEST_INTERNAL_GTEST_PORT_H_\n"
        insert_loc = port_text.find(guard)
        if insert_loc == -1:
            print(
                f"Error: Failed to locate header guard in {port_header}",
                file=sys.stderr,
            )
            return False
        insert_pos = insert_loc + len(guard)
        insertion = (
            "\n#ifndef GTEST_HAS_STD_STRING\n"
            "#define GTEST_HAS_STD_STRING 1\n"
            "#endif  // GTEST_HAS_STD_STRING\n"
        )
        port_text = port_text[:insert_pos] + "\n" + insertion + port_text[insert_pos:]
        port_header.write_text(port_text)
        print(f"✓ Defined GTEST_HAS_STD_STRING in {port_header}")
    else:
        print(f"✓ GTEST_HAS_STD_STRING already defined in {port_header}")

    # Update internal header declaration to ensure the overload is declared
    internal_text = internal_header.read_text()
    declaration_block = (
        "#if GTEST_HAS_STD_STRING\n"
        "GTEST_API_ TestInfo* MakeAndRegisterTestInfo(\n"
        "    std::string test_suite_name, const char* name, const char* type_param,\n"
        "    const char* value_param, CodeLocation code_location,\n"
        "    TypeId fixture_class_id, SetUpTestSuiteFunc set_up_tc,\n"
        "    TearDownTestSuiteFunc tear_down_tc, TestFactoryBase* factory);\n"
        "#endif  // GTEST_HAS_STD_STRING\n"
    )
    inline_pattern = re.compile(
        r"#if GTEST_HAS_STD_STRING\n"
        r"(?:inline\s+)?TestInfo\*\s+MakeAndRegisterTestInfo\(\s*\n"
        r"\s*std::string\s+test_suite_name,.*?\n#endif  // GTEST_HAS_STD_STRING\n",
        re.DOTALL,
    )
    if inline_pattern.search(internal_text):
        internal_text = inline_pattern.sub(declaration_block + "\n", internal_text)
        internal_header.write_text(internal_text)
        print(
            f"✓ Replaced std::string overload inline body with declaration in {internal_header}"
        )
    elif declaration_block in internal_text:
        print(
            f"✓ std::string overload declaration already present in {internal_header}"
        )
    else:
        insertion_point = internal_text.find(
            "GTEST_API_ TestInfo* MakeAndRegisterTestInfo(\n"
            "    const char* test_suite_name"
        )
        if insertion_point == -1:
            print(
                f"Error: Failed to locate MakeAndRegisterTestInfo declaration in {internal_header}",
                file=sys.stderr,
            )
            return False
        insert_offset = internal_text.find(");\n", insertion_point)
        if insert_offset == -1:
            print(
                f"Error: Could not determine insertion point in {internal_header}",
                file=sys.stderr,
            )
            return False
        insert_offset += 3  # past closing );
        internal_text = (
            internal_text[:insert_offset]
            + "\n"
            + declaration_block
            + internal_text[insert_offset:]
        )
        internal_header.write_text(internal_text)
        print(f"✓ Added std::string overload declaration to {internal_header}")

    # Update public header friend declaration
    public_text = public_header.read_text()
    if (
        "friend TestInfo* internal::MakeAndRegisterTestInfo(\n      std::string test_suite_name"
        not in public_text
    ):
        pattern = re.compile(
            r"(friend\s+TestInfo\*\s+internal::MakeAndRegisterTestInfo\(\s*\n"
            r"\s*const char\*\s+test_suite_name,.*?internal::TestFactoryBase\*\s+factory\);\n)",
            re.DOTALL,
        )
        match = pattern.search(public_text)
        if not match:
            print(
                f"Error: Failed to locate friend declaration in {public_header}",
                file=sys.stderr,
            )
            return False
        insertion = (
            "#if GTEST_HAS_STD_STRING\n"
            "  friend TestInfo* internal::MakeAndRegisterTestInfo(\n"
            "      std::string test_suite_name, const char* name, const char* type_param,\n"
            "      const char* value_param, internal::CodeLocation code_location,\n"
            "      internal::TypeId fixture_class_id, internal::SetUpTestSuiteFunc set_up_tc,\n"
            "      internal::TearDownTestSuiteFunc tear_down_tc,\n"
            "      internal::TestFactoryBase* factory);\n"
            "#endif\n"
        )
        public_text = (
            public_text[: match.end()] + insertion + public_text[match.end() :]
        )
        public_header.write_text(public_text)
        print(f"✓ Added std::string friend declaration to {public_header}")
    else:
        print(f"✓ std::string friend declaration already present in {public_header}")

    # Remove legacy source definition (handled in shim source)
    source_text = source_file.read_text()
    source_inline_pattern = re.compile(
        r"\n#if GTEST_HAS_STD_STRING\n"
        r"TestInfo\*\s+MakeAndRegisterTestInfo\(\s*\n"
        r"\s*std::string\s+test_suite_name,.*?\n#endif  // GTEST_HAS_STD_STRING\n",
        re.DOTALL,
    )
    if source_inline_pattern.search(source_text):
        source_text = source_inline_pattern.sub("\n", source_text)
        source_file.write_text(source_text)
        print(f"✓ Removed redundant std::string overload definition from {source_file}")
    else:
        print(
            f"✓ No standalone std::string overload definition present in {source_file}"
        )

    # Ensure shim source exists and is added to the build
    shim_source = gtest_root / "src" / "dart_make_and_register_shim.cc"
    shim_contents = """\
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
    if not shim_source.exists():
        shim_source.write_text(shim_contents)
        print(f"✓ Added gtest shim source {shim_source}")
    else:
        existing = shim_source.read_text()
        if existing != shim_contents:
            shim_source.write_text(shim_contents)
            print(f"✓ Updated gtest shim source {shim_source}")
        else:
            print(f"✓ gtest shim source already present at {shim_source}")

    cmake_file = gtest_root / "CMakeLists.txt"
    cmake_text = cmake_file.read_text()
    shim_entry = "${CMAKE_CURRENT_SOURCE_DIR}/src/dart_make_and_register_shim.cc"
    if shim_entry not in cmake_text:
        pattern = (
            "add_library(gtest STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/gtest-all.cc)"
        )
        replacement = (
            "add_library(gtest STATIC\n"
            "  ${CMAKE_CURRENT_SOURCE_DIR}/src/gtest-all.cc\n"
            "  ${CMAKE_CURRENT_SOURCE_DIR}/src/dart_make_and_register_shim.cc\n"
            ")"
        )
        if pattern not in cmake_text:
            print(
                f"Error: Failed to update {cmake_file} with shim source",
                file=sys.stderr,
            )
            return False
        cmake_file.write_text(cmake_text.replace(pattern, replacement, 1))
        print(f"✓ Added shim source to {cmake_file}")
    else:
        print(f"✓ Shim source already referenced in {cmake_file}")

    return True


def patch_custom_mesh_shape(mesh_shape_path: Path) -> bool:
    """
    Update CustomMeshShape to use MeshShape::setMesh with explicit ownership
    now that MeshShape stores meshes in shared_ptr.
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
        "      scene.release(),\n"
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
            mesh_shape_path.write_text(text)
            return True
        print(
            "Error: Failed to locate CustomMeshShape mesh assignment block",
            file=sys.stderr,
        )
        return False

    text = text.replace(old_tail, new_tail, 1)
    mesh_shape_path.write_text(text)
    print(f"✓ Patched CustomMeshShape to use MeshShape::setMesh in {mesh_shape_path}")
    return True


def main():
    """Main entry point for the script."""
    # Get the gz-physics CMakeLists.txt path
    repo_root = Path(__file__).parent.parent
    cmake_file = repo_root / ".deps" / "gz-physics" / "CMakeLists.txt"
    gtest_root = cmake_file.parent / "test" / "gtest_vendor"
    custom_mesh_shape = cmake_file.parent / "dartsim" / "src" / "CustomMeshShape.cc"

    # Patch versions
    old_version = "6.10"
    new_version = "7.0"

    # Apply the patch
    success = patch_gz_physics_cmake(cmake_file, old_version, new_version)

    if success:
        success = patch_dart_component_requirements(cmake_file) and success

    if success:
        success = inject_make_and_register_overload(gtest_root) and success
    if success:
        success = patch_custom_mesh_shape(custom_mesh_shape) and success

    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
