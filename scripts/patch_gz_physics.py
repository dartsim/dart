#!/usr/bin/env python3
"""
Patch gz-physics CMakeLists.txt to update DART version requirement.

This script updates the DART version requirement in gz-physics from 6.10 to 7.0.
Note: DART 7.0 will have API breaking changes. This patch may need updates when
gz-physics officially supports DART 7.x or when breaking changes are finalized.
"""

import sys
from pathlib import Path
import re


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


def inject_make_and_register_overload(gtest_root: Path) -> bool:
    """
    Ensure the vendored gtest provides the std::string MakeAndRegisterTestInfo overload.

    Returns:
        True if the overload is present or successfully added, False otherwise.
    """

    internal_header = (
        gtest_root
        / "include"
        / "gtest"
        / "internal"
        / "gtest-internal.h"
    )
    public_header = gtest_root / "include" / "gtest" / "gtest.h"
    source_file = gtest_root / "src" / "gtest.cc"

    for path in (internal_header, public_header, source_file):
        if not path.exists():
            print(f"Error: Expected gtest file missing: {path}", file=sys.stderr)
            return False

    # Update internal header declaration with inline shim
    internal_text = internal_header.read_text()
    inline_block = (
        "#if GTEST_HAS_STD_STRING\n"
        "inline TestInfo* MakeAndRegisterTestInfo(\n"
        "    std::string test_suite_name, const char* name, const char* type_param,\n"
        "    const char* value_param, CodeLocation code_location,\n"
        "    TypeId fixture_class_id, SetUpTestSuiteFunc set_up_tc,\n"
        "    TearDownTestSuiteFunc tear_down_tc, TestFactoryBase* factory) {\n"
        "  return MakeAndRegisterTestInfo(test_suite_name.c_str(), name, type_param,\n"
        "                                 value_param, code_location, fixture_class_id,\n"
        "                                 set_up_tc, tear_down_tc, factory);\n"
        "}\n"
        "#endif  // GTEST_HAS_STD_STRING\n"
    )
    inline_pattern = re.compile(
        r"#if GTEST_HAS_STD_STRING\n"
        r"GTEST_API_\s+TestInfo\*\s+MakeAndRegisterTestInfo\(\s*\n"
        r"\s*std::string\s+test_suite_name,.*?\);\n"
        r"#endif\s+//\s+GTEST_HAS_STD_STRING\n",
        re.DOTALL,
    )
    if inline_pattern.search(internal_text):
        internal_text = inline_pattern.sub(inline_block + "\n", internal_text)
        internal_header.write_text(internal_text)
        print(
            f"✓ Replaced std::string overload declaration with inline shim in {internal_header}"
        )
    elif inline_block in internal_text:
        print(
            f"✓ std::string inline shim already present in {internal_header}"
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
            + inline_block
            + internal_text[insert_offset:]
        )
        internal_header.write_text(internal_text)
        print(
            f"✓ Added std::string inline shim to {internal_header}"
        )

    # Update public header friend declaration
    public_text = public_header.read_text()
    if "friend TestInfo* internal::MakeAndRegisterTestInfo(\n      std::string test_suite_name" not in public_text:
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

    # Remove legacy source definition (now provided inline)
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

    return True


def main():
    """Main entry point for the script."""
    # Get the gz-physics CMakeLists.txt path
    repo_root = Path(__file__).parent.parent
    cmake_file = repo_root / ".deps" / "gz-physics" / "CMakeLists.txt"
    gtest_root = cmake_file.parent / "test" / "gtest_vendor"

    # Patch versions
    old_version = "6.10"
    new_version = "7.0"

    # Apply the patch
    success = patch_gz_physics_cmake(cmake_file, old_version, new_version)

    if success:
        overload_success = inject_make_and_register_overload(gtest_root)
        success = success and overload_success

    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
