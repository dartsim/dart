#!/usr/bin/env python3
"""
Migration script for updating CMakeLists.txt files to use the new
dart_generate_component_headers macro.

This script will:
1. Find all CMakeLists.txt files that use dart_generate_include_header_file
2. Extract the component name from the file path
3. Update them to use dart_generate_component_headers
4. Update the install() commands to include both all.hpp and the deprecated header

Usage:
    python migrate_cmakelists.py [--dry-run]

Options:
    --dry-run: Show what would be changed without actually modifying files
"""

import re
import sys
from pathlib import Path
from typing import List, Optional


def extract_component_name(cmake_path: Path) -> str:
    """
    Extract component name from the file path.
    For example:
        - dart/collision/CMakeLists.txt -> collision
        - dart/dynamics/CMakeLists.txt -> dynamics
        - dart/collision/fcl/CMakeLists.txt -> fcl
    """
    parts = cmake_path.parent.parts
    # The component name is the last part after 'dart'
    return parts[-1]


def extract_target_dir(content: str) -> Optional[str]:
    """
    Extract the TARGET_DIR from the existing dart_generate_include_header_file call.
    Example: "dart/collision/" from:
        dart_generate_include_header_file(
          "${CMAKE_CURRENT_BINARY_DIR}/collision.hpp"
          "dart/collision/"
          ${header_names}
        )
    """
    pattern = r'dart_generate_include_header_file\s*\(\s*"[^"]+"\s*"([^"]+)"'
    match = re.search(pattern, content)
    if match:
        return match.group(1)
    return None


def migrate_cmake_file(file_path: Path, dry_run: bool = False) -> bool:
    """
    Migrate a single CMakeLists.txt file to use the new macro.
    Returns True if the file was modified, False otherwise.
    """
    try:
        with open(file_path, 'r') as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return False

    # Check if this file uses dart_generate_include_header_file
    if 'dart_generate_include_header_file' not in content:
        return False

    print(f"\nProcessing: {file_path}")

    # Extract component name
    component_name = extract_component_name(file_path)
    print(f"  Component name: {component_name}")

    # Extract target directory
    target_dir = extract_target_dir(content)
    if not target_dir:
        print(f"  Warning: Could not extract target directory")
        return False
    print(f"  Target directory: {target_dir}")

    # Pattern to match the old dart_generate_include_header_file call
    old_pattern = re.compile(
        r'dart_generate_include_header_file\s*\(\s*'
        r'"(\$\{CMAKE_CURRENT_BINARY_DIR\})/([^"]+)"\s*'
        r'"([^"]+)"\s*'
        r'(\$\{[^}]+\})\s*'
        r'\)',
        re.DOTALL
    )

    # Replace with new dart_generate_component_headers call
    def replace_generate_call(match):
        binary_dir = match.group(1)
        target_dir = match.group(3)
        headers_var = match.group(4)

        return (
            f'dart_generate_component_headers(\n'
            f'  COMPONENT_NAME {component_name}\n'
            f'  TARGET_DIR "{target_dir}"\n'
            f'  OUTPUT_DIR "{binary_dir}"\n'
            f'  HEADERS {headers_var}\n'
            f')'
        )

    new_content = old_pattern.sub(replace_generate_call, content)

    # Update the install() command to include both all.hpp and the deprecated header
    # Pattern to match install command with the old header
    install_pattern = re.compile(
        r'install\s*\(\s*FILES\s+([^)]*)\$\{CMAKE_CURRENT_BINARY_DIR\}/([a-z_]+)\.hpp([^)]*)\)',
        re.DOTALL
    )

    def replace_install_call(match):
        pre_files = match.group(1)
        component = match.group(2)
        post_content = match.group(3)

        return (
            f'install(\n'
            f'  FILES {pre_files}${{CMAKE_CURRENT_BINARY_DIR}}/all.hpp ${{CMAKE_CURRENT_BINARY_DIR}}/{component}.hpp{post_content})'
        )

    new_content = install_pattern.sub(replace_install_call, new_content)

    # Check if content changed
    if new_content == content:
        print(f"  No changes needed")
        return False

    if dry_run:
        print(f"  Would update (dry run)")
        print(f"  Changes preview:")
        print("  " + "=" * 60)
        # Show a snippet of changes
        import difflib
        diff = difflib.unified_diff(
            content.splitlines(keepends=True),
            new_content.splitlines(keepends=True),
            fromfile=str(file_path),
            tofile=str(file_path) + ' (new)',
            lineterm=''
        )
        for line in list(diff)[:50]:  # Show first 50 lines of diff
            print(f"  {line.rstrip()}")
        print("  " + "=" * 60)
    else:
        try:
            with open(file_path, 'w') as f:
                f.write(new_content)
            print(f"  ✓ Updated successfully")
        except Exception as e:
            print(f"  Error writing {file_path}: {e}")
            return False

    return True


def find_cmake_files(root_dir: Path) -> List[Path]:
    """
    Find all CMakeLists.txt files under the dart directory.
    """
    dart_dir = root_dir / 'dart'
    if not dart_dir.exists():
        print(f"Error: dart directory not found at {dart_dir}")
        return []

    cmake_files = list(dart_dir.rglob('CMakeLists.txt'))
    return cmake_files


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='Migrate CMakeLists.txt files to use dart_generate_component_headers'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be changed without actually modifying files'
    )
    parser.add_argument(
        '--root',
        type=Path,
        default=Path.cwd(),
        help='Root directory of the DART project (default: current directory)'
    )

    args = parser.parse_args()

    root_dir = args.root
    if not root_dir.exists():
        print(f"Error: Root directory does not exist: {root_dir}")
        return 1

    print(f"Searching for CMakeLists.txt files in {root_dir / 'dart'}...")
    cmake_files = find_cmake_files(root_dir)

    if not cmake_files:
        print("No CMakeLists.txt files found")
        return 1

    print(f"Found {len(cmake_files)} CMakeLists.txt files")

    if args.dry_run:
        print("\n*** DRY RUN MODE - No files will be modified ***\n")

    modified_count = 0
    for cmake_file in sorted(cmake_files):
        if migrate_cmake_file(cmake_file, dry_run=args.dry_run):
            modified_count += 1

    print(f"\n{'Would modify' if args.dry_run else 'Modified'} {modified_count} file(s)")

    if args.dry_run:
        print("\nRun without --dry-run to apply changes")

    return 0


if __name__ == '__main__':
    sys.exit(main())
