#!/usr/bin/env python3
"""
Verify version consistency across package.xml and built wheels.

Usage:
    # Check version in package.xml
    python scripts/verify_version.py

    # Verify version in a wheel matches package.xml
    python scripts/verify_version.py dist/dartpy-*.whl
"""

import re
import sys
import zipfile
from pathlib import Path


def extract_version_from_package_xml(package_xml_path: Path) -> str:
    """Extract version from package.xml."""
    content = package_xml_path.read_text()
    match = re.search(
        r"<version>([0-9]+\.[0-9]+\.[0-9]+(?:\.(dev|alpha|beta|rc)[0-9]+)?)</version>",
        content,
    )
    if not match:
        raise ValueError(f"Could not find version in {package_xml_path}")
    return match.group(1)


def extract_version_from_wheel(wheel_path: Path) -> str:
    """Extract version from wheel METADATA."""
    with zipfile.ZipFile(wheel_path, "r") as zf:
        # Find METADATA file
        metadata_files = [f for f in zf.namelist() if f.endswith("/METADATA")]
        if not metadata_files:
            raise ValueError(f"No METADATA file found in {wheel_path}")

        metadata_content = zf.read(metadata_files[0]).decode("utf-8")
        match = re.search(r"^Version: (.+)$", metadata_content, re.MULTILINE)
        if not match:
            raise ValueError(f"Could not find Version in METADATA")
        return match.group(1).strip()


def main():
    repo_root = Path(__file__).parent.parent
    package_xml = repo_root / "package.xml"

    # Get version from package.xml
    pkg_version = extract_version_from_package_xml(package_xml)
    print(f"✓ package.xml version: {pkg_version}")

    # Validate version format
    version_pattern = r"^[0-9]+\.[0-9]+\.[0-9]+(?:\.(dev|alpha|beta|rc)[0-9]+)?$"
    if not re.match(version_pattern, pkg_version):
        print(f"✗ ERROR: Invalid version format: {pkg_version}")
        print(f"  Expected format: X.Y.Z or X.Y.Z.{dev|alpha|beta|rc}N")
        return 1

    # Check if it's a pre-release version
    if any(suffix in pkg_version for suffix in [".dev", ".alpha", ".beta", ".rc"]):
        print(f"  → Pre-release version")
    else:
        print(f"  → Release version")

    # If wheel paths are provided, verify them
    if len(sys.argv) > 1:
        print()
        wheel_paths = sys.argv[1:]
        all_match = True

        for wheel_arg in wheel_paths:
            # Expand glob patterns
            wheel_files = list(Path(".").glob(wheel_arg))
            if not wheel_files:
                print(f"✗ No wheel files found matching: {wheel_arg}")
                all_match = False
                continue

            for wheel_path in wheel_files:
                try:
                    wheel_version = extract_version_from_wheel(wheel_path)
                    if wheel_version == pkg_version:
                        print(f"✓ {wheel_path.name}: {wheel_version} (matches)")
                    else:
                        print(
                            f"✗ {wheel_path.name}: {wheel_version} (expected {pkg_version})"
                        )
                        all_match = False
                except Exception as e:
                    print(f"✗ {wheel_path.name}: ERROR - {e}")
                    all_match = False

        if not all_match:
            return 1
    else:
        print()
        print("Tip: To verify wheels, run:")
        print("  python scripts/verify_version.py dist/dartpy-*.whl")

    return 0


if __name__ == "__main__":
    sys.exit(main())
