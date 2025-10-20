#!/usr/bin/env python3

"""
Verify that the dartpy wheel contains the necessary extension module.

This script extracts and examines a wheel file to ensure it contains
the dartpy extension module (.so, .pyd, or .dylib file).
"""

import sys
import zipfile
from pathlib import Path


def verify_wheel(wheel_path: Path) -> bool:
    """
    Verify that the wheel contains the dartpy extension module.

    Args:
        wheel_path: Path to the wheel file

    Returns:
        True if the wheel is valid, False otherwise
    """
    print(f"Verifying wheel: {wheel_path}")

    if not wheel_path.exists():
        print(f"❌ Wheel file not found: {wheel_path}")
        return False

    # Open the wheel (which is a zip file)
    with zipfile.ZipFile(wheel_path, "r") as wheel:
        file_list = wheel.namelist()

        print(f"\n📦 Wheel contents ({len(file_list)} files):")
        for file in sorted(file_list):
            print(f"  - {file}")

        # Look for the dartpy extension module
        # Extension modules can have different suffixes:
        # - .so (Linux)
        # - .pyd (Windows)
        # - .dylib (macOS, rare for Python extensions)
        # Python 3 extensions often have ABI tags like: dartpy.cpython-310-x86_64-linux-gnu.so
        dartpy_extensions = [
            f
            for f in file_list
            if "dartpy" in f.lower()
            and (f.endswith(".so") or f.endswith(".pyd") or ".so." in f)
        ]

        print(f"\n🔍 Found {len(dartpy_extensions)} dartpy extension(s):")
        for ext in dartpy_extensions:
            print(f"  ✓ {ext}")

        if not dartpy_extensions:
            print("\n❌ ERROR: No dartpy extension module found in wheel!")
            print("   The wheel appears to contain only metadata.")
            return False

        # Check that we have more than just metadata
        non_metadata_files = [
            f
            for f in file_list
            if not f.startswith("dartpy-") and not f.endswith(".dist-info/")
        ]

        if len(non_metadata_files) < 1:
            print("\n❌ ERROR: Wheel contains only metadata files!")
            return False

        print("\n✅ Wheel verification successful!")
        print(
            f"   Found dartpy extension module(s) and {len(non_metadata_files)} non-metadata files."
        )
        return True


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python verify_wheel.py <wheel_file>")
        sys.exit(1)

    wheel_path = Path(sys.argv[1])

    if not verify_wheel(wheel_path):
        sys.exit(1)

    print("\n🎉 Wheel is ready for distribution!")


if __name__ == "__main__":
    main()
