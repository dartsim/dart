#!/usr/bin/env python3
"""
Generate Python stub files (.pyi) for dartpy module using nanobind.stubgen.

This script generates type stubs that can be used for:
1. IDE autocompletion and type checking
2. Documentation generation with Sphinx autodoc (on Read the Docs)

Usage:
    python scripts/generate_stubs.py
"""

import os
import subprocess
import sys
from pathlib import Path


def main():
    # Get the repository root directory
    repo_root = Path(__file__).parent.parent

    # Determine the dartpy module path
    # Check if PYTHONPATH is set (for local builds with compiled module)
    pythonpath = os.environ.get("PYTHONPATH", "")
    if pythonpath:
        for path in reversed(pythonpath.split(os.pathsep)):
            if path:
                sys.path.insert(0, path)

    # Verify dartpy can be imported
    try:
        import dartpy  # type: ignore

        print(f"✓ Found dartpy at: {dartpy.__file__}")
    except ImportError as e:
        print(f"ERROR: Cannot import dartpy: {e}")
        print("Please build dartpy first: pixi run build-py-dev (or ensure dartpy is installed in the current interpreter)")
        sys.exit(1)

    # Output directory for stubs
    stubs_dir = repo_root / "python" / "stubs"
    stubs_dir.mkdir(parents=True, exist_ok=True)

    dartpy_stub = stubs_dir / "dartpy" / "__init__.pyi"

    print(f"Generating stubs in: {stubs_dir}")

    # Run nanobind.stubgen
    cmd = [
        sys.executable,
        "-m",
        "nanobind.stubgen",
        "dartpy",
        "-o",
        str(dartpy_stub),
        "-r",
    ]

    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        print(f"✓ Successfully generated stubs in {stubs_dir}/dartpy")
    except subprocess.CalledProcessError as e:
        print(f"ERROR: Failed to generate stubs: {e}")
        print(e.stdout)
        print(e.stderr, file=sys.stderr)
        sys.exit(1)

    # Check that stubs were created
    if dartpy_stub.exists():
        print(f"✓ Verified stub file exists: {dartpy_stub}")
    else:
        print(f"WARNING: Expected stub file not found: {dartpy_stub}")

    print("\nStub generation complete!")
    print(f"Stubs location: {stubs_dir}")


if __name__ == "__main__":
    main()
