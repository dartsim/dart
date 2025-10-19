#!/usr/bin/env python3
"""
Generate Python stub files (.pyi) for dartpy module using pybind11-stubgen.

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
    if not pythonpath:
        print("ERROR: PYTHONPATH is not set. Please build dartpy first.")
        print("Run: pixi run build-py-dev")
        sys.exit(1)

    # Add dartpy to Python path
    sys.path.insert(0, pythonpath)

    # Verify dartpy can be imported
    try:
        import dartpy  # type: ignore

        print(f"✓ Found dartpy at: {dartpy.__file__}")
    except ImportError as e:
        print(f"ERROR: Cannot import dartpy: {e}")
        print("Please build dartpy first: pixi run build-py-dev")
        sys.exit(1)

    # Output directory for stubs
    stubs_dir = repo_root / "python" / "stubs"
    stubs_dir.mkdir(parents=True, exist_ok=True)

    print(f"Generating stubs in: {stubs_dir}")

    # Run pybind11-stubgen
    cmd = [
        "pybind11-stubgen",
        "dartpy",
        "-o",
        str(stubs_dir),
        "--numpy-array-use-type-var",
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
    dartpy_stub = stubs_dir / "dartpy" / "__init__.pyi"
    if dartpy_stub.exists():
        print(f"✓ Verified stub file exists: {dartpy_stub}")
    else:
        print(f"WARNING: Expected stub file not found: {dartpy_stub}")

    print("\nStub generation complete!")
    print(f"Stubs location: {stubs_dir}")


if __name__ == "__main__":
    main()
