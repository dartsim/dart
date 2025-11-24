#!/usr/bin/env python3

"""
Test dartpy wheel installation and basic functionality.

This script creates a temporary virtual environment, installs a dartpy wheel,
and runs basic functionality tests to ensure the wheel works correctly.

Usage:
    python scripts/test_wheel.py dist/dartpy-*.whl
    python scripts/test_wheel.py dist/dartpy-0.2.0-cp312-cp312-linux_x86_64.whl
"""

import argparse
import glob
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def run_command(cmd, check=True):
    """Run a shell command and return the result."""
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if check and result.returncode != 0:
        print(f"Error: Command failed with return code {result.returncode}")
        print(f"stdout: {result.stdout}")
        print(f"stderr: {result.stderr}")
        sys.exit(result.returncode)
    return result


def test_wheel(wheel_path):
    """Test a dartpy wheel by installing it in a temporary environment."""
    wheel_path = Path(wheel_path).resolve()
    if not wheel_path.exists():
        print(f"Error: Wheel file not found: {wheel_path}")
        sys.exit(1)

    # Extract Python version from wheel filename
    # Format: dartpy-VERSION-cpXXX-cpXXX-PLATFORM.whl
    wheel_name = wheel_path.name
    parts = wheel_name.split("-")
    if len(parts) < 3:
        print(f"Error: Invalid wheel filename format: {wheel_name}")
        sys.exit(1)

    # Extract cpXXX (e.g., cp312)
    python_tag = parts[2]
    if not python_tag.startswith("cp"):
        print(f"Error: Could not extract Python version from wheel: {wheel_name}")
        sys.exit(1)

    # Convert cp312 -> 3.12
    python_version = f"{python_tag[2]}.{python_tag[3:]}"

    print(f"\n{'='*60}")
    print(f"Testing wheel: {wheel_path.name}")
    print(f"Python version: {python_version}")
    print(f"{'='*60}\n")

    # Create temporary directory for virtual environment
    with tempfile.TemporaryDirectory() as tmpdir:
        venv_path = Path(tmpdir) / "test-env"

        # Determine the correct virtualenv layout based on platform
        # Windows: Scripts/python.exe, Unix: bin/python
        if sys.platform == "win32":
            python_path = venv_path / "Scripts" / "python.exe"
        else:
            python_path = venv_path / "bin" / "python"

        print(f"Creating virtual environment at {venv_path}...")
        run_command(["uv", "venv", str(venv_path), "--python", python_version])

        print(f"\nInstalling wheel...")
        run_command(
            ["uv", "pip", "install", "--python", str(python_path), str(wheel_path)]
        )

        print(f"\nTesting dartpy import...")
        result = run_command(
            [
                str(python_path),
                "-c",
                "import dartpy; print(f'✓ dartpy {dartpy.__version__} imported successfully')",
            ]
        )
        print(result.stdout.strip())

        print(f"\nTesting dartpy submodules...")
        result = run_command(
            [
                str(python_path),
                "-c",
                """
import dartpy
import sys

# Core submodules that should always be available
core_modules = [
    'collision',
    'common',
    'constraint',
    'dynamics',
    'math',
    'simulation',
    'utils',
]

# Optional submodules (may not be available on all platforms)
optional_modules = [
    'gui',  # May not be available if OSG is disabled
]

print('Testing core submodules:')
failed = []
for module_name in core_modules:
    try:
        module = getattr(dartpy, module_name)
        print(f'  ✓ dartpy.{module_name}')
    except AttributeError as e:
        print(f'  ✗ dartpy.{module_name} - MISSING!')
        failed.append(module_name)

print('\\nTesting optional submodules:')
for module_name in optional_modules:
    try:
        module = getattr(dartpy, module_name)
        print(f'  ✓ dartpy.{module_name}')
        if module_name == 'gui':
            marker = '✓' if hasattr(module, 'Viewer') else '⚠'
            detail = 'Viewer available' if marker == '✓' else 'Viewer missing'
            print(f'    {marker} dartpy.gui ({detail})')
    except AttributeError as e:
        print(f'  ⚠ dartpy.{module_name} not available (optional)')

if failed:
    print(f'\\n✗ ERROR: Missing required modules: {failed}')
    sys.exit(1)
else:
    print('\\n✓ All required submodules available')
""",
            ],
        )
        print(result.stdout.strip())

        print(f"\nTesting basic functionality...")
        result = run_command(
            [
                str(python_path),
                "-c",
                "import dartpy.dynamics; world = dartpy.simulation.World(); print('✓ Basic functionality works')",
            ]
        )
        print(result.stdout.strip())

    print(f"\n{'='*60}")
    print(f"✓ All tests passed for {wheel_path.name}")
    print(f"{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(description="Test dartpy wheel installation")
    parser.add_argument(
        "wheel",
        nargs="?",
        help="Path to wheel file or pattern (e.g., dist/dartpy-*.whl)",
    )
    parser.add_argument(
        "--all", action="store_true", help="Test all wheels in dist/ directory"
    )

    args = parser.parse_args()

    # Determine which wheels to test
    if args.all:
        wheels = sorted(glob.glob("dist/dartpy-*.whl"))
        if not wheels:
            print("Error: No wheels found in dist/ directory")
            sys.exit(1)
    elif args.wheel:
        # Support glob patterns
        if "*" in args.wheel or "?" in args.wheel:
            wheels = sorted(glob.glob(args.wheel))
            if not wheels:
                print(f"Error: No wheels found matching pattern: {args.wheel}")
                sys.exit(1)
        else:
            wheels = [args.wheel]
    else:
        # Default: test all wheels in dist/
        wheels = sorted(glob.glob("dist/dartpy-*.whl"))
        if not wheels:
            print("Error: No wheels specified and no wheels found in dist/")
            print("Usage: python scripts/test_wheel.py WHEEL_PATH")
            sys.exit(1)

    # Test each wheel
    for wheel in wheels:
        try:
            test_wheel(wheel)
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            sys.exit(1)
        except Exception as e:
            print(f"\n\nError testing {wheel}: {e}")
            sys.exit(1)

    print(f"\n{'='*60}")
    print(f"✓ All {len(wheels)} wheel(s) tested successfully!")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
