#!/usr/bin/env python3
"""Install a dartpy wheel in a temporary venv and run DART 6 smoke tests."""

from __future__ import annotations

import argparse
import glob
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SMOKE_TEST = r"""
import dartpy as dart
from dartpy.math import Random
from dartpy.utils import DartLoader

world = dart.simulation.World("wheel smoke")
assert world.getNumSkeletons() == 0
assert world.getNumSimpleFrames() == 0
assert world.getConstraintSolver() is not None

Random.setSeed(7)
assert Random.getSeed() == 7
value = Random.uniform(-1.0, 1.0)
assert -1.0 <= value <= 1.0

loader = DartLoader()
assert loader.parseSkeleton("missing-file-for-wheel-smoke.urdf") is None

print("dartpy wheel smoke test passed")
"""


def run_command(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    if result.returncode != 0:
        raise subprocess.CalledProcessError(
            result.returncode,
            cmd,
            output=result.stdout,
            stderr=result.stderr,
        )
    return result


def _venv_python(venv_path: Path) -> Path:
    if sys.platform == "win32":
        return venv_path / "Scripts" / "python.exe"
    return venv_path / "bin" / "python"


def test_wheel(wheel_path: Path) -> None:
    wheel_path = wheel_path.resolve()
    if not wheel_path.exists():
        raise FileNotFoundError(f"Wheel file not found: {wheel_path}")

    with tempfile.TemporaryDirectory() as tmpdir:
        venv_path = Path(tmpdir) / "test-env"
        python_path = _venv_python(venv_path)

        print(f"Creating virtual environment at {venv_path}")
        run_command([sys.executable, "-m", "venv", str(venv_path)])

        run_command(
            [
                str(python_path),
                "-m",
                "pip",
                "install",
                "--upgrade",
                "pip",
            ]
        )
        run_command([str(python_path), "-m", "pip", "install", str(wheel_path)])
        run_command([str(python_path), "-c", SMOKE_TEST])


def _expand_wheels(patterns: list[str]) -> list[Path]:
    wheels: list[Path] = []
    for pattern in patterns:
        matches = sorted(Path(path) for path in glob.glob(pattern))
        if matches:
            wheels.extend(matches)
        else:
            wheels.append(Path(pattern))
    return wheels


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", nargs="*", help="Wheel path or glob pattern")
    parser.add_argument("--all", action="store_true", help="Test all dist wheels")
    args = parser.parse_args(argv[1:])

    if args.all:
        wheels = sorted(Path(path) for path in glob.glob("dist/dartpy-*.whl"))
    elif args.wheel:
        wheels = _expand_wheels(args.wheel)
    else:
        wheels = sorted(Path(path) for path in glob.glob("dist/dartpy-*.whl"))

    if not wheels:
        print("ERROR: No wheels found.")
        return 1

    if shutil.which("python") is None:
        print("ERROR: python is not available on PATH.")
        return 1

    for wheel_path in wheels:
        print(f"Testing wheel: {wheel_path}")
        test_wheel(wheel_path)

    print(f"All {len(wheels)} wheel(s) tested successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
