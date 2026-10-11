#!/usr/bin/env python3
"""Install a dartpy wheel in a temporary venv and run dartpy smoke tests."""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import tempfile
from pathlib import Path

SMOKE_TEST = r"""
import dartpy as dart
from dartpy.gui import InteractiveFrame
from dartpy.math import Random
from dartpy.utils import DartLoader, MjcfParser, SdfParser, SkelParser

assert dart.collision.BulletCollisionDetector() is not None
assert dart.collision.OdeCollisionDetector() is not None

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
skeleton = loader.parseSkeletonString(
    '<robot name="wheel_smoke"><link name="base"/></robot>', ""
)
assert skeleton is not None
assert skeleton.getNumBodyNodes() == 1

frame = InteractiveFrame(dart.dynamics.Frame.World())
shapes = frame.getShapeFrames()
assert isinstance(shapes, list)
assert shapes

# C++ must dispatch to Python overrides; a broken wheel silently skips them.
skeletons = [dart.dynamics.Skeleton("a"), dart.dynamics.Skeleton("b")]
for skeleton in skeletons:
    body = skeleton.createFreeJointAndBodyNodePair()[1]
    body.createShapeNode(dart.dynamics.BoxShape([1, 1, 1])).createCollisionAspect()
skeletons[1].getJoint(0).setPosition(3, 0.5)
group = dart.collision.FCLCollisionDetector().createCollisionGroup()
for skeleton in skeletons:
    group.addShapeFramesOf(skeleton)


class IgnoreAll(dart.collision.CompositeCollisionFilter):
    def ignoresCollision(self, object1, object2):
        self.calls += 1
        return True


option = dart.collision.CollisionOption(True, 100, None)
assert group.collide(option)
option.collisionFilter = IgnoreAll()
option.collisionFilter.calls = 0
assert not group.collide(option)
assert option.collisionFilter.calls > 0

assert hasattr(dart.gui, "Viewer")
node = dart.gui.WorldNode(world)
viewer = dart.gui.Viewer()
viewer.addWorldNode(node)
assert viewer.getNumWorldNodes() == 1
assert node.getWorld().getName() == "wheel smoke"

print("dartpy wheel smoke test passed")
"""


def run_command(
    cmd: list[str], *, env: dict[str, str] | None = None
) -> subprocess.CompletedProcess[str]:
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True, check=False, env=env)
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


def _smoke_environment() -> dict[str, str]:
    env = os.environ.copy()
    conda_prefix = env.get("CONDA_PREFIX")
    for name in (
        "CONDA_PREFIX",
        "CONDA_DEFAULT_ENV",
        "CONDA_PROMPT_MODIFIER",
        "PYTHONPATH",
        "PYTHONHOME",
        "LD_LIBRARY_PATH",
        "DYLD_LIBRARY_PATH",
        "DYLD_FALLBACK_LIBRARY_PATH",
        "DISPLAY",
        "WAYLAND_DISPLAY",
        "VK_DRIVER_FILES",
        "VK_ICD_FILENAMES",
        "LIBGL_DRIVERS_PATH",
        "MESA_LOADER_DRIVER_OVERRIDE",
    ):
        env.pop(name, None)
    if conda_prefix:
        prefix = Path(conda_prefix).resolve()
        env["PATH"] = os.pathsep.join(
            entry
            for entry in env.get("PATH", "").split(os.pathsep)
            if entry and not Path(entry).resolve().is_relative_to(prefix)
        )
    return env


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
        run_command(
            [str(python_path), "-I", "-c", SMOKE_TEST], env=_smoke_environment()
        )


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

    for wheel_path in wheels:
        print(f"Testing wheel: {wheel_path}")
        test_wheel(wheel_path)

    print(f"All {len(wheels)} wheel(s) tested successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
