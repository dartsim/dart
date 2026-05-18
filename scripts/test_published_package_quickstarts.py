#!/usr/bin/env python3
"""Verify first-simulation snippets against currently published packages."""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

PYTHON_SNIPPET = r"""
import dartpy as dart

if hasattr(dart, "World"):
    world = dart.World()
    skeleton_type = dart.Skeleton
else:
    world = dart.simulation.World()
    skeleton_type = dart.dynamics.Skeleton

skeleton = skeleton_type("box")
if hasattr(skeleton, "create_free_joint_and_body_node_pair"):
    skeleton.create_free_joint_and_body_node_pair()
else:
    skeleton.createFreeJointAndBodyNodePair()

if hasattr(world, "add_skeleton"):
    world.add_skeleton(skeleton)
else:
    world.addSkeleton(skeleton)

world.step()
positions = (
    skeleton.get_positions()
    if hasattr(skeleton, "get_positions")
    else skeleton.getPositions()
)
print(f"positions_size={positions.shape[0]}")
if positions.shape[0] == 0:
    raise SystemExit("expected at least one position")
"""

CPP_SOURCE = r"""
#include <dart/dart.hpp>

#include <iostream>

int main() {
  auto world = dart::simulation::World::create();
  auto skeleton = dart::dynamics::Skeleton::create("box");
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  world->addSkeleton(skeleton);
  world->step();

  const auto positions = skeleton->getPositions();
  std::cout << "positions_size=" << positions.size() << "\n";
  return positions.size() > 0 ? 0 : 1;
}
"""

CMAKE_LISTS = r"""
cmake_minimum_required(VERSION 3.22)
project(dart_published_package_quickstart LANGUAGES CXX)

find_package(DART REQUIRED CONFIG)

add_executable(dart_published_package_quickstart main.cpp)
target_link_libraries(dart_published_package_quickstart PRIVATE dart)
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--python-version",
        default="3.12",
        help="Python version to use for the temporary dartpy environment.",
    )
    parser.add_argument(
        "--skip-python",
        action="store_true",
        help="Skip the published dartpy package check.",
    )
    parser.add_argument(
        "--skip-cpp",
        action="store_true",
        help="Skip the published dartsim-cpp package check.",
    )
    return parser.parse_args()


def run(cmd: list[str], *, cwd: Path | None = None) -> None:
    print(f"Running: {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def test_python_package(python_version: str) -> None:
    if not shutil.which("uv"):
        raise RuntimeError("uv is required to create the temporary dartpy environment")

    with tempfile.TemporaryDirectory(prefix="dartpy-current-package-") as tmp:
        root = Path(tmp)
        venv = root / "venv"
        if os.name == "nt":
            python = venv / "Scripts" / "python.exe"
        else:
            python = venv / "bin" / "python"

        run(["uv", "venv", str(venv), "--python", python_version])
        run(["uv", "pip", "install", "--python", str(python), "dartpy"])
        run([str(python), "-c", PYTHON_SNIPPET])

    print("Published dartpy package quick-start passed")


def test_cpp_package() -> None:
    if not shutil.which("pixi"):
        raise RuntimeError("pixi is required to create the temporary C++ environment")

    with tempfile.TemporaryDirectory(prefix="dartsim-cpp-current-package-") as tmp:
        root = Path(tmp)
        build = root / "build"
        (root / "main.cpp").write_text(CPP_SOURCE, encoding="utf-8")
        (root / "CMakeLists.txt").write_text(CMAKE_LISTS, encoding="utf-8")

        command = " && ".join(
            [
                f"cmake -S {shlex.quote(str(root))} -B {shlex.quote(str(build))}",
                f"cmake --build {shlex.quote(str(build))} --parallel 2",
                shlex.quote(str(build / "dart_published_package_quickstart")),
            ]
        )
        run(
            [
                "pixi",
                "exec",
                "-s",
                "dartsim-cpp",
                "-s",
                "cmake",
                "-s",
                "cxx-compiler",
                "--",
                "bash",
                "-lc",
                command,
            ]
        )

    print("Published dartsim-cpp package quick-start passed")


def main() -> int:
    args = parse_args()
    if args.skip_python and args.skip_cpp:
        print("Nothing to check")
        return 0

    try:
        if not args.skip_python:
            test_python_package(args.python_version)
        if not args.skip_cpp:
            test_cpp_package()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Published package quick-start check failed: {error}", file=sys.stderr)
        return 1

    print("Published package quick-start checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
