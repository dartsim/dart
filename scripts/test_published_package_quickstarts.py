#!/usr/bin/env python3
"""Verify first-simulation snippets against currently published packages."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

PYTHON_SNIPPET = r"""
import dartpy as dart

world = dart.World()
world.add_rigid_body("box")
world.step()
print(f"rigid_body_count={world.num_rigid_bodies}")
if world.num_rigid_bodies != 1:
    raise SystemExit("expected one rigid body")
"""

CPP_SOURCE = r"""
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/world.hpp>

#include <iostream>

int main() {
  dart::simulation::World world;
  auto body = world.addRigidBody("box");
  (void)body;
  world.step();

  const auto count = world.getRigidBodyCount();
  std::cout << "rigid_body_count=" << count << "\n";
  return count == 1 ? 0 : 1;
}
"""

CMAKE_LISTS = r"""
cmake_minimum_required(VERSION 4.2.3)
project(dart_published_package_quickstart LANGUAGES CXX)

find_package(DART REQUIRED COMPONENTS simulation CONFIG)

add_executable(dart_published_package_quickstart main.cpp)
target_link_libraries(dart_published_package_quickstart PRIVATE dart-simulation)
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--python-version",
        default="3.14",
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


def executable_path(build_dir: Path, build_type: str) -> Path:
    name = (
        "dart_published_package_quickstart.exe"
        if os.name == "nt"
        else "dart_published_package_quickstart"
    )
    candidates = [
        build_dir / build_type / name,
        build_dir / name,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


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

    build_type = "Release"
    with tempfile.TemporaryDirectory(prefix="dartsim-cpp-current-package-") as tmp:
        root = Path(tmp)
        build = root / "build"
        (root / "main.cpp").write_text(CPP_SOURCE, encoding="utf-8")
        (root / "CMakeLists.txt").write_text(CMAKE_LISTS, encoding="utf-8")

        pixi_env = [
            "pixi",
            "exec",
            "-s",
            "dartsim-cpp",
            "-s",
            "cmake",
            "-s",
            "cxx-compiler",
        ]
        run(
            [
                *pixi_env,
                "--",
                "cmake",
                "-S",
                str(root),
                "-B",
                str(build),
                f"-DCMAKE_BUILD_TYPE={build_type}",
            ]
        )
        run(
            [
                *pixi_env,
                "--",
                "cmake",
                "--build",
                str(build),
                "--config",
                build_type,
                "--parallel",
                "2",
            ]
        )
        run([*pixi_env, "--", str(executable_path(build, build_type))])

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
