#!/usr/bin/env python3
"""Build and run the README C++ quick-start against an installed DART prefix."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

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
  if (count != 1) {
    std::cerr << "Expected one rigid body\n";
    return 1;
  }

  return 0;
}
"""


CMAKE_LISTS = r"""
cmake_minimum_required(VERSION 4.2.3)
project(dart_readme_cpp_quickstart LANGUAGES CXX)

find_package(DART 7.0.0 REQUIRED COMPONENTS simulation CONFIG)

add_executable(dart_readme_cpp_quickstart main.cpp)
target_link_libraries(
  dart_readme_cpp_quickstart
  PRIVATE
    dart-simulation
)
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--prefix",
        action="append",
        default=[],
        help="Installed DART or dependency prefix to prepend to CMAKE_PREFIX_PATH.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type for single-config generators.",
    )
    return parser.parse_args()


def run(cmd: list[str], *, cwd: Path, env: dict[str, str]) -> None:
    print(f"Running: {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, cwd=cwd, env=env, check=True)


def merged_prefix_path(prefixes: list[str]) -> str:
    entries = [str(Path(prefix).resolve()) for prefix in prefixes if prefix]
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        entries.append(conda_prefix)
    existing = os.environ.get("CMAKE_PREFIX_PATH")
    if existing:
        entries.extend(existing.split(os.pathsep))
    return os.pathsep.join(dict.fromkeys(entries))


def effective_prefixes(prefixes: list[str]) -> list[str]:
    entries = [prefix for prefix in prefixes if prefix]
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        entries.append(conda_prefix)
    return list(dict.fromkeys(entries))


def prepend_runtime_paths(env: dict[str, str], prefixes: list[str]) -> None:
    lib_dirs = []
    for prefix in effective_prefixes(prefixes):
        path = Path(prefix).resolve()
        lib_dirs.extend([path / "lib", path / "lib64"])
    lib_dirs = [path for path in lib_dirs if path.exists()]
    if not lib_dirs:
        return

    if sys.platform.startswith("linux"):
        name = "LD_LIBRARY_PATH"
    elif sys.platform == "darwin":
        name = "DYLD_LIBRARY_PATH"
    elif os.name == "nt":
        name = "PATH"
    else:
        return

    current = env.get(name)
    values = [str(path) for path in lib_dirs]
    if current:
        values.append(current)
    env[name] = os.pathsep.join(values)


def executable_path(build_dir: Path, build_type: str) -> Path:
    name = (
        "dart_readme_cpp_quickstart.exe"
        if os.name == "nt"
        else "dart_readme_cpp_quickstart"
    )
    candidates = [
        build_dir / build_type / name,
        build_dir / name,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def main() -> int:
    args = parse_args()
    if not shutil.which("cmake"):
        print("cmake not found", file=sys.stderr)
        return 1

    env = os.environ.copy()
    cmake_prefix_path = merged_prefix_path(args.prefix)
    if cmake_prefix_path:
        env["CMAKE_PREFIX_PATH"] = cmake_prefix_path
    prepend_runtime_paths(env, args.prefix)

    with tempfile.TemporaryDirectory(prefix="dart-cpp-quickstart-") as tmp:
        root = Path(tmp)
        source_dir = root / "src"
        build_dir = root / "build"
        source_dir.mkdir()
        build_dir.mkdir()
        (source_dir / "main.cpp").write_text(CPP_SOURCE, encoding="utf-8")
        (source_dir / "CMakeLists.txt").write_text(CMAKE_LISTS, encoding="utf-8")

        configure = [
            "cmake",
            "-S",
            str(source_dir),
            "-B",
            str(build_dir),
            f"-DCMAKE_BUILD_TYPE={args.build_type}",
        ]
        run(configure, cwd=root, env=env)
        run(
            [
                "cmake",
                "--build",
                str(build_dir),
                "--config",
                args.build_type,
                "--parallel",
                "2",
            ],
            cwd=root,
            env=env,
        )

        run([str(executable_path(build_dir, args.build_type))], cwd=root, env=env)

    print("C++ quick-start package check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
