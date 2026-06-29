#!/usr/bin/env python3
"""Smoke-test the staged DART 7 simulation package after local install.

This is the installed-package counterpart to the static PLAN-041 promotion
checks. It configures a small source build, installs it into a temporary prefix,
then verifies that a downstream CMake project can use the staged public
simulation headers while hidden ECS/internal headers stay absent.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

from cmake_host_linker_flags import cmake_host_linker_flags

REPO_ROOT = Path(__file__).resolve().parents[1]

REQUIRED_INSTALLED_HEADERS = (
    "dart/simulation/world.hpp",
    "dart/simulation/world_options.hpp",
    "dart/simulation/body/rigid_body.hpp",
    "dart/simulation/body/rigid_body_options.hpp",
    "dart/simulation/multibody/multibody.hpp",
    "dart/simulation/compute/compute_executor.hpp",
    "dart/simulation/compute/parallel_executor.hpp",
    "dart/simulation/compute/sequential_executor.hpp",
    "dart/simulation/compute/world_step_profile.hpp",
    "dart/simulation/compute/world_step_stage.hpp",
)

FORBIDDEN_INSTALLED_HEADERS = (
    "dart/simulation/comps/rigid_body.hpp",
    "dart/simulation/ecs/component_access.hpp",
    "dart/simulation/detail/world_storage.hpp",
    "dart/simulation/io/serializer.hpp",
    "dart/simulation/space/component_mapper.hpp",
)

PRIVATE_INTERFACE_TARGET_MARKERS = (
    "EnTT::EnTT",
    "Taskflow::Taskflow",
    "Taskflow",
    "spdlog::spdlog",
    "CUDA::",
)

PRIVATE_COMPONENT_DEPENDENCY_MARKERS = (
    "EnTT",
    "entt",
    "Taskflow",
    "taskflow",
    "spdlog",
    "CUDA",
)

POSITIVE_CPP = r"""
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/world.hpp>

#include <iostream>

int main() {
  namespace sx = dart::simulation;

  sx::World world;
  auto body = world.addRigidBody("box");
  (void)body;
  world.enterSimulationMode();

  sx::compute::SequentialExecutor sequential;
  world.sync(sx::WorldSyncStage::Kinematics, sequential);

  sx::compute::ParallelExecutor parallel(1);
  world.step(parallel);

  const auto count = world.getRigidBodyCount();
  std::cout << "rigid_body_count=" << count << "\n";
  return count == 1 ? 0 : 1;
}
"""

NEGATIVE_CPP = r"""
#include <dart/simulation/comps/rigid_body.hpp>

int main() {
  return 0;
}
"""

CMAKE_LISTS = r"""
cmake_minimum_required(VERSION 4.2.3)
project(dart7_promotion_installed_package LANGUAGES CXX)

find_package(DART 7.0.0 REQUIRED COMPONENTS simulation CONFIG)

add_executable(dart7_promotion_installed_package main.cpp)
target_link_libraries(
  dart7_promotion_installed_package
  PRIVATE dart-simulation
)
"""


@dataclass(frozen=True)
class Violation:
    path: str
    message: str


def relative_to_prefix(path: Path, prefix: Path) -> str:
    try:
        return path.relative_to(prefix).as_posix()
    except ValueError:
        return path.as_posix()


def cmake_property_values(text: str, property_name: str) -> list[str]:
    """Return simple exported CMake target property values."""
    values: list[str] = []
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        line = lines[index]
        if property_name not in line:
            index += 1
            continue

        value = line.split(property_name, 1)[1].strip()
        if not value and index + 1 < len(lines):
            index += 1
            value = lines[index].strip()

        if value.startswith('"'):
            while not value.rstrip().endswith('"') and index + 1 < len(lines):
                index += 1
                value = f"{value}\n{lines[index].strip()}"
            value = value.strip()
            if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
                value = value[1:-1]

        values.append(value)
        index += 1
    return values


def component_dependency_lines(text: str) -> list[str]:
    return [
        line.strip()
        for line in text.splitlines()
        if "_DEPENDENCIES" in line or "external_dep" in line or "dart_find_" in line
    ]


def run(
    cmd: list[str],
    *,
    cwd: Path,
    env: dict[str, str] | None = None,
    expect_success: bool = True,
) -> subprocess.CompletedProcess[str]:
    print(f"Running: {' '.join(cmd)}", flush=True)
    result = subprocess.run(
        cmd,
        cwd=cwd,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if expect_success and result.returncode != 0:
        print(result.stdout)
        raise subprocess.CalledProcessError(result.returncode, cmd, result.stdout)
    if not expect_success and result.returncode == 0:
        raise RuntimeError(f"Command unexpectedly succeeded: {' '.join(cmd)}")
    if result.stdout:
        print(result.stdout)
    return result


def parallel_jobs() -> int:
    env_value = os.environ.get("DART_PARALLEL_JOBS")
    if env_value:
        try:
            return max(1, int(env_value))
        except ValueError:
            pass
    cpus = os.cpu_count() or 1
    return max(1, min(8, cpus // 4 if cpus > 4 else cpus))


def configure_build(
    source_dir: Path, build_dir: Path, prefix: Path, build_type: str
) -> None:
    conda_prefix = os.environ.get("CONDA_PREFIX")
    prefix_path = os.pathsep.join(
        str(path)
        for path in (conda_prefix, os.environ.get("CMAKE_PREFIX_PATH"))
        if path
    )
    cmd = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        str(source_dir),
        "-B",
        str(build_dir),
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_PREFIX_PATH={prefix_path}",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_GUI=OFF",
        "-DDART_BUILD_EXAMPLES=OFF",
        "-DDART_BUILD_TUTORIALS=OFF",
        "-DDART_BUILD_TESTS=OFF",
        "-DDART_ENABLE_SDFORMAT=OFF",
        "-DDART_ENABLE_EXPERIMENTAL_CUDA=OFF",
        "-DDART_BUILD_DIFF=OFF",
        "-DDART_BUILD_PROFILE=OFF",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_NANOBIND=OFF",
        "-DDART_USE_SYSTEM_TRACY=OFF",
        "-DDART_VERBOSE=OFF",
    ]
    cmd.extend(cmake_host_linker_flags())
    if shutil.which("sccache") and os.environ.get("SCCACHE_GHA_ENABLED") != "false":
        cmd.extend(
            [
                "-DCMAKE_C_COMPILER_LAUNCHER=sccache",
                "-DCMAKE_CXX_COMPILER_LAUNCHER=sccache",
            ]
        )
    run(cmd, cwd=source_dir)


def build_and_install(build_dir: Path, build_type: str) -> None:
    run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--config",
            build_type,
            "--target",
            "install",
            "--parallel",
            str(parallel_jobs()),
        ],
        cwd=REPO_ROOT,
    )


def inspect_installed_tree(
    prefix: Path, *, strict_final: bool = False
) -> list[Violation]:
    violations: list[Violation] = []
    include_dir = prefix / "include"
    cmake_dir = prefix / "share" / "dart" / "cmake"

    for relpath in REQUIRED_INSTALLED_HEADERS:
        if not (include_dir / relpath).is_file():
            violations.append(Violation(relpath, "required promoted header is missing"))

    for relpath in FORBIDDEN_INSTALLED_HEADERS:
        if (include_dir / relpath).exists():
            violations.append(
                Violation(relpath, "internal header must not be installed")
            )

    component_file = cmake_dir / "dart_simulationComponent.cmake"
    if not component_file.is_file():
        violations.append(
            Violation(
                relative_to_prefix(component_file, prefix),
                "simulation component file is missing while staged",
            )
        )
    else:
        text = component_file.read_text(encoding="utf-8", errors="replace")
        dependency_text = "\n".join(component_dependency_lines(text))
        for marker in PRIVATE_COMPONENT_DEPENDENCY_MARKERS:
            if marker in dependency_text:
                violations.append(
                    Violation(
                        relative_to_prefix(component_file, prefix),
                        "private implementation dependency leaked into "
                        f"simulation component dependencies: {marker}",
                    )
                )

    target_file = cmake_dir / "dart_simulationTargets.cmake"
    if not target_file.is_file():
        violations.append(
            Violation(
                relative_to_prefix(target_file, prefix),
                "simulation target export file is missing while staged",
            )
        )

    for cmake_file in sorted(cmake_dir.glob("dart_simulationTargets*.cmake")):
        text = cmake_file.read_text(encoding="utf-8", errors="replace")
        interface_values = cmake_property_values(text, "INTERFACE_LINK_LIBRARIES")
        for value in interface_values:
            for marker in PRIVATE_INTERFACE_TARGET_MARKERS:
                if marker in value:
                    violations.append(
                        Violation(
                            relative_to_prefix(cmake_file, prefix),
                            "private implementation dependency leaked into "
                            f"installed target interface: {marker}",
                        )
                    )

    if strict_final:
        for cmake_file in sorted(cmake_dir.glob("*.cmake")):
            text = cmake_file.read_text(encoding="utf-8", errors="replace")
            if "simulation" not in text:
                continue
            violations.append(
                Violation(
                    relative_to_prefix(cmake_file, prefix),
                    "final promotion must remove simulation package names",
                )
            )

    return violations


def runtime_env(prefix: Path) -> dict[str, str]:
    env = os.environ.copy()
    conda_prefix = os.environ.get("CONDA_PREFIX")
    prefix_entries = [str(prefix)]
    if conda_prefix:
        prefix_entries.append(conda_prefix)
    existing_prefix_path = env.get("CMAKE_PREFIX_PATH")
    if existing_prefix_path:
        prefix_entries.extend(existing_prefix_path.split(os.pathsep))
    env["CMAKE_PREFIX_PATH"] = os.pathsep.join(dict.fromkeys(prefix_entries))

    lib_dirs = [prefix / "lib", prefix / "lib64"]
    if conda_prefix:
        conda_prefix_path = Path(conda_prefix)
        lib_dirs.extend([conda_prefix_path / "lib", conda_prefix_path / "lib64"])
    existing = None
    if sys.platform.startswith("linux"):
        runtime_var = "LD_LIBRARY_PATH"
        existing = env.get(runtime_var)
    elif sys.platform == "darwin":
        runtime_var = "DYLD_LIBRARY_PATH"
        existing = env.get(runtime_var)
    elif os.name == "nt":
        runtime_var = "PATH"
        existing = env.get(runtime_var)
        lib_dirs.append(prefix / "bin")
    else:
        return env

    paths = [str(path) for path in lib_dirs if path.exists()]
    if existing:
        paths.append(existing)
    if paths:
        env[runtime_var] = os.pathsep.join(paths)
    return env


def downstream_executable(build_dir: Path, build_type: str) -> Path:
    name = (
        "dart7_promotion_installed_package.exe"
        if os.name == "nt"
        else "dart7_promotion_installed_package"
    )
    candidates = [build_dir / build_type / name, build_dir / name]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def run_downstream_project(prefix: Path, source: str, *, expect_success: bool) -> None:
    with tempfile.TemporaryDirectory(prefix="dart7-promotion-downstream-") as tmp:
        root = Path(tmp)
        source_dir = root / "src"
        build_dir = root / "build"
        source_dir.mkdir()
        build_dir.mkdir()
        (source_dir / "main.cpp").write_text(source, encoding="utf-8")
        (source_dir / "CMakeLists.txt").write_text(CMAKE_LISTS, encoding="utf-8")

        env = runtime_env(prefix)
        run(
            [
                "cmake",
                "-S",
                str(source_dir),
                "-B",
                str(build_dir),
                "-DCMAKE_BUILD_TYPE=Release",
            ],
            cwd=root,
            env=env,
            expect_success=True,
        )
        run(
            [
                "cmake",
                "--build",
                str(build_dir),
                "--config",
                "Release",
                "--parallel",
                str(parallel_jobs()),
            ],
            cwd=root,
            env=env,
            expect_success=expect_success,
        )
        if not expect_success:
            return
        run([str(downstream_executable(build_dir, "Release"))], cwd=root, env=env)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=REPO_ROOT)
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="Build directory for the temporary install build.",
    )
    parser.add_argument("--prefix", type=Path, default=None)
    parser.add_argument("--build-type", default="Release")
    parser.add_argument(
        "--strict-final",
        action="store_true",
        help="Also fail if installed CMake files still name simulation.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_dir = args.source_dir.resolve()
    if not shutil.which("cmake"):
        print("cmake not found", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="dart7-promotion-install-") as tmp:
        temp_root = Path(tmp)
        prefix = args.prefix.resolve() if args.prefix else temp_root / "prefix"
        build_dir = args.build_dir.resolve() if args.build_dir else temp_root / "build"
        if prefix.exists():
            if prefix in (source_dir, build_dir) or prefix.parent == prefix:
                raise RuntimeError(f"refusing to clean unsafe prefix: {prefix}")
            shutil.rmtree(prefix)

        configure_build(source_dir, build_dir, prefix, args.build_type)
        build_and_install(build_dir, args.build_type)

        violations = inspect_installed_tree(prefix, strict_final=args.strict_final)
        if violations:
            for violation in violations:
                print(f"{violation.path}: {violation.message}", file=sys.stderr)
            return 1

        run_downstream_project(prefix, POSITIVE_CPP, expect_success=True)
        try:
            run_downstream_project(prefix, NEGATIVE_CPP, expect_success=False)
        except RuntimeError as error:
            print(error, file=sys.stderr)
            return 1

    print("DART 7 simulation installed-package smoke passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
