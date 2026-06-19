"""Regression checks for the default and CUDA py-demos Pixi build paths."""

from __future__ import annotations

import tomllib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PIXI = ROOT / "pixi.toml"
CMAKE_COMPILER_CACHE = ROOT / "cmake" / "compiler_cache.cmake"


def _tasks() -> dict:
    return tomllib.loads(PIXI.read_text(encoding="utf-8"))["tasks"]


def _task_script(task_name: str) -> str:
    task = _tasks()[task_name]
    return "\n".join(str(part) for part in task["cmd"])


def test_config_py_honors_dartpy_on_for_cuda_py_demos() -> None:
    script = _task_script("config-py")

    assert (
        'if [ "${DART_BUILD_DARTPY_VALUE}" = "OFF" ] '
        '&& [ "{{ dartpy }}" = "ON" ]; then'
    ) in script
    assert "DART_BUILD_DARTPY_VALUE=ON" in script


def test_config_py_resets_stale_cuda_compiler_cache_before_cmake() -> None:
    script = _task_script("config-py")

    reset_index = script.index("CMake compiler cache changed")
    cmake_index = script.index("cmake -G Ninja")
    assert reset_index < cmake_index

    assert "CMAKE_BUILD_DIR=build/$PIXI_ENVIRONMENT_NAME/cpp/${BUILD_DIR_NAME}" in script
    assert "CMAKE_C_COMPILER:*)" in script
    assert "CMAKE_CXX_COMPILER:*)" in script
    assert "CMAKE_CUDA_COMPILER:*)" in script
    assert "CMAKE_CUDA_HOST_COMPILER:*)" in script
    assert "CMAKE_CUDA_COMPILER_LAUNCHER:*)" in script
    assert '[ -z "${CMAKE_CUDA_COMPILER_LAUNCHER:-}" ]' in script
    assert '[ -n "${cached_cuda_compiler_launcher}" ]' in script
    assert 'rm -f "${CMAKE_CACHE}"' in script
    assert 'rm -rf "${CMAKE_BUILD_DIR}/CMakeFiles"' in script


def test_config_py_keeps_compiler_cache_for_default_and_cuda_py_demos() -> None:
    script = _task_script("config-py")

    disable_assignment = script.index(
        "DART_DISABLE_COMPILER_CACHE_VALUE=${DART_DISABLE_COMPILER_CACHE:-OFF}"
    )
    gha_disable = script.index('if [ "${SCCACHE_GHA_ENABLED:-}" = "false" ]; then')
    default_cache_section = script[disable_assignment:gha_disable]
    assert "DART_DISABLE_COMPILER_CACHE_VALUE=ON" not in default_cache_section

    cxx_launcher = script.index("CMAKE_CXX_COMPILER_LAUNCHER=sccache")
    cuda_launcher = script.index(
        'LAUNCHER_DEFS="$LAUNCHER_DEFS -DCMAKE_CUDA_COMPILER_LAUNCHER:STRING="'
    )
    assert cxx_launcher < cuda_launcher


def test_cmake_compiler_cache_respects_empty_cuda_launcher() -> None:
    script = CMAKE_COMPILER_CACHE.read_text(encoding="utf-8")

    preconfigured_check = script.index("DEFINED CMAKE_CUDA_COMPILER_LAUNCHER")
    cuda_launcher_clear = script.index(
        'CMAKE_CUDA_COMPILER_LAUNCHER\n          ""'
    )
    assert preconfigured_check < cuda_launcher_clear
    assert "NOT _dart_cuda_compiler_launcher_preconfigured" in script
    assert 'CMAKE_CUDA_COMPILER_LAUNCHER\n          "${_cache_executable}"' not in script


def test_py_demos_depends_on_docking_dartpy_build() -> None:
    tasks = _tasks()

    assert tasks["py-demos"]["depends-on"] == ["build-py-dev-docking"]

    build_task = tasks["build-py-dev-docking"]
    defaults = {arg["arg"]: arg["default"] for arg in build_task["args"]}
    assert defaults["dartpy"] == "ON"
    assert defaults["build_dir"] == "Release-docking"

    config_dependency = build_task["depends-on"][0]
    assert config_dependency["task"] == "config-py"
    assert config_dependency["args"] == [
        "{{ dartpy }}",
        "{{ build_type }}",
        "OFF",
        "{{ build_dir }}",
    ]
