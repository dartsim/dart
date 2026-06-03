"""Regression checks for the CUDA py-demos Pixi build path."""

from __future__ import annotations

import tomllib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PIXI = ROOT / "pixi.toml"


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
    assert 'rm -f "${CMAKE_CACHE}"' in script
    assert 'rm -rf "${CMAKE_BUILD_DIR}/CMakeFiles"' in script


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
