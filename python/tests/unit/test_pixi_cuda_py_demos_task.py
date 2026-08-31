"""Regression checks for the default and CUDA py-demos Pixi build paths."""

from __future__ import annotations

import tomllib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PIXI = ROOT / "pixi.toml"
CMAKE_COMPILER_CACHE = ROOT / "cmake" / "compiler_cache.cmake"


def _tasks() -> dict:
    return tomllib.loads(PIXI.read_text(encoding="utf-8"))["tasks"]


def test_config_py_delegates_to_cmake_config_script() -> None:
    """config-py is a thin wrapper over scripts/cmake_config.py.

    The behaviors this file used to grep out of the embedded bash block —
    dartpy=ON beating the cuda environment's OFF override, the stale CUDA
    compiler-cache reset before cmake runs, and keeping the C/CXX compiler
    cache while pinning the CUDA launcher empty — are now guarded
    behaviorally in tests/test_cmake_config.py against the script itself.
    """
    task = _tasks()["config-py"]
    cmd = [str(part) for part in task["cmd"]]

    assert cmd[:3] == ["python", "scripts/cmake_config.py", "config-py"]
    assert "--dartpy={{ dartpy }}" in cmd
    assert "--build-type={{ build_type }}" in cmd
    assert "--use-system-imgui={{ use_system_imgui }}" in cmd
    assert "--build-dir={{ build_dir }}" in cmd


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
