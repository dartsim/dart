"""Guard which CMake files the gersemi lint formats."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_find_cmake_files_skips_hidden_and_excluded_trees(tmp_path):
    for relative in (
        "CMakeLists.txt",
        "cmake/DARTMacros.cmake",
        "cmake/DARTConfig.cmake.in",
        "dart/CMakeLists.txt",
        ".private/evidence/CMakeLists.txt",
        ".pixi/envs/default/lib/cmake/foo.cmake",
        "build/default/CMakeFiles/bar.cmake",
        "external/imgui/CMakeLists.txt",
        "tools/freebsd/patches/patch-cmake_DARTMacros.cmake",
    ):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("", encoding="utf-8")
    spec = importlib.util.spec_from_file_location(
        "lint_cmake", ROOT / "scripts/lint_cmake.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module.find_cmake_files(tmp_path) == [
        Path("CMakeLists.txt"),
        Path("cmake/DARTMacros.cmake"),
        Path("dart/CMakeLists.txt"),
    ]
