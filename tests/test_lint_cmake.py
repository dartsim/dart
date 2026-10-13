"""Guard CMake lint inputs and the FreeBSD source compatibility patches."""

import importlib.util
import shutil
import subprocess
from pathlib import Path

import pytest

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


@pytest.mark.skipif(shutil.which("patch") is None, reason="patch tool unavailable")
def test_freebsd_patches_apply_to_current_sources(tmp_path):
    for patch in sorted((ROOT / "tools/freebsd/patches").glob("patch-*")):
        for line in patch.read_text().splitlines():
            if line.startswith("+++ "):
                relative = line[4:].split()[0]
                target = tmp_path / relative
                if not target.exists():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(ROOT / relative, target)
        result = subprocess.run(
            ["patch", "-F", "0", "-p0", "-N", "-i", str(patch)],
            cwd=tmp_path,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr
