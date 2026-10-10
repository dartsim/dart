"""Exercise the actual CMake compiler guards without native cross-toolchains."""

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def configure(tmp_path, compiler, version, *, msvc=False, wheels=False, xcode=None):
    source = tmp_path / "source"
    source.mkdir()
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\n"
        "project(compiler_requirements NONE)\n"
        f'include("{ROOT.as_posix()}/cmake/dart_defs.cmake")\n'
        f'set(CMAKE_CXX_COMPILER_ID "{compiler}")\n'
        f'set(CMAKE_CXX_COMPILER_VERSION "{version}")\n'
        f"set(DART_BUILD_WHEELS {'ON' if wheels else 'OFF'})\n"
        + (f'set(XCODE_VERSION "{xcode}")\n' if xcode is not None else "")
        + "dart_check_compiler_version()\n"
        + (
            "set(MSVC TRUE)\n"
            f"set(MSVC_VERSION {version if compiler == 'MSVC' else '1920'})\n"
            "dart_configure_msvc_toolchain(REQUIRED_VERSION 1930 "
            'REQUIRED_LABEL "Visual Studio 2022 v143")\n'
            if msvc
            else ""
        ),
        encoding="utf-8",
    )
    return subprocess.run(
        ["cmake", "-S", str(source), "-B", str(tmp_path / "build")],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )


@pytest.mark.parametrize(
    "compiler,version,accepted",
    [
        ("GNU", "11.1.0", False),
        ("GNU", "11.2.0", True),
        ("GNU", "16.0.0", True),
        ("Clang", "12.0.1", False),
        ("Clang", "13.0.0", True),
        ("Clang", "19.0.0", True),
        ("AppleClang", "13.1.6", False),
        ("AppleClang", "14.0.0", True),
        ("AppleClang", "17.0.0", True),
        ("MSVC", "1929", False),
        ("MSVC", "1930", True),
        ("MSVC", "1944", True),
    ],
)
def test_compiler_version_boundary(tmp_path, compiler, version, accepted):
    result = configure(tmp_path, compiler, version, msvc=compiler == "MSVC")
    assert (result.returncode == 0) == accepted, result.stdout + result.stderr
    if not accepted:
        assert "requires" in result.stderr


@pytest.mark.parametrize("compiler", ["GNU", "Clang", "AppleClang"])
def test_missing_compiler_version_is_rejected(tmp_path, compiler):
    result = configure(tmp_path, compiler, "")
    assert result.returncode != 0
    assert "requires" in result.stderr


def test_wheel_build_requires_same_gcc_floor(tmp_path):
    result = configure(tmp_path, "GNU", "10.2.1", wheels=True)
    assert result.returncode != 0
    assert "GCC 11.2.0" in result.stderr


@pytest.mark.parametrize("version,accepted", [("14.0", False), ("14.1", True)])
def test_xcode_version_boundary_when_reported_by_generator(tmp_path, version, accepted):
    result = configure(tmp_path, "AppleClang", "14.0.0", xcode=version)
    assert (result.returncode == 0) == accepted, result.stdout + result.stderr
    if not accepted:
        assert "Xcode 14.1" in result.stderr


@pytest.mark.parametrize("version,accepted", [("12.0.0", False), ("13.0.0", True)])
def test_clang_cl_uses_clang_floor_not_msvc_emulation_version(
    tmp_path, version, accepted
):
    result = configure(tmp_path, "Clang", version, msvc=True)
    assert (result.returncode == 0) == accepted, result.stdout + result.stderr
