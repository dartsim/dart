"""Exercise the actual CMake compiler guards without native cross-toolchains."""

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def configure(
    tmp_path, compiler, version, *, msvc=False, wheels=False, xcode=None, extra_cmake=""
):
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
        )
        + extra_cmake,
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


@pytest.mark.parametrize(
    ("compiler", "version", "msvc", "system"),
    (
        ("GNU", "11.2.0", False, "Linux"),
        ("GNU", "13.2.0", False, "Linux"),
        ("Clang", "13.0.0", False, "Linux"),
        ("AppleClang", "14.0.0", False, "Darwin"),
        ("MSVC", "1930", True, "Windows"),
        ("Clang", "13.0.0", True, "Windows"),
    ),
)
@pytest.mark.parametrize("tuning", (False, True))
def test_compiler_options_preserve_caller_flags_and_parsing_guards(
    tmp_path, compiler, version, msvc, system, tuning
):
    dart_cmake = (ROOT / "dart/CMakeLists.txt").read_text(encoding="utf-8")
    start = dart_cmake.index("foreach(option IN LISTS DART_MSVC_CXX_PARSING_OPTIONS)")
    public_options = dart_cmake[
        start : dart_cmake.index("endforeach()", start) + len("endforeach()")
    ]
    variables = (
        "CMAKE_CXX_FLAGS",
        "CMAKE_CXX_FLAGS_RELEASE",
        "CMAKE_CXX_FLAGS_DEBUG",
        "CMAKE_CXX_FLAGS_RELWITHDEBINFO",
        "CMAKE_CXX_FLAGS_PROFILE",
        "CMAKE_SHARED_LINKER_FLAGS",
    )
    body = (
        f"set(MSVC {'TRUE' if msvc else 'FALSE'})\n"
        f"set(CMAKE_COMPILER_IS_GNUCXX {'TRUE' if compiler == 'GNU' else 'FALSE'})\n"
        f"set(CMAKE_SYSTEM_NAME {system})\n"
        + "".join(
            f"set({option} {'ON' if tuning else 'OFF'})\n"
            for option in (
                "DART_TREAT_WARNINGS_AS_ERRORS",
                "DART_FAST_DEBUG",
                "DART_FORCE_COLORED_OUTPUT",
                "CMAKE_UNITY_BUILD",
            )
        )
        + "".join(f'set({variable} "caller")\n' for variable in variables)
        + "dart_configure_compiler_options()\n"
        "add_library(dart INTERFACE)\n"
        + public_options
        + "\nget_target_property(public_options dart INTERFACE_COMPILE_OPTIONS)\n"
        "get_directory_property(options COMPILE_OPTIONS)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/flags.txt" "options=${options}\\n" '
        '"public_options=${public_options}\\n")\n'
        + "".join(
            f'file(APPEND "${{CMAKE_BINARY_DIR}}/flags.txt" "{variable}=${{{variable}}}\\n")\n'
            for variable in variables
        )
    )
    result = configure(tmp_path, compiler, version, msvc=msvc, extra_cmake=body)
    assert result.returncode == 0, result.stdout + result.stderr
    values = dict(
        line.split("=", 1)
        for line in (tmp_path / "build/flags.txt")
        .read_text(encoding="utf-8")
        .splitlines()
    )
    expected = dict.fromkeys(variables, "caller")
    if not msvc:
        debug = "-g -fno-omit-frame-pointer -fno-inline-functions"
        if compiler == "GNU":
            debug += " -fno-inline-functions-called-once"
            expected["CMAKE_SHARED_LINKER_FLAGS"] = "-Wl,--no-undefined"
        debug += " -fno-optimize-sibling-calls" + (" -O1" if tuning else "")
        expected.update(
            CMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG",
            CMAKE_CXX_FLAGS_DEBUG=debug,
            CMAKE_CXX_FLAGS_RELWITHDEBINFO=f"-O3 -DNDEBUG {debug}",
            CMAKE_CXX_FLAGS_PROFILE=f"{debug} -pg",
        )
        if system == "Darwin":
            expected["CMAKE_CXX_FLAGS"] = "caller -stdlib=libc++"
    assert {variable: values[variable] for variable in variables} == expected
    options = values["options"].split(";")
    warning_option = "$<$<COMPILE_LANGUAGE:CXX>:/WX>" if msvc else "-Werror"
    assert (warning_option in options) == tuning
    parsing_options = ("/permissive-", "/Zc:twoPhase-")
    assert values["public_options"].split(";") == [
        f"$<$<COMPILE_LANG_AND_ID:CXX,MSVC>:{option}>" for option in parsing_options
    ]
    for option in parsing_options:
        assert options.count(f"$<$<COMPILE_LANGUAGE:CXX>:{option}>") == int(msvc)
        assert option not in options
    if not msvc:
        assert {"-Wall", "-Wextra", "-fPIC"}.issubset(options)
        color = (
            "-fdiagnostics-color=always" if compiler == "GNU" else "-fcolor-diagnostics"
        )
        assert (color in options) == tuning
        assert ("-Wno-array-bounds" in options) == (
            compiler == "GNU" and version == "13.2.0"
        )
        assert ("$<$<COMPILE_LANGUAGE:CXX>:-Wno-subobject-linkage>" in options) == (
            compiler == "GNU" and tuning
        )
    else:
        assert ("$<$<COMPILE_LANGUAGE:CXX>:/MP>" in options) == (compiler == "MSVC")
        assert not {"-Wall", "-Wextra", "-fPIC", "-Werror"}.intersection(options)


def test_compiler_options_reject_unsupported_compiler(tmp_path):
    result = configure(
        tmp_path, "Unknown", "1.0", extra_cmake="dart_configure_compiler_options()\n"
    )
    assert result.returncode != 0
    assert "not supported" in result.stderr
