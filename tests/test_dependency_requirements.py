"""Exercise DART's installed dependency finders with isolated CMake packages."""

import hashlib
import os
import re
import shutil
import subprocess
import sys
import tarfile
import time
from pathlib import Path, PureWindowsPath

import pytest

ROOT = Path(__file__).resolve().parents[1]


# DART_TEST_CMAKE pins an exact CMake build (pixi's own bin dir wins on PATH).
CMAKE = os.environ.get("DART_TEST_CMAKE") or shutil.which("cmake")


def configure(tmp_path, content, *, cmake_args=(), language="NONE"):
    source = tmp_path / "source"
    source.mkdir(exist_ok=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        f"project(DependencyRequirements {language})\n"
        f'set(CMAKE_MODULE_PATH "{(tmp_path / "modules").as_posix()}" "{(ROOT / "cmake").as_posix()}")\n'
        f'set(CMAKE_PREFIX_PATH "{(tmp_path / "prefix").as_posix()}")\n'
        "set(CMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH FALSE)\n"
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)\n"
        "set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH FALSE)\n"
        "set(CMAKE_FIND_USE_PACKAGE_REGISTRY FALSE)\n"
        "set(CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY FALSE)\n"
        "set(CMAKE_DISABLE_FIND_PACKAGE_PkgConfig TRUE)\n" + content
    )
    return subprocess.run(
        [
            CMAKE,
            "-G",
            "Ninja",
            f"-DCMAKE_MAKE_PROGRAM={shutil.which('ninja')}",
            "-S",
            str(source),
            "-B",
            str(tmp_path / "build"),
            *cmake_args,
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def config_package(tmp_path, package, version, target=None, extra=""):
    directory = tmp_path / "prefix" / "lib" / "cmake" / package
    directory.mkdir(parents=True)
    upper = package.upper()
    contents = f"set({package}_FOUND TRUE)\nset({upper}_FOUND TRUE)\n"
    if version is not None:
        contents += f'set({package}_VERSION "{version}")\n'
        (directory / f"{package}ConfigVersion.cmake").write_text(
            f'set(PACKAGE_VERSION "{version}")\n'
            "if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)\n"
            "  set(PACKAGE_VERSION_COMPATIBLE FALSE)\n"
            "elseif(PACKAGE_FIND_VERSION_MAJOR STREQUAL "
            f'"{version.split(".")[0]}")\n'
            "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
            "endif()\n"
        )
    if target:
        contents += f"add_library({target} INTERFACE IMPORTED)\n"
    (directory / f"{package}Config.cmake").write_text(contents + extra)


@pytest.mark.parametrize(
    "source_kind",
    [
        "missing",
        "override",
        "default",
        "custom",
        "invalid_override",
        "empty_default",
        "empty_override",
        "file_override",
        "connected",
    ],
)
def test_fetchcontent_disconnected_sources(tmp_path, source_kind):
    source = tmp_path / (
        "build/_deps/fixture-src"
        if source_kind in ("default", "invalid_override", "empty_default")
        else "seed"
    )
    if source_kind == "file_override":
        source.write_text("not a directory")
    else:
        source.mkdir(parents=True)
        if not source_kind.startswith("empty_"):
            (source / "CMakeLists.txt").write_text(
                'message(STATUS "Pre-seeded fixture reached")\n'
            )
    settings = "set(FETCHCONTENT_FULLY_DISCONNECTED ON)\n"
    custom = ""
    if source_kind in ("override", "empty_override", "file_override"):
        settings += f'set(FETCHCONTENT_SOURCE_DIR_FIXTURE "{source.as_posix()}")\n'
    elif source_kind in ("default", "invalid_override"):
        if source_kind == "invalid_override":
            settings += f'set(FETCHCONTENT_SOURCE_DIR_FIXTURE "{(tmp_path / "missing").as_posix()}")\n'
    elif source_kind == "custom":
        custom = f' "{source.as_posix()}"'
    elif source_kind == "connected":
        settings = "set(FETCHCONTENT_FULLY_DISCONNECTED OFF)\n"
    declare = (
        f'SOURCE_DIR "{source.as_posix()}"'
        if source_kind in ("override", "default", "custom", "connected")
        else "URL https://invalid.example/fixture.tar.gz"
    )
    result = configure(
        tmp_path,
        "include(DARTMacros)\ninclude(FetchContent)\n"
        + settings
        + f"FetchContent_Declare(fixture {declare})\nFetchContent_MakeAvailable(fixture)\n"
        + f"dart_check_fetchcontent(FiXtUrE DART_USE_SYSTEM_FIXTURE{custom})\n",
    )
    output = result.stdout + result.stderr
    if source_kind in (
        "missing",
        "invalid_override",
        "empty_default",
        "empty_override",
        "file_override",
    ):
        assert result.returncode != 0
        assert "FETCHCONTENT_SOURCE_DIR_FIXTURE" in output
        # CMake itself rejects a missing override before the postcheck runs.
        if source_kind != "invalid_override":
            assert "Cannot fetch FiXtUrE" in output
            assert "DART_USE_SYSTEM_FIXTURE=ON" in output
    else:
        assert result.returncode == 0, output
        if source_kind != "connected":
            assert "Pre-seeded fixture reached" in output


@pytest.mark.parametrize("umbrella", [None, "OFF", "ON"])
@pytest.mark.parametrize("explicit", [None, "OFF", "ON"])
def test_system_dependency_defaults_are_seeded_once(tmp_path, umbrella, explicit):
    source = (ROOT / "CMakeLists.txt").read_text()
    options = source[
        source.index("dart_option(\n  DART_USE_SYSTEM_DEPENDENCIES") : source.index(
            "dart_option(\n  DART_ALLOW_SINGLE_PRECISION_LIBCCD"
        )
    ]
    content = "include(dart_defs)\n" + options
    switches = ("GOOGLETEST", "GOOGLEBENCHMARK", "TRACY")
    args = [] if umbrella is None else [f"-DDART_USE_SYSTEM_DEPENDENCIES={umbrella}"]
    if explicit is not None:
        args.extend(f"-DDART_USE_SYSTEM_{name}={explicit}" for name in switches)
    expected = explicit or umbrella or "OFF"
    for configure_args in (
        args,
        [f"-DDART_USE_SYSTEM_DEPENDENCIES={'OFF' if umbrella == 'ON' else 'ON'}"],
    ):
        result = configure(tmp_path, content, cmake_args=configure_args)
        assert result.returncode == 0, result.stdout + result.stderr
        cache = (tmp_path / "build" / "CMakeCache.txt").read_text()
        for name in switches:
            assert f"DART_USE_SYSTEM_{name}:BOOL={expected}" in cache


_DEPENDENCIES = (ROOT / "cmake/DARTFindDependencies.cmake").read_text()
TRACY_FIXTURE = (
    "set(DART_BUILD_PROFILE ON)\nset(DART_PROFILE_TRACY ON)\n"
    "set(DART_USE_SYSTEM_TRACY ON)\n"
    + _DEPENDENCIES[
        _DEPENDENCIES.index(
            "if(DART_BUILD_PROFILE AND DART_PROFILE_TRACY)"
        ) : _DEPENDENCIES.index(
            "# Deprecated compatibility option; DART owns its logging implementation."
        )
    ]
    + 'if(NOT TARGET Tracy::TracyClient)\nmessage(FATAL_ERROR "no TracyClient")\nendif()\n'
)


def cmake_version():
    out = subprocess.run([CMAKE, "--version"], capture_output=True, text=True).stdout
    return tuple(int(part) for part in out.split()[2].split("-")[0].split("."))


PROVIDER = """\
macro(fixture_provider method name)
  file(APPEND "{log}" "${{name}}\\n")
  if(NOT "${{name}}" STREQUAL "{fulfills}")
    return()
  endif()
  if(NOT TARGET fixture::fixture)
    add_library(fixture::fixture INTERFACE IMPORTED GLOBAL)
  endif()
  FetchContent_SetPopulated(${{name}})
endmacro()
cmake_language(
  SET_DEPENDENCY_PROVIDER fixture_provider
  SUPPORTED_METHODS FETCHCONTENT_MAKEAVAILABLE_SERIAL
)
"""


@pytest.mark.skipif(cmake_version() < (3, 24), reason="dependency providers need 3.24")
@pytest.mark.parametrize(
    "state",
    [
        "fulfilled",
        "declined",
        "none",
        "unrelated",
        "fulfilled_other_missing",
        "fulfilled_invalid_override",
    ],
)
def test_fetchcontent_dependency_provider_fulfillment(tmp_path, state):
    log = tmp_path / "provider.log"
    fulfills = {"declined": "", "unrelated": "other"}.get(state, "fixture")
    args = []
    if state != "none":
        (tmp_path / "provider.cmake").write_text(
            PROVIDER.format(log=log.as_posix(), fulfills=fulfills)
        )
        args.append(
            f"-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES={(tmp_path / 'provider.cmake').as_posix()}"
        )
    content = "include(DARTMacros)\ninclude(FetchContent)\n"
    content += "set(FETCHCONTENT_FULLY_DISCONNECTED ON)\n"
    if state == "fulfilled_invalid_override":
        content += f'set(FETCHCONTENT_SOURCE_DIR_FIXTURE "{(tmp_path / "missing").as_posix()}")\n'
    for name in (
        ("fixture", "second") if state == "fulfilled_other_missing" else ("fixture",)
    ):
        content += (
            f"FetchContent_Declare({name} URL https://invalid.example/{name}.tar.gz)\n"
            f"FetchContent_MakeAvailable({name})\n"
            f"dart_check_fetchcontent({name} DART_USE_SYSTEM_FIXTURE)\n"
        )
    content += 'if(NOT TARGET fixture::fixture)\nmessage(FATAL_ERROR "target missing")\nendif()\n'
    result = configure(tmp_path, content, cmake_args=args)
    output = result.stdout + result.stderr
    calls = log.read_text().split() if log.exists() else []
    assert not (tmp_path / "build/_deps/fixture-src").exists()
    if state in ("fulfilled", "fulfilled_other_missing", "fulfilled_invalid_override"):
        assert calls.count("fixture") == (0 if state.endswith("override") else 1), calls
    if state == "fulfilled":
        assert result.returncode == 0, output
        assert calls == ["fixture"]
    else:
        assert result.returncode != 0
        assert "Cannot fetch" in output or "FETCHCONTENT_SOURCE_DIR" in output, output
        if state == "fulfilled_other_missing":
            assert "Cannot fetch second" in output


@pytest.mark.parametrize(
    "version,accepted",
    [
        ("0.10.0", False),
        ("0.11.0", False),
        ("0.11.1", True),
        ("0.12.0", True),
        (None, True),
    ],
)
def test_system_tracy_minimum_version(tmp_path, version, accepted):
    # Installed Tracy packages ship TracyConfig.cmake without a version file.
    config_package(tmp_path, "Tracy", version, "Tracy::TracyClient")
    result = configure(tmp_path, TRACY_FIXTURE)
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == accepted, output
    if not accepted:
        assert "Tracy >= 0.11.1" in output


TRACY_ENUM = "namespace tracy {{ namespace Version {{ enum {{ {parts} }}; }} }}\n"


@pytest.mark.parametrize(
    "header,accepted",
    [
        ("Major = 0, Minor = 10, Patch = 0", False),
        ("Major = 0, Minor = 11, Patch = 0", False),
        ("Major = 0, Minor = 11, Patch = 1", True),
        ("Major = 0,\n Minor = 12,\n Patch = 0", True),
        ("Major = 0, Minor = 11", False),
        (None, True),
    ],
)
def test_system_tracy_target_header_version(tmp_path, header, accepted):
    include = tmp_path / "prefix/include/tracy"
    (include / "common").mkdir(parents=True)
    if header is not None:
        (include / "common/TracyVersion.hpp").write_text(
            TRACY_ENUM.format(parts=header)
        )
    config_package(
        tmp_path,
        "Tracy",
        None,
        "Tracy::TracyClient",
        extra=(
            "set_target_properties(Tracy::TracyClient PROPERTIES "
            f'INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:{include.as_posix()}>")\n'
        ),
    )
    result = configure(tmp_path, TRACY_FIXTURE)
    output = result.stdout + result.stderr
    # A header with a missing component is malformed, not versionless.
    assert (result.returncode == 0) == accepted, output
    if not accepted:
        assert "Tracy" in output and "0.11.1" in output


def test_system_tracy_ignores_host_header_decoy(tmp_path):
    decoy = tmp_path / "host/common"
    decoy.mkdir(parents=True)
    (decoy / "TracyVersion.hpp").write_text(
        TRACY_ENUM.format(parts="Major = 0, Minor = 1, Patch = 0")
    )
    config_package(tmp_path, "Tracy", None, "Tracy::TracyClient")
    result = configure(
        tmp_path,
        f'set(CMAKE_INCLUDE_PATH "{(tmp_path / "host").as_posix()}")\n' + TRACY_FIXTURE,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("build_version", ["3.4.0", "5.0.1"])
@pytest.mark.parametrize("consumer_version", ["3.4.1", "3.4.90", "5.0.2", "6.0.0"])
def test_component_exports_only_install_owned_dependency_finders(
    tmp_path, build_version, consumer_version
):
    config_package(tmp_path, "Eigen3", build_version)
    modules = tmp_path / "modules"
    modules.mkdir()
    # Gazebo's FindEIGEN3 matches FindEigen3 on case-insensitive filesystems.
    (modules / "FindEigen3.cmake").write_text("gz_import_target(EIGEN3 INTERFACE)\n")
    (modules / "FindODE.cmake").write_text(
        'message(FATAL_ERROR "Foreign ODE finder")\n'
    )
    result = configure(
        tmp_path,
        f'set(DART_SOURCE_DIR "{ROOT.as_posix()}")\n'
        "set(CONFIG_INSTALL_DIR lib/cmake/DART)\n"
        "set(LIBRARY_INSTALL_DIR lib)\n"
        "include(Components)\n"
        "include(DARTFindEigen3)\n"
        "initialize_component_helpers(dart)\n"
        "add_component(dart fixture)\n"
        "add_library(fixture INTERFACE)\n"
        "add_component_targets(dart fixture fixture)\n"
        "add_component_dependency_packages(dart fixture Eigen3 ODE urdfdom)\n"
        "install_component_exports(dart)\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    stage = tmp_path / "stage"
    result = subprocess.run(
        [
            shutil.which("cmake"),
            "--install",
            str(tmp_path / "build"),
            "--prefix",
            str(stage),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    installed = stage / "lib/cmake/DART"
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    config_package(consumer, "Eigen3", consumer_version)
    result = configure(
        consumer,
        f'set(CMAKE_MODULE_PATH "{installed.as_posix()}")\n'
        "find_package(Eigen3 REQUIRED)\n"
        "include(DARTFindEigen3)\n",
    )
    output = result.stdout + result.stderr
    build_major = build_version.split(".")[0]
    same_major = build_major == consumer_version.split(".")[0]
    assert (result.returncode == 0) == same_major, output
    if consumer_version.startswith("6."):
        assert "Eigen3 >= 3.4.0, < 6 is required" in output
    elif not same_major:
        assert f"DART was built with Eigen major {build_major}" in output
        assert f"found Eigen {consumer_version}" in output
        assert "rebuild DART" in output
        assert "clear the cached Eigen3_DIR" in output
    assert not (installed / "FindEigen3.cmake").exists()
    assert (installed / "DARTFindEigen3.cmake").read_text() == (
        f"set(DART_BUILD_EIGEN_MAJOR {build_major})\n"
        + (ROOT / "cmake/DARTFindEigen3.cmake").read_text()
    )
    for name in ("DARTFindODE.cmake", "FindODE.cmake", "DARTFindurdfdom.cmake"):
        assert (installed / name).read_text() == (ROOT / "cmake" / name).read_text()


@pytest.mark.parametrize("build_version", ["", "unknown", "3.4.90"])
def test_component_eigen_finder_generation(tmp_path, build_version):
    modules = tmp_path / "modules"
    modules.mkdir()
    for name in ("Components.cmake", "DARTFindEigen3.cmake"):
        shutil.copy2(ROOT / "cmake" / name, modules / name)
    result = configure(
        tmp_path,
        f'set(DART_SOURCE_DIR "{ROOT.as_posix()}")\n'
        "set(CONFIG_INSTALL_DIR lib/cmake/DART)\n"
        f'set(Eigen3_VERSION "{build_version}")\n'
        "include(Components)\n"
        "initialize_component_helpers(dart)\n"
        "add_component(dart fixture)\n"
        "add_library(fixture INTERFACE)\n"
        "add_component_targets(dart fixture fixture)\n"
        "add_component_dependency_packages(dart fixture Eigen3)\n"
        "install_component_exports(dart)\n",
    )
    output = result.stdout + result.stderr
    if build_version != "3.4.90":
        assert result.returncode != 0, output
        assert "Cannot determine Eigen major" in output
        return
    assert result.returncode == 0, output
    finder = modules / "DARTFindEigen3.cmake"
    # Windows file times advance in clock ticks of about 16 ms, so an edit made
    # right after the configure can share build.ninja's time.
    configured = (tmp_path / "build/build.ninja").stat().st_mtime_ns
    while time.time_ns() < configured + 100_000_000:
        time.sleep(0.01)
    finder.write_text(finder.read_text() + "\n# Updated finder\n")
    result = subprocess.run(
        [shutil.which("cmake"), "--build", str(tmp_path / "build")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    generated = tmp_path / "build/lib/cmake/DART/DARTFindEigen3.cmake"
    assert (
        generated.read_text() == "set(DART_BUILD_EIGEN_MAJOR 3)\n" + finder.read_text()
    )


@pytest.mark.parametrize("skip", ["ON", "OFF"])
def test_optional_package_with_undefined_found_flag(tmp_path, skip):
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        f"set(DART_SKIP_Missing {skip})\n"
        'dart_check_optional_package(Missing "optional component" "Missing")\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cache = (tmp_path / "build" / "CMakeCache.txt").read_text()
    assert "HAVE_Missing:BOOL=FALSE" in cache


@pytest.mark.parametrize("missing", ["tinyxml2", "urdfdom", "sdformat"])
def test_io_requires_all_parser_dependencies(tmp_path, missing):
    if missing != "tinyxml2":
        config_package(tmp_path, "tinyxml2", "10.0.0", "tinyxml2::tinyxml2")
    if missing == "sdformat":
        config_package(tmp_path, "urdfdom", "4.0.0", "urdfdom")
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        f"set(CMAKE_DISABLE_FIND_PACKAGE_{missing} TRUE)\n"
        f'include("{(ROOT / "dart/utils/CMakeLists.txt").as_posix()}")\n',
    )
    assert result.returncode != 0
    assert f"dart-io requires {missing}" in result.stdout + result.stderr
    assert "DART_BUILD_IO=OFF" in result.stdout + result.stderr


@pytest.mark.parametrize("lookup", ["config", "module"])
def test_io_rejects_tinyxml2_9_with_minimum_version_message(tmp_path, lookup):
    variables = ""
    if lookup == "config":
        config_package(tmp_path, "tinyxml2", "9.0.0", "tinyxml2::tinyxml2")
    else:
        include = tmp_path / "include"
        include.mkdir()
        (include / "tinyxml2.h").write_text(
            "#define TINYXML2_MAJOR_VERSION 9\n"
            "#define TINYXML2_MINOR_VERSION 0\n"
            "#define TINYXML2_PATCH_VERSION 0\n"
        )
        library = tmp_path / "library"
        library.touch()
        variables = (
            f'set(TINYXML2_INCLUDE_DIRS "{include.as_posix()}")\n'
            f'set(TINYXML2_LIBRARIES "{library.as_posix()}")\n'
        )
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        + variables
        + f'include("{(ROOT / "dart/utils/CMakeLists.txt").as_posix()}")\n',
    )
    assert result.returncode != 0
    assert "dart-io requires tinyxml2 >= 10.0.0" in result.stdout + result.stderr


@pytest.mark.parametrize(
    "package,minimum,target,required",
    [
        ("Eigen3", "3.4.0", "Eigen3::Eigen", True),
        ("fcl", "0.7.0", "fcl", True),
        ("octomap", "1.9.7", "octomap", False),
        ("tinyxml2", "10.0.0", "tinyxml2::tinyxml2", False),
        ("urdfdom", "4.0.0", "urdfdom", False),
        ("ODE", "0.16.2", "ODE::ODE", False),
        ("imgui", "1.92.8", "imgui::imgui", True),
    ],
)
@pytest.mark.parametrize("version_kind", ["minimum", "older", "unknown", "newer"])
def test_config_floors(tmp_path, package, minimum, target, required, version_kind):
    major, minor, patch = map(int, minimum.split("."))
    versions = {
        "minimum": minimum,
        "older": f"{major}.{minor}.{patch - 1}" if patch else f"{major}.{minor - 1}.9",
        "unknown": None,
        "newer": f"{major}.{minor + 1}.0",
    }
    if package == "tinyxml2" and version_kind == "older":
        versions["older"] = "9.0.0"
    if package == "urdfdom" and version_kind == "older":
        versions["older"] = "3.1.1"
    if package in ("tinyxml2", "urdfdom") and version_kind == "newer":
        versions["newer"] = f"{major + 1}.0.0"
    extra = ""
    if package == "ODE":
        extra = 'set_target_properties(ODE::ODE PROPERTIES INTERFACE_COMPILE_DEFINITIONS "dLIBCCD_BOX_CYL;dDOUBLE")\n'
    config_package(tmp_path, package, versions[version_kind], target, extra)
    accepted = version_kind in ("minimum", "newer")
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_ODE ON)\n"
        f"include(DARTFind{package})\n"
        f"if({package}_FOUND OR {package.upper()}_FOUND)\n"
        + ("" if accepted else 'message(FATAL_ERROR "Accepted unsupported version")\n')
        + "else()\n"
        + ('message(FATAL_ERROR "Rejected supported version")\n' if accepted else "")
        + "endif()\n",
    )
    if required and not accepted:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "Accepted unsupported version" not in result.stderr
        assert minimum in result.stdout + result.stderr
    else:
        assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("version", [None, "3.1.1", "4.0.0", "5.0.0", "6.0.1"])
def test_urdfdom_version_file_without_config_version_variable(tmp_path, version):
    config_package(tmp_path, "urdfdom", version)
    # Upstream configs leave the version to the separate CMake version file.
    config = tmp_path / "prefix/lib/cmake/urdfdom/urdfdomConfig.cmake"
    config.write_text("set(urdfdom_FOUND TRUE)\n")
    accepted = version in {"4.0.0", "5.0.0", "6.0.1"}
    result = configure(
        tmp_path,
        "include(DARTFindurdfdom)\n"
        "if(urdfdom_FOUND OR URDFDOM_FOUND)\n"
        + ("" if accepted else 'message(FATAL_ERROR "Accepted unsupported version")\n')
        + "else()\n"
        + ('message(FATAL_ERROR "Rejected supported version")\n' if accepted else "")
        + "endif()\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_urdfdom_msvc_strips_invalid_include_without_tinyxml(tmp_path):
    include_dir = tmp_path / "prefix/include"
    include_dir.mkdir(parents=True)
    config_package(
        tmp_path,
        "urdfdom",
        "4.0.0",
        extra=f'set(urdfdom_INCLUDE_DIRS "/include;{include_dir.as_posix()}")\n',
    )
    result = configure(
        tmp_path,
        "set(MSVC TRUE)\n"
        "set(CMAKE_DISABLE_FIND_PACKAGE_TinyXML TRUE)\n"
        "include(DARTFindurdfdom)\n"
        "get_target_property(includes urdfdom INTERFACE_INCLUDE_DIRECTORIES)\n"
        f'if(NOT urdfdom_FOUND OR NOT includes STREQUAL "{include_dir.as_posix()}")\n'
        'message(FATAL_ERROR "Incorrect urdfdom includes")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("version", ["3.4.0", "3.4.1", "5.0.1", "6.0.0"])
def test_eigen_supported_range_with_minor_scoped_version_files(tmp_path, version):
    # Eigen 5's version file accepts a requested single version only within
    # that minor release, so a 3.4.0 request rejects Eigen 5.
    config_package(tmp_path, "Eigen3", version, "Eigen3::Eigen")
    major, minor, _ = version.split(".")
    (tmp_path / "prefix/lib/cmake/Eigen3/Eigen3ConfigVersion.cmake").write_text(
        f'set(PACKAGE_VERSION "{version}")\n'
        "set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
        "if(PACKAGE_FIND_VERSION AND (PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION\n"
        f"    OR NOT PACKAGE_FIND_VERSION_MAJOR EQUAL {major}\n"
        f"    OR NOT PACKAGE_FIND_VERSION_MINOR EQUAL {minor}))\n"
        "  set(PACKAGE_VERSION_COMPATIBLE FALSE)\n"
        "endif()\n"
    )
    result = configure(
        tmp_path,
        "include(DARTFindEigen3)\n"
        'if(NOT Eigen3_FOUND)\n  message(FATAL_ERROR "Rejected Eigen3")\nendif()\n',
    )
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == (major != "6"), output
    if major == "6":
        assert "Eigen3 >= 3.4.0, < 6 is required" in output


@pytest.mark.parametrize("version", [None, "3.2.10", "3.3.0", "3.4.0", "5.0.1"])
def test_nanobind_eigen_probe_version_floor(tmp_path, version):
    config_package(tmp_path, "Eigen3", version, "Eigen3::Eigen")
    source = (ROOT / "scripts/nanobind/eigen_probe/CMakeLists.txt").read_text()
    discovery = source[
        source.index("find_package(Eigen3") : source.index("get_filename_component(")
    ]
    result = configure(tmp_path, discovery)
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == (version not in {None, "3.2.10"}), output
    if result.returncode:
        assert "Eigen3 >= 3.3 is required" in output


@pytest.mark.parametrize(
    "version,accepted", [("1.92.8b", True), ("1.92.7b", False), ("1.91.9b", False)]
)
def test_imgui_release_suffix_floor(tmp_path, version, accepted):
    config_package(tmp_path, "imgui", version, "imgui::imgui")
    result = configure(tmp_path, "include(DARTFindimgui)\n")
    assert (result.returncode == 0) == accepted, result.stdout + result.stderr
    if not accepted:
        assert "ImGui >= 1.92.8" in result.stderr


@pytest.mark.parametrize("setting", ["legacy-default", "explicit-on", "explicit-off"])
def test_legacy_gui_option_migration(tmp_path, setting):
    source = (ROOT / "CMakeLists.txt").read_text()
    migration = source[
        source.index("# DART 7 removed the OpenSceneGraph GUI.") : source.index(
            'dart_option(DART_BUILD_IO "'
        )
    ]
    if setting == "legacy-default":
        build = tmp_path / "build"
        build.mkdir()
        (build / "CMakeCache.txt").write_text(
            "//Build osgDart library\nDART_BUILD_GUI_OSG:BOOL=ON\n"
        )
        args = ()
    else:
        args = (f"-DDART_BUILD_GUI_OSG={'ON' if setting == 'explicit-on' else 'OFF'}",)
    result = configure(
        tmp_path,
        "include(dart_defs)\n"
        + migration
        + 'dart_option(DART_BUILD_GUI "Build the Filament GUI and ImGui support" ON)\n',
        cmake_args=args,
    )
    output = result.stdout + result.stderr
    if setting == "explicit-on":
        assert result.returncode != 0
        assert "DART 7 removed the OpenSceneGraph GUI" in output
    else:
        assert result.returncode == 0, output
        cache = (tmp_path / "build" / "CMakeCache.txt").read_text()
        assert (
            f"DART_BUILD_GUI:BOOL={'OFF' if setting == 'explicit-off' else 'ON'}"
            in cache
        )
        assert "DART_BUILD_GUI_OSG:" not in cache
        if setting == "legacy-default":
            assert "Removing the cached legacy OSG GUI default" in output


@pytest.mark.parametrize("gui", ["absent", "legacy", "filament", "other-prefix"])
def test_examples_detect_selected_filament_gui(tmp_path, gui):
    include = tmp_path / "selected-include"
    include.mkdir()
    if gui in ("filament", "other-prefix"):
        header = (
            (include if gui == "filament" else tmp_path / "prefix" / "include")
            / "dart"
            / "gui"
            / "Viewer.hpp"
        )
        header.parent.mkdir(parents=True)
        header.touch()
    target = ""
    if gui != "absent":
        target = (
            "add_library(dart-gui INTERFACE IMPORTED)\n"
            f'set_target_properties(dart-gui PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "{include.as_posix()}")\n'
        )
    result = configure(
        tmp_path,
        target
        + f'include("{(ROOT / "examples" / "CheckFilamentGui.cmake").as_posix()}")\n'
        + f"if(NOT DART_EXAMPLES_HAVE_FILAMENT_GUI STREQUAL {'TRUE' if gui == 'filament' else 'FALSE'})\n"
        + 'message(FATAL_ERROR "Incorrect GUI capability")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("bundled", [False, True])
@pytest.mark.parametrize("version", [304, 305, 325, None])
def test_bullet_header_floor(tmp_path, bundled, version):
    include = tmp_path / "bullet" / "src"
    header = include / "LinearMath" / "btScalar.h"
    header.parent.mkdir(parents=True)
    header.write_text(f"#define BT_BULLET_VERSION {version}\n" if version else "")
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "FindBullet.cmake").write_text(
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        f'set(BULLET_INCLUDE_DIR "{include.as_posix()}")\n'
        "set(BULLET_COLLISION_LIBRARY BulletCollision)\n"
        "set(BULLET_MATH_LIBRARY LinearMath)\n"
    )
    content = f"set(DART_USE_SYSTEM_BULLET {'OFF' if bundled else 'ON'})\n"
    if bundled:
        content += (
            "add_library(BulletCollision INTERFACE)\nadd_library(LinearMath INTERFACE)\n"
            f'set(DART_BULLET_SOURCE_DIR "{include.parent.as_posix()}")\n'
        )
    content += "include(DARTFindBullet)\n"
    # Bullet 3.06 and 3.07 report 305, so 305 is the checkable floor.
    if version and version >= 305:
        content += 'if(NOT BULLET_FOUND OR NOT TARGET Bullet)\nmessage(FATAL_ERROR "Missing Bullet")\nendif()\n'
    else:
        content += 'if(BULLET_FOUND OR Bullet_FOUND OR TARGET Bullet)\nmessage(FATAL_ERROR "Accepted old Bullet")\nendif()\n'
    result = configure(tmp_path, content)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("debug", [False, True])
@pytest.mark.parametrize(
    "installation",
    ["collision-only", "complete", "no-math", "no-collision", "no-headers"],
)
def test_bullet_collision_libraries(tmp_path, installation, debug):
    prefix = tmp_path / "prefix"
    include = prefix / "include/bullet"
    if installation != "no-headers":
        (include / "LinearMath").mkdir(parents=True)
        (include / "btBulletCollisionCommon.h").touch()
        (include / "LinearMath/btScalar.h").write_text(
            "#define BT_BULLET_VERSION 305\n"
        )
    lib = prefix / "lib"
    lib.mkdir(parents=True)
    expected = []
    for name, missing in (
        ("BulletCollision", "no-collision"),
        ("LinearMath", "no-math"),
        ("BulletDynamics", "collision-only"),
        ("BulletSoftBody", "collision-only"),
    ):
        if installation == missing:
            continue
        library = lib / f"lib{name}.a"
        library.touch()
        if debug:
            debug_library = lib / f"lib{name}_Debug.a"
            debug_library.touch()
        if name in ("BulletCollision", "LinearMath"):
            expected.extend(
                ["optimized", library.as_posix(), "debug", debug_library.as_posix()]
                if debug
                else [library.as_posix()]
            )
    accepted = installation in ("collision-only", "complete")
    result = configure(
        tmp_path,
        'set(BULLET_INCLUDE_DIRS "stale-parent-include")\n'
        "set(DART_USE_SYSTEM_BULLET ON)\ninclude(DARTFindBullet)\n"
        + (
            "if(NOT BULLET_FOUND OR NOT Bullet_FOUND OR NOT TARGET Bullet)\n"
            'message(FATAL_ERROR "Rejected collision libraries")\nendif()\n'
            f'if(NOT BULLET_INCLUDE_DIRS STREQUAL "{include.as_posix()}"\n'
            f'   OR NOT BULLET_LIBRARIES STREQUAL "{";".join(expected)}")\n'
            'message(FATAL_ERROR "Incorrect Bullet usage requirements")\nendif()\n'
            if accepted
            else "if(BULLET_FOUND OR Bullet_FOUND OR TARGET Bullet)\n"
            'message(FATAL_ERROR "Accepted incomplete Bullet")\nendif()\n'
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr


def bullet_fallback_source(tmp_path):
    source = tmp_path / "bullet-source"
    src = source / "src"
    src.mkdir(parents=True)
    for name in ("BulletCollision", "LinearMath", "BulletDynamics", "BulletSoftBody"):
        (src / name).mkdir()
        (src / name / "fixture.h").write_text(f"int fixture_{name}(void);\n")
        (src / f"{name}.c").write_text(f"int fixture_{name}(void) {{ return 1; }}\n")
    (src / "LinearMath/btScalar.h").write_text(
        "#define BT_BULLET_VERSION 325\nint fixture_LinearMath(void);\n"
    )
    (src / "btBulletCollisionCommon.h").write_text(
        "#include <LinearMath/btScalar.h>\nint fixture_BulletCollision(void);\n"
    )
    (src / "btBulletDynamicsCommon.h").write_text("/* Not part of the SDK. */\n")
    (src / "CMakeLists.txt").write_text(
        "foreach(name BulletCollision LinearMath BulletDynamics BulletSoftBody)\n"
        'add_library(${name} SHARED "${name}.c")\n'
        "set_target_properties(${name} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)\n"
        'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/${name}-link.txt" CONTENT "$<TARGET_LINKER_FILE:${name}>")\n'
        "endforeach()\n"
        "install(TARGETS BulletCollision LinearMath BulletDynamics BulletSoftBody\n"
        "RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)\n"
        "install(DIRECTORY BulletCollision LinearMath BulletDynamics BulletSoftBody\n"
        'DESTINATION include/bullet FILES_MATCHING PATTERN "*.h")\n'
        "install(FILES btBulletDynamicsCommon.h DESTINATION include/bullet)\n"
    )
    (source / "bullet.pc.cmake").write_text(
        "prefix=@CMAKE_INSTALL_PREFIX@\nexec_prefix=${prefix}\n"
        "libdir=${exec_prefix}/@LIB_DESTINATION@\n"
        "includedir=${prefix}/@INCLUDE_INSTALL_DIR@\n"
        "Name: bullet\nDescription: Fixture collision SDK\nVersion: 3.25\n"
        "Libs: -L${libdir} -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath\n"
        "Cflags: -I${includedir} -I${prefix}/include\n"
    )
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\nproject(BulletFixture C)\n"
        'set(LIB_DESTINATION "lib" CACHE STRING "")\n'
        "configure_file(bullet.pc.cmake bullet.pc @ONLY)\n"
        'install(FILES "${CMAKE_CURRENT_BINARY_DIR}/bullet.pc" DESTINATION lib/pkgconfig)\n'
        "add_subdirectory(src)\n"
    )
    return source


def bullet_fallback_content():
    begin = _DEPENDENCIES.index(
        "if(NOT DART_USE_SYSTEM_BULLET AND NOT DART_SKIP_BULLET)"
    )
    end = _DEPENDENCIES.index("# Misc. dependencies", begin)
    return (
        "include(DARTMacros)\ninclude(FetchContent)\ninclude(GNUInstallDirs)\n"
        "set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})\n"
        "set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})\n"
        'set(DART_USE_SYSTEM_BULLET OFF)\nset(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)\n'
        + _DEPENDENCIES[begin:end]
        + 'if(BUILD_SHARED_LIBS)\nmessage(FATAL_ERROR "Caller shared-library state changed")\nendif()\n'
    )


def bullet_fixture_run(*args, env=None):
    result = subprocess.run(
        list(map(str, args)), capture_output=True, text=True, env=env, check=False
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout.strip()


@pytest.mark.parametrize("libdir", ["lib", "lib64", "absolute"])
@pytest.mark.parametrize("source_suffix", ["", "/", "/.", "/sub/.."])
def test_bullet_fetched_pkgconfig_libdir(tmp_path, monkeypatch, libdir, source_suffix):
    source = bullet_fallback_source(tmp_path)
    (source / "sub").mkdir()
    stage = tmp_path / "installed"
    directory = tmp_path / "separate-libs" if libdir == "absolute" else Path(libdir)
    result = configure(
        tmp_path,
        f'set(FETCHCONTENT_SOURCE_DIR_BULLET "{source.as_posix()}{source_suffix}")\n'
        + bullet_fallback_content(),
        language="C",
        cmake_args=(
            f"-DCMAKE_INSTALL_PREFIX={stage.as_posix()}",
            f"-DCMAKE_INSTALL_LIBDIR={directory.as_posix()}",
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    build = tmp_path / "build"
    bullet_fixture_run(
        CMAKE, "--build", build, "--target", "BulletCollision", "LinearMath"
    )
    bullet_fixture_run(CMAKE, "--install", build)
    pc = stage / "lib/pkgconfig/bullet.pc"
    pkgconfig = shutil.which("pkg-config")
    assert pkgconfig
    monkeypatch.setenv("PKG_CONFIG_PATH", str(pc.parent))
    monkeypatch.setenv("PKG_CONFIG_LIBDIR", str(pc.parent))
    monkeypatch.delenv("PKG_CONFIG_SYSROOT_DIR", raising=False)
    actual = bullet_fixture_run(pkgconfig, "--variable=libdir", "bullet")
    expected = directory if directory.is_absolute() else stage / directory
    assert Path(actual) == expected
    flags = bullet_fixture_run(pkgconfig, "--libs", "bullet")
    assert "-lBulletCollision" in flags and "-lLinearMath" in flags
    assert "BulletDynamics" not in flags and "BulletSoftBody" not in flags
    assert (stage / "include/btBulletCollisionCommon.h").is_file()
    assert (stage / "include/BulletCollision/btBulletCollisionCommon.h").is_file()
    assert not (stage / "include/bullet/BulletDynamics").exists()
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    result = configure(
        consumer,
        "set(CMAKE_DISABLE_FIND_PACKAGE_PkgConfig FALSE)\n"
        f'set(PKG_CONFIG_EXECUTABLE "{Path(pkgconfig).as_posix()}")\n'
        "find_package(PkgConfig REQUIRED)\npkg_check_modules(BULLET REQUIRED IMPORTED_TARGET bullet)\n"
        'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.c" "#include <btBulletCollisionCommon.h>\\nint main(void) { return fixture_BulletCollision() + fixture_LinearMath() == 2 ? 0 : 1; }\\n")\n'
        'add_executable(consumer "${CMAKE_CURRENT_BINARY_DIR}/main.c")\n'
        "target_link_libraries(consumer PRIVATE PkgConfig::BULLET)\n",
        language="C",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    bullet_fixture_run(CMAKE, "--build", consumer / "build")
    env = {**os.environ, "PATH": str(stage / "bin") + os.pathsep + os.environ["PATH"]}
    bullet_fixture_run(
        consumer / "build" / ("consumer.exe" if os.name == "nt" else "consumer"),
        env=env,
    )
    uninstall = tmp_path / "uninstall.cmake"
    uninstall.write_text(
        (ROOT / "cmake/uninstall_target.cmake.in")
        .read_text()
        .replace("@CMAKE_CURRENT_BINARY_DIR@", build.as_posix())
    )
    bullet_fixture_run(CMAKE, "-P", uninstall)
    assert not pc.exists()
    assert not (stage / "include/btBulletCollisionCommon.h").exists()


@pytest.mark.parametrize("includedir", ["include", "include/sdk", "absolute"])
def test_bullet_fetched_pkgconfig_includedir_first_configure(
    tmp_path, monkeypatch, includedir
):
    source = bullet_fallback_source(tmp_path)
    cmakelists = source / "CMakeLists.txt"
    # Bullet's older cache policy drops the inherited normal value on first use.
    cmakelists.write_text(
        cmakelists.read_text().replace(
            "configure_file(bullet.pc.cmake bullet.pc @ONLY)",
            'cmake_policy(SET CMP0126 OLD)\nset(INCLUDE_INSTALL_DIR "include/bullet" CACHE PATH "")\n'
            "configure_file(bullet.pc.cmake bullet.pc @ONLY)",
        )
    )
    stage = tmp_path / "installed"
    directory = (
        tmp_path / "separate-headers" if includedir == "absolute" else Path(includedir)
    )
    result = configure(
        tmp_path,
        f'set(FETCHCONTENT_SOURCE_DIR_BULLET "{source.as_posix()}")\n'
        + bullet_fallback_content(),
        language="C",
        cmake_args=(
            f"-DCMAKE_INSTALL_PREFIX={stage.as_posix()}",
            f"-DCMAKE_INSTALL_INCLUDEDIR={directory.as_posix()}",
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    build = tmp_path / "build"
    bullet_fixture_run(
        CMAKE, "--build", build, "--target", "BulletCollision", "LinearMath"
    )
    bullet_fixture_run(CMAKE, "--install", build)
    pc = stage / "lib/pkgconfig/bullet.pc"
    pkgconfig = shutil.which("pkg-config")
    assert pkgconfig
    monkeypatch.setenv("PKG_CONFIG_PATH", str(pc.parent))
    monkeypatch.setenv("PKG_CONFIG_LIBDIR", str(pc.parent))
    monkeypatch.delenv("PKG_CONFIG_SYSROOT_DIR", raising=False)
    actual = bullet_fixture_run(pkgconfig, "--variable=includedir", "bullet")
    expected = directory if directory.is_absolute() else stage / directory
    assert Path(actual) == expected
    assert (expected / "btBulletCollisionCommon.h").is_file()
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    result = configure(
        consumer,
        "set(CMAKE_DISABLE_FIND_PACKAGE_PkgConfig FALSE)\n"
        f'set(PKG_CONFIG_EXECUTABLE "{Path(pkgconfig).as_posix()}")\n'
        "find_package(PkgConfig REQUIRED)\npkg_check_modules(BULLET REQUIRED IMPORTED_TARGET bullet)\n"
        'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.c" "#include <btBulletCollisionCommon.h>\\nint main(void) { return fixture_BulletCollision() + fixture_LinearMath() == 2 ? 0 : 1; }\\n")\n'
        'add_executable(consumer "${CMAKE_CURRENT_BINARY_DIR}/main.c")\n'
        "target_link_libraries(consumer PRIVATE PkgConfig::BULLET)\n",
        language="C",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    bullet_fixture_run(CMAKE, "--build", consumer / "build")
    env = {**os.environ, "PATH": str(stage / "bin") + os.pathsep + os.environ["PATH"]}
    bullet_fixture_run(
        consumer / "build" / ("consumer.exe" if os.name == "nt" else "consumer"),
        env=env,
    )


@pytest.mark.parametrize(
    "mode", ["no-source", "metadata-source", "no-collision", "no-math", "bad-headers"]
)
def test_bullet_provider_keeps_ownership(tmp_path, mode):
    source = bullet_fallback_source(tmp_path)
    sdk = tmp_path / "provider-build"
    bullet_fixture_run(CMAKE, "-S", source, "-B", sdk, "-G", "Ninja")
    bullet_fixture_run(
        CMAKE, "--build", sdk, "--target", "BulletCollision", "LinearMath"
    )
    original_pc = (sdk / "bullet.pc").read_bytes()
    provider = tmp_path / "provider.cmake"
    targets = ""
    for name, missing in [
        ("BulletCollision", "no-collision"),
        ("LinearMath", "no-math"),
    ]:
        if mode != missing:
            library = (sdk / f"{name}-link.txt").read_text()
            targets += (
                f"add_library({name} UNKNOWN IMPORTED GLOBAL)\n"
                f'set_target_properties({name} PROPERTIES IMPORTED_LOCATION "{library}")\n'
            )
    metadata = f'SOURCE_DIR "{source.as_posix()}"' if mode == "metadata-source" else ""
    headers = tmp_path / "no-headers" if mode == "bad-headers" else source
    provider.write_text(
        "macro(bullet_provider method name)\n"
        + targets
        + f'set(BULLET_PHYSICS_SOURCE_DIR "{headers.as_posix()}" CACHE PATH "" FORCE)\n'
        + f'FetchContent_SetPopulated(${{name}} {metadata} BINARY_DIR "{sdk.as_posix()}")\n'
        + "endmacro()\n"
        "cmake_language(SET_DEPENDENCY_PROVIDER bullet_provider SUPPORTED_METHODS FETCHCONTENT_MAKEAVAILABLE_SERIAL)\n"
    )
    case = tmp_path / "case"
    case.mkdir()
    accepted = mode not in ("no-collision", "no-math", "bad-headers")
    result = configure(
        case,
        'set(DART_BULLET_SOURCE_DIR "/stale/fetched" CACHE INTERNAL "")\n'
        'set(DART_BULLET_BINARY_DIR "/stale/build" CACHE INTERNAL "")\n'
        "set(FETCHCONTENT_FULLY_DISCONNECTED ON)\n"
        + bullet_fallback_content()
        + "include(DARTFindBullet)\n"
        + f'if({"NOT " if accepted else ""}BULLET_FOUND)\nmessage(FATAL_ERROR "Wrong provider acceptance")\nendif()\n'
        + 'if(DEFINED CACHE{DART_BULLET_SOURCE_DIR} OR DEFINED CACHE{DART_BULLET_BINARY_DIR})\nmessage(FATAL_ERROR "Provider retained fetched-source metadata")\nendif()\n'
        + (
            'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.c" "#include <btBulletCollisionCommon.h>\\nint main(void) { return fixture_BulletCollision() + fixture_LinearMath() == 2 ? 0 : 1; }\\n")\n'
            'add_executable(provider_consumer "${CMAKE_CURRENT_BINARY_DIR}/main.c")\n'
            "target_link_libraries(provider_consumer PRIVATE Bullet)\n"
            if accepted
            else ""
        ),
        language="C",
        cmake_args=(
            f"-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES={provider.as_posix()}",
            f"-DCMAKE_INSTALL_PREFIX={(tmp_path / 'installed').as_posix()}",
        ),
    )
    if mode in ("no-collision", "no-math"):
        assert result.returncode != 0
        return
    assert result.returncode == 0, result.stdout + result.stderr
    if accepted:
        bullet_fixture_run(CMAKE, "--build", case / "build")
        env = {**os.environ, "PATH": str(sdk / "src") + os.pathsep + os.environ["PATH"]}
        bullet_fixture_run(
            case
            / "build"
            / ("provider_consumer.exe" if os.name == "nt" else "provider_consumer"),
            env=env,
        )
    bullet_fixture_run(CMAKE, "--install", case / "build")
    assert not (tmp_path / "installed").exists()
    assert (sdk / "bullet.pc").read_bytes() == original_pc


@pytest.mark.parametrize(
    "mode",
    [
        "adapter-source",
        "direct-root-source",
        "alias-source",
        "adapter-no-source",
        "upstream-source",
        "upstream-nested-source",
    ],
)
def test_bullet_provider_source_targets_keep_ownership(tmp_path, mode):
    source = bullet_fallback_source(tmp_path)
    adapter = tmp_path / "adapter"
    adapter.mkdir()
    sdk = tmp_path / "provider-build"
    sdk.mkdir()
    stage = tmp_path / "installed"
    upstream = mode in ("upstream-source", "upstream-nested-source")
    target = "provider_${name}" if mode == "alias-source" else "${name}"
    targets = (
        "foreach(name BulletCollision LinearMath)\n"
        f'add_library({target} SHARED "{source.as_posix()}/src/${{name}}.c")\n'
        f"set_target_properties({target} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)\n"
        f'target_include_directories({target} PUBLIC "{source.as_posix()}/src")\n'
        + (f"add_library(${{name}} ALIAS {target})\n" if mode == "alias-source" else "")
        + "endforeach()\n"
    )
    (adapter / "CMakeLists.txt").write_text(targets)
    if mode == "direct-root-source":
        (source / "CMakeLists.txt").write_text(targets)
    elif mode == "alias-source":
        (source / "CMakeLists.txt").write_text("add_subdirectory(src)\n")
        (source / "src/CMakeLists.txt").write_text(targets)
    elif mode == "upstream-nested-source":
        # Both target directories may be arbitrary descendants of upstream src.
        nested = source / "src/provider/nested"
        nested.mkdir(parents=True)
        (source / "src/CMakeLists.txt").write_text(
            "add_subdirectory(provider/nested)\n"
        )
        (nested / "CMakeLists.txt").write_text(targets)
    directory = (
        source
        if upstream or mode in ("direct-root-source", "alias-source")
        else adapter
    )
    marker = source / "provider-owned.txt"
    marker.write_text("Provider install rules remain active.\n")
    install_directory = (
        (nested if mode == "upstream-nested-source" else source / "src")
        if upstream
        else directory
    )
    with (install_directory / "CMakeLists.txt").open("a") as cmakelists:
        cmakelists.write(
            f'install(FILES "{marker.as_posix()}" DESTINATION share/provider-sdk)\n'
        )
    original_pc = b"Name: provider-owned\nLibs: -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath\n"
    (sdk / "bullet.pc").write_bytes(original_pc)
    metadata = (
        f'SOURCE_DIR "{source.as_posix()}"' if mode != "adapter-no-source" else ""
    )
    provider = tmp_path / "provider.cmake"
    provider.write_text(
        "macro(bullet_provider method name)\n"
        f'add_subdirectory("{directory.as_posix()}" "{sdk.as_posix()}")\n'
        f'configure_file("{sdk.as_posix()}/bullet.pc" "{tmp_path.as_posix()}/provider.pc.before" COPYONLY)\n'
        f'set(BULLET_PHYSICS_SOURCE_DIR "{source.as_posix()}" CACHE PATH "" FORCE)\n'
        f'FetchContent_SetPopulated(${{name}} {metadata} BINARY_DIR "{sdk.as_posix()}")\n'
        "endmacro()\n"
        "cmake_language(SET_DEPENDENCY_PROVIDER bullet_provider SUPPORTED_METHODS FETCHCONTENT_MAKEAVAILABLE_SERIAL)\n"
    )
    case = tmp_path / "case"
    case.mkdir()
    result = configure(
        case,
        'set(DART_BULLET_SOURCE_DIR "/stale/fetched" CACHE INTERNAL "")\n'
        'set(DART_BULLET_BINARY_DIR "/stale/build" CACHE INTERNAL "")\n'
        "set(FETCHCONTENT_FULLY_DISCONNECTED ON)\n"
        + bullet_fallback_content()
        + "include(DARTFindBullet)\n"
        'if(NOT BULLET_FOUND)\nmessage(FATAL_ERROR "Valid source provider rejected")\nendif()\n'
        + 'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/fetched-cache.txt" "$CACHE{DART_BULLET_SOURCE_DIR};$CACHE{DART_BULLET_BINARY_DIR}")\n'
        + 'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.c" "#include <btBulletCollisionCommon.h>\\nint main(void) { return fixture_BulletCollision() + fixture_LinearMath() == 2 ? 0 : 1; }\\n")\n'
        'add_executable(provider_consumer "${CMAKE_CURRENT_BINARY_DIR}/main.c")\n'
        "target_link_libraries(provider_consumer PRIVATE Bullet)\n",
        language="C",
        cmake_args=(
            f"-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES={provider.as_posix()}",
            f"-DCMAKE_INSTALL_PREFIX={stage.as_posix()}",
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    bullet_fixture_run(CMAKE, "--build", case / "build")
    dll_dirs = sorted({str(path.parent) for path in sdk.rglob("*.dll")})
    env = {**os.environ, "PATH": os.pathsep.join([*dll_dirs, os.environ["PATH"]])}
    bullet_fixture_run(
        case
        / "build"
        / ("provider_consumer.exe" if os.name == "nt" else "provider_consumer"),
        env=env,
    )
    bullet_fixture_run(CMAKE, "--install", case / "build")
    installed_marker = stage / "share/provider-sdk/provider-owned.txt"
    assert (
        installed_marker.exists(),
        (sdk / "bullet.pc").read_bytes()
        == (tmp_path / "provider.pc.before").read_bytes(),
    ) == (True, True)
    assert installed_marker.read_bytes() == marker.read_bytes()
    assert (case / "build/fetched-cache.txt").read_text() == ";"


@pytest.mark.parametrize("mode", ["regular", "imported", "alias"])
@pytest.mark.parametrize("version", [325, 304, None])
def test_bullet_provider_current_source_metadata(tmp_path, mode, version):
    source = bullet_fallback_source(tmp_path)
    (source / "src/LinearMath/btScalar.h").write_text(
        (f"#define BT_BULLET_VERSION {version}\n" if version else "")
        + "#ifdef BT_USE_DOUBLE_PRECISION\ntypedef double btScalar;\n"
        "#else\ntypedef float btScalar;\n#endif\n"
        "int fixture_LinearMath(void);\n"
    )
    sdk = tmp_path / "provider-build"
    sdk.mkdir()
    original_pc = b"Name: provider-owned\nLibs: -lBulletCollision -lLinearMath\n"
    if mode == "imported":
        bullet_fixture_run(CMAKE, "-S", source, "-B", sdk, "-G", "Ninja")
        bullet_fixture_run(
            CMAKE, "--build", sdk, "--target", "BulletCollision", "LinearMath"
        )
    (sdk / "bullet.pc").write_bytes(original_pc)
    targets = ""
    for name in ("BulletCollision", "LinearMath"):
        target = f"provider_{name}" if mode == "alias" else name
        if mode == "imported":
            library = (sdk / f"{name}-link.txt").read_text()
            targets += (
                f"add_library({target} UNKNOWN IMPORTED GLOBAL)\n"
                f'set_target_properties({target} PROPERTIES IMPORTED_LOCATION "{library}")\n'
            )
        else:
            targets += (
                f'add_library({target} STATIC "{source.as_posix()}/src/{name}.c")\n'
            )
        visibility = "INTERFACE" if mode == "imported" else "PUBLIC"
        targets += (
            f'target_include_directories({target} {visibility} "{source.as_posix()}/src")\n'
            f"target_compile_definitions({target} {visibility} BT_USE_DOUBLE_PRECISION)\n"
        )
        if mode == "alias":
            targets += f"add_library({name} ALIAS {target})\n"
    provider = tmp_path / "provider.cmake"
    provider.write_text(
        "macro(bullet_provider method name)\n"
        + targets
        + f'set(BULLET_PHYSICS_SOURCE_DIR "{source.as_posix()}")\n'
        + f'FetchContent_SetPopulated(${{name}} SOURCE_DIR "{source.as_posix()}" BINARY_DIR "{sdk.as_posix()}")\n'
        + "endmacro()\n"
        "cmake_language(SET_DEPENDENCY_PROVIDER bullet_provider SUPPORTED_METHODS FETCHCONTENT_MAKEAVAILABLE_SERIAL)\n"
    )
    case = tmp_path / "case"
    case.mkdir()
    accepted = version == 325
    result = configure(
        case,
        'set(DART_BULLET_SOURCE_DIR "/stale/fetched" CACHE INTERNAL "")\n'
        'set(DART_BULLET_BINARY_DIR "/stale/build" CACHE INTERNAL "")\n'
        "set(FETCHCONTENT_FULLY_DISCONNECTED ON)\n"
        + bullet_fallback_content()
        + "include(DARTFindBullet)\n"
        + f'if({"NOT " if accepted else ""}BULLET_FOUND)\nmessage(FATAL_ERROR "Wrong provider header validation")\nendif()\n'
        + 'if(DEFINED CACHE{DART_BULLET_SOURCE_DIR} OR DEFINED CACHE{DART_BULLET_BINARY_DIR} OR DEFINED CACHE{BULLET_PHYSICS_SOURCE_DIR})\nmessage(FATAL_ERROR "Provider metadata leaked into cache")\nendif()\n'
        + (
            'if(NOT BULLET_VERSION VERSION_EQUAL "3.25")\nmessage(FATAL_ERROR "Wrong provider version")\nendif()\n'
            'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.c" "#include <btBulletCollisionCommon.h>\\ntypedef char fixture_double_precision[(sizeof(btScalar) == sizeof(double)) ? 1 : -1];\\nint main(void) { return fixture_BulletCollision() + fixture_LinearMath() == 2 ? 0 : 1; }\\n")\n'
            'add_executable(provider_consumer "${CMAKE_CURRENT_BINARY_DIR}/main.c")\n'
            "target_link_libraries(provider_consumer PRIVATE Bullet)\n"
            if accepted
            else ""
        ),
        language="C",
        cmake_args=(
            f"-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES={provider.as_posix()}",
            f"-DCMAKE_INSTALL_PREFIX={(tmp_path / 'installed').as_posix()}",
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if accepted:
        bullet_fixture_run(CMAKE, "--build", case / "build")
        env = {**os.environ, "PATH": str(sdk / "src") + os.pathsep + os.environ["PATH"]}
        bullet_fixture_run(
            case
            / "build"
            / ("provider_consumer.exe" if os.name == "nt" else "provider_consumer"),
            env=env,
        )
    bullet_fixture_run(CMAKE, "--install", case / "build")
    assert not (tmp_path / "installed").exists()
    assert (sdk / "bullet.pc").read_bytes() == original_pc


@pytest.mark.parametrize(
    "mode",
    [
        "fallback",
        "decline",
        "delegate",
        "populate-only",
        "parent-first",
        "partial-collision",
        "partial-math",
        "no-source",
    ],
)
def test_bullet_fallback_dispatch(tmp_path, mode):
    source = bullet_fallback_source(tmp_path)
    root_cmake = source / "CMakeLists.txt"
    root_cmake.write_text(
        root_cmake.read_text().replace("VERSION 3.28", "VERSION 2.4.3")
        + 'file(APPEND "${CMAKE_BINARY_DIR}/sdk-configured.txt" "configured\\n")\n'
    )
    marker = source / "src/provider-owned.txt"
    marker.write_text("Parent install rule.\n")
    with (source / "src/CMakeLists.txt").open("a") as cmakelists:
        cmakelists.write(
            "install(FILES provider-owned.txt DESTINATION share/provider-sdk)\n"
        )
    archive = tmp_path / "bullet fixture.tar.gz"
    with tarfile.open(archive, "w:gz") as packed:
        packed.add(source, arcname="bullet")
    content = (
        bullet_fallback_content()
        .replace(
            "https://github.com/bulletphysics/bullet3/archive/refs/tags/3.25.tar.gz",
            f'"{archive.as_posix()}"',
        )
        .replace(
            "c45afb6399e3f68036ddb641c6bf6f552bf332d5ab6be62f7e6c54eda05ceb77",
            hashlib.sha256(archive.read_bytes()).hexdigest(),
        )
    )
    sdk = tmp_path / "provider-build"
    provider = tmp_path / "provider.cmake"
    args = [f"-DCMAKE_INSTALL_PREFIX={(tmp_path / 'installed').as_posix()}"]
    if mode not in ("fallback", "parent-first"):
        body = 'file(APPEND "${CMAKE_BINARY_DIR}/provider-called.txt" "called\\n")\n'
        if mode == "delegate":
            body += "FetchContent_MakeAvailable(bullet)\n"
        elif mode != "decline":
            if mode.startswith("partial-"):
                name = (
                    "BulletCollision" if mode == "partial-collision" else "LinearMath"
                )
                body += (
                    f'add_library({name} STATIC "{source.as_posix()}/src/{name}.c")\n'
                )
            metadata = (
                ""
                if mode == "no-source"
                else f'SOURCE_DIR "{source.as_posix()}" BINARY_DIR "{sdk.as_posix()}"'
            )
            body += f"FetchContent_SetPopulated(${{name}} {metadata})\n"
        provider.write_text(
            "macro(bullet_provider method name)\n" + body + "endmacro()\n"
            "cmake_language(SET_DEPENDENCY_PROVIDER bullet_provider SUPPORTED_METHODS FETCHCONTENT_MAKEAVAILABLE_SERIAL)\n"
        )
        args.append(f"-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES={provider.as_posix()}")
    setup = 'include(FetchContent)\nset(CMAKE_POLICY_VERSION_MINIMUM "3.4")\n'
    if mode == "parent-first":
        setup += f'FetchContent_Declare(bullet SOURCE_DIR "{source.as_posix()}" BINARY_DIR "{sdk.as_posix()}")\n'
    result = configure(
        tmp_path,
        setup
        + content
        + 'if(NOT CMAKE_POLICY_VERSION_MINIMUM STREQUAL "3.4")\nmessage(FATAL_ERROR "Caller policy minimum changed")\nendif()\n'
        + "include(DARTFindBullet)\n"
        'if(NOT BULLET_FOUND)\nmessage(FATAL_ERROR "Lost source headers")\nendif()\n'
        'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.c" "#include <btBulletCollisionCommon.h>\\nint main(void) { return fixture_BulletCollision()+fixture_LinearMath()==2 ? 0 : 1; }\\n")\n'
        'add_executable(consumer "${CMAKE_CURRENT_BINARY_DIR}/main.c")\n'
        "target_link_libraries(consumer PRIVATE Bullet)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/owned.txt" "$CACHE{DART_BULLET_SOURCE_DIR}")\n',
        language="C",
        cmake_args=args,
    )
    build = tmp_path / "build"
    if mode.startswith("partial-") or mode == "no-source":
        assert result.returncode != 0
        assert (
            "Bullet must provide"
            if mode.startswith("partial-")
            else "Bullet requires complete"
        ) in result.stdout + result.stderr
        assert not (build / "sdk-configured.txt").exists()
        assert "already exists" not in result.stdout + result.stderr
        return
    assert result.returncode == 0, result.stdout + result.stderr
    assert (build / "sdk-configured.txt").read_text() == "configured\n"
    if provider.exists():
        assert (build / "provider-called.txt").read_text() == "called\n"
    owned = mode != "parent-first"
    assert bool((build / "owned.txt").read_text()) == owned
    bullet_fixture_run(CMAKE, "--build", build, "--target", "consumer")
    if not owned:
        bullet_fixture_run(
            CMAKE, "--build", build, "--target", "BulletDynamics", "BulletSoftBody"
        )
    dll_dirs = sorted({str(path.parent) for path in tmp_path.rglob("*.dll")})
    env = {**os.environ, "PATH": os.pathsep.join([*dll_dirs, os.environ["PATH"]])}
    bullet_fixture_run(
        build / ("consumer.exe" if os.name == "nt" else "consumer"), env=env
    )
    bullet_fixture_run(CMAKE, "--install", build)
    stage = tmp_path / "installed"
    assert (stage / "share/provider-sdk/provider-owned.txt").exists() != owned
    pc = (stage / "lib/pkgconfig/bullet.pc").read_text()
    assert ("BulletDynamics" not in pc) == owned
    if owned:
        assert (stage / "include/btBulletCollisionCommon.h").is_file()
        assert not (stage / "include/bullet/BulletDynamics").exists()


@pytest.mark.parametrize(
    "package,minimum,header,defines,target",
    [
        (
            "tinyxml2",
            "10.0.0",
            "tinyxml2.h",
            "#define TINYXML2_MAJOR_VERSION 10\n#define TINYXML2_MINOR_VERSION 0\n#define TINYXML2_PATCH_VERSION 0\n",
            "tinyxml2::tinyxml2",
        ),
        (
            "ODE",
            "0.16.2",
            "ode/version.h",
            '#define dODE_VERSION "0.16.2"\n',
            "ODE::ODE",
        ),
        (
            "imgui",
            "1.92.8",
            "imgui.h",
            '#define IMGUI_VERSION "1.92.8"\n',
            "imgui::imgui",
        ),
    ],
)
def test_config_header_version_fallback(
    tmp_path, package, minimum, header, defines, target
):
    include = tmp_path / "include"
    path = include / header
    path.parent.mkdir(parents=True)
    path.write_text(defines)
    extra = f'set_target_properties({target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "{include.as_posix()}")\n'
    if package == "ODE":
        extra += 'set_target_properties(ODE::ODE PROPERTIES INTERFACE_COMPILE_DEFINITIONS "dLIBCCD_BOX_CYL;dDOUBLE")\n'
    config_package(tmp_path, package, None, target, extra)
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_ODE ON)\n"
        f"include(DARTFind{package})\n"
        f'if(NOT {package}_FOUND)\nmessage(FATAL_ERROR "Version missing")\nendif()\n'
        + (
            f"if(NOT TINYXML2_VERSION VERSION_EQUAL {minimum})\n"
            if package == "tinyxml2"
            else f"if(NOT {package}_VERSION VERSION_EQUAL {minimum})\n"
        )
        + 'message(FATAL_ERROR "Incorrect header version")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    "package,minimum,prefix,header,definition",
    [
        ("assimp", "5.3.0", "ASSIMP", "assimp/scene.h", None),
        ("fcl", "0.7.0", "FCL", "fcl/config.h", "FCL_VERSION"),
        ("tinyxml2", "10.0.0", "TINYXML2", "tinyxml2.h", "TINYXML2"),
        ("ODE", "0.16.2", "ODE", "ode/version.h", "dODE_VERSION"),
        ("imgui", "1.92.8", "imgui", "imgui.h", "IMGUI_VERSION"),
    ],
)
@pytest.mark.parametrize("version_kind", ["minimum", "older", "unknown"])
def test_module_floors(
    tmp_path, package, minimum, prefix, header, definition, version_kind
):
    include = tmp_path / "include"
    path = include / header
    path.parent.mkdir(parents=True)
    version = minimum if version_kind == "minimum" else "0.1.0"
    if package == "tinyxml2" and version_kind == "older":
        version = "9.0.0"
    content = ""
    if version_kind != "unknown" and definition:
        if package == "tinyxml2":
            content = "".join(
                f"#define TINYXML2_{part}_VERSION {number}\n"
                for part, number in zip(("MAJOR", "MINOR", "PATCH"), version.split("."))
            )
        else:
            content = f'#define {definition} "{version}"\n'
    path.write_text(content)
    library = tmp_path / "library"
    library.touch()
    variables = (
        f'set({prefix}_INCLUDE_DIRS "{include.as_posix()}")\n'
        f'set({prefix}_LIBRARIES "{library.as_posix()}")\n'
    )
    if package == "assimp" and version_kind != "unknown":
        variables += f"set(assimp_VERSION {version})\n"
    if package == "imgui":
        variables += f'set(imgui_INCLUDE_DIR "{include.as_posix()}")\n'
    accepted = version_kind == "minimum"
    result = configure(
        tmp_path,
        variables
        + (
            "include(DARTFindtinyxml2)\n"
            if package == "tinyxml2"
            else f"find_package({package} {minimum} QUIET MODULE)\n"
        )
        + f"if({package}_FOUND OR {prefix}_FOUND)\n"
        + ("" if accepted else 'message(FATAL_ERROR "Accepted unsupported version")\n')
        + "else()\n"
        + ('message(FATAL_ERROR "Rejected supported version")\n' if accepted else "")
        + "endif()\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("preset", ["cache", "normal"])
@pytest.mark.parametrize("msvc", [False, True])
@pytest.mark.parametrize(
    "version,accepted",
    [("6.0.5", True), ("5.3.0", True), ("5.2.2", False), ("5.2.0", False)],
)
def test_assimp_version_from_config_without_pkg_config(
    tmp_path, version, accepted, msvc, preset
):
    # Without pkg-config, assimp's CMake package supplies the version. Selected
    # paths win over the variables that package sets, except that MSVC builds
    # link the package's target, as Findassimp.cmake always did.
    config_package(
        tmp_path,
        "assimp",
        version,
        "assimp::assimp",
        'set(ASSIMP_INCLUDE_DIRS "config-include")\n'
        "set(ASSIMP_LIBRARIES assimp::assimp)\n",
    )
    include = tmp_path / "include"
    (include / "assimp").mkdir(parents=True)
    (include / "assimp" / "scene.h").touch()
    library = tmp_path / "library"
    library.touch()
    expected = "assimp::assimp" if msvc else library.as_posix()
    path_cache = ' CACHE PATH ""' if preset == "cache" else ""
    file_cache = ' CACHE FILEPATH ""' if preset == "cache" else ""
    result = configure(
        tmp_path,
        f"set(MSVC {'TRUE' if msvc else 'FALSE'})\n"
        f'set(ASSIMP_INCLUDE_DIRS "{include.as_posix()}"{path_cache})\n'
        f'set(ASSIMP_LIBRARIES "{library.as_posix()}"{file_cache})\n'
        "include(DARTFindassimp)\n"
        "get_target_property(libraries assimp INTERFACE_LINK_LIBRARIES)\n"
        f"if(NOT ASSIMP_VERSION VERSION_EQUAL {version}\n"
        f'   OR NOT libraries STREQUAL "{expected}")\n'
        '  message(FATAL_ERROR "Wrong assimp: ${ASSIMP_VERSION} ${libraries}")\n'
        "endif()\n",
    )
    assert (result.returncode == 0) == accepted, result.stdout + result.stderr


@pytest.mark.parametrize("module", ["dart", "foreign"])
def test_assimp_module_lookup_skips_config(tmp_path, module):
    # When the module lookup succeeds, DART must not load assimp's CMake
    # package, so lookups that already work stay unchanged. A consumer's own
    # Findassimp.cmake comes first and may set only assimp_FOUND.
    config_package(
        tmp_path, "assimp", "5.3.0", None, 'message(FATAL_ERROR "Loaded config")\n'
    )
    include = tmp_path / "include"
    (include / "assimp").mkdir(parents=True)
    (include / "assimp" / "scene.h").touch()
    library = tmp_path / "library"
    library.touch()
    lookup = (
        f'set(ASSIMP_INCLUDE_DIRS "{include.as_posix()}")\n'
        f'set(ASSIMP_LIBRARIES "{library.as_posix()}")\n'
    )
    if module == "foreign":
        (tmp_path / "modules").mkdir()
        (tmp_path / "modules" / "Findassimp.cmake").write_text(
            lookup + "set(assimp_FOUND TRUE)\n"
        )
        lookup = ""
    result = configure(
        tmp_path,
        lookup + "set(assimp_VERSION 5.3.0)\n"
        "include(DARTFindassimp)\n"
        'if(NOT TARGET assimp)\n  message(FATAL_ERROR "No assimp target")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_bundled_ode_version(tmp_path):
    source = tmp_path / "ode-source"
    header = source / "include" / "ode" / "version.h"
    header.parent.mkdir(parents=True)
    header.write_text('#define dODE_VERSION "0.16.6"\n')
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_ODE OFF)\n"
        f'set(DART_ODE_SOURCE_DIR "{source.as_posix()}")\n'
        "add_library(ODE INTERFACE)\n"
        "set_target_properties(ODE PROPERTIES INTERFACE_COMPILE_DEFINITIONS dLIBCCD_BOX_CYL)\n"
        "include(DARTFindODE)\n"
        'if(NOT ODE_FOUND OR NOT TARGET ODE::ODE OR ODE_VERSION VERSION_LESS 0.16.2)\nmessage(FATAL_ERROR "Missing bundled ODE")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("version", ["0.13.0", "0.16.2", None])
@pytest.mark.parametrize("use_version_metadata", [False, True])
def test_parent_ode_target_version(tmp_path, version, use_version_metadata):
    include = tmp_path / "ode-parent" / "include"
    header = include / "ode" / "version.h"
    header.parent.mkdir(parents=True)
    header.write_text(
        f'#define dODE_VERSION "{version}"\n'
        if version and not use_version_metadata
        else ""
    )
    content = (
        "add_library(ODE INTERFACE)\n"
        f'set_target_properties(ODE PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "{include.as_posix()}" '
        "INTERFACE_COMPILE_DEFINITIONS dLIBCCD_BOX_CYL)\n"
    )
    if version and use_version_metadata:
        content += f"set(ODE_VERSION {version})\n"
    content += "include(DARTFindODE)\n"
    if version == "0.16.2":
        content += 'if(NOT ODE_FOUND OR NOT ode_FOUND)\nmessage(FATAL_ERROR "Rejected parent ODE")\nendif()\n'
    else:
        content += 'if(ODE_FOUND OR ode_FOUND)\nmessage(FATAL_ERROR "Accepted unsupported parent ODE")\nendif()\n'
    result = configure(tmp_path, content)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("package", ["tinyxml2", "urdfdom", "ODE"])
def test_rejected_config_resets_mixed_case_flags(tmp_path, package):
    config_package(tmp_path, package, "0.1.0")
    result = configure(
        tmp_path,
        f"set({package.upper()}_FOUND TRUE)\n"
        f"include(DARTFind{package})\n"
        f'if({package}_FOUND OR {package.upper()}_FOUND)\nmessage(FATAL_ERROR "Stale found flag")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_filament_explicit_root_rejects_an_incomplete_sdk(tmp_path):
    config_package(tmp_path, "Filament", None, "Filament::filament")
    root = tmp_path / "incomplete-sdk"
    (root / "include" / "filament").mkdir(parents=True)
    (root / "include" / "filament" / "Engine.h").touch()
    result = configure(
        tmp_path,
        f'set(Filament_ROOT "{root.as_posix()}")\n'
        "find_package(Filament QUIET MODULE)\n"
        'if(Filament_FOUND)\nmessage(FATAL_ERROR "Mixed incomplete SDK with prefix")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("explicit_root", [False, True])
def test_filament_config_case_variants_are_loaded_once(tmp_path, explicit_root):
    config_package(tmp_path, "Filament", None, "Filament::filament")
    config = tmp_path / "prefix/lib/cmake/Filament/FilamentConfig.cmake"
    # Both spellings reach the same SDK on case-insensitive filesystems.
    config.with_name("filamentConfig.cmake").write_text(config.read_text())
    result = configure(
        tmp_path,
        (
            f'set(Filament_ROOT "{(tmp_path / "prefix").as_posix()}")\n'
            if explicit_root
            else ""
        )
        + "find_package(Filament REQUIRED MODULE)\n"
        + 'if(NOT TARGET Filament::filament)\nmessage(FATAL_ERROR "Missing SDK target")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_filament_changed_root_invalidates_cached_package_directory(tmp_path):
    config_package(tmp_path, "Filament", None, "Filament::filament")
    original_root = tmp_path / "prefix"
    result = configure(
        tmp_path,
        f'set(Filament_ROOT "{original_root.as_posix()}")\n'
        "find_package(Filament REQUIRED MODULE)\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    replacement_root = tmp_path / "replacement-sdk"
    replacement_root.mkdir()
    result = configure(
        tmp_path,
        f'set(Filament_ROOT "{replacement_root.as_posix()}")\n'
        "find_package(Filament QUIET MODULE)\n"
        'if(Filament_FOUND)\nmessage(FATAL_ERROR "Kept the previous SDK")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(os.name != "posix", reason="fake matc is a POSIX shell script")
@pytest.mark.parametrize(
    "material_version,accepted", [("77", True), ("76", False), ("78", False)]
)
def test_filament_system_sdk_requires_pinned_material_version(
    tmp_path, material_version, accepted
):
    # The SDK exposes no release version, so the provider enforces the 1.77.3
    # pin through the material format version that upstream bumps each minor
    # release. Stub the unrelated GUI dependencies to isolate that check.
    sdk = tmp_path / "filament-sdk"
    (sdk / "include" / "filament").mkdir(parents=True)
    (sdk / "include" / "filament" / "Engine.h").touch()
    (sdk / "include" / "filament" / "MaterialEnums.h").write_text(
        f"static constexpr size_t MATERIAL_VERSION = {material_version};\n"
    )
    lib = sdk / "lib"
    lib.mkdir()
    for name in ("filament", "backend", "filabridge", "filaflat", "utils", "geometry"):
        (lib / f"lib{name}.a").touch()
    matc = sdk / "bin" / "matc"
    matc.parent.mkdir()
    matc.write_text(f"#!/bin/sh\necho {material_version}\n")
    matc.chmod(0o755)
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "FindPNG.cmake").write_text("set(PNG_FOUND TRUE)\n")
    (modules / "FindJPEG.cmake").write_text("set(JPEG_FOUND TRUE)\n")
    (modules / "FindThreads.cmake").write_text(
        "set(Threads_FOUND TRUE)\nadd_library(Threads::Threads INTERFACE IMPORTED)\n"
    )
    (modules / "DARTFilamentNotices.cmake").write_text(
        "function(dart_filament_stage_notices prefix)\nendfunction()\n"
    )
    config_package(tmp_path, "glfw3", "3.4.0", "glfw")
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_FILAMENT ON)\n"
        f'set(Filament_ROOT "{sdk.as_posix()}")\n'
        "include(DARTFilamentProvider)\n",
    )
    output = result.stdout + result.stderr
    if accepted:
        assert result.returncode == 0, output
    else:
        # CMake wraps long diagnostics, so match against flattened output.
        flat = " ".join(output.split())
        assert result.returncode != 0
        assert f"material version {material_version}" in flat
        assert "Filament 1.77.3" in flat
        assert "DART_USE_SYSTEM_FILAMENT=OFF" in flat


@pytest.mark.parametrize(
    "package,version,target,accepted",
    [
        ("sdformat14", "14.8.0", "sdformat14::sdformat14", True),
        ("sdformat14", "14.7.0", "sdformat14::sdformat14", False),
        ("sdformat15", "15.0.0", "sdformat15::sdformat15", True),
        ("sdformat16", "16.1.0", "sdformat16::sdformat16", True),
        ("sdformat", "16.1.0", "sdformat::sdformat16", True),
        ("sdformat", "15.3.0", "sdformat::sdformat", True),
        ("sdformat", "13.9.0", "sdformat::sdformat", False),
        ("sdformat", "17.0.0", "sdformat::sdformat", False),
        ("sdformat15", None, "sdformat15::sdformat15", False),
        ("sdformat15", "15.3.0", None, False),
        ("sdformat15", "16.1.0", "sdformat15::sdformat15", False),
    ],
)
def test_sdformat_supported_packages(tmp_path, package, version, target, accepted):
    config_package(tmp_path, package, version, target)
    result = configure(
        tmp_path,
        "include(DARTFindsdformat)\n"
        f"if(NOT SDFORMAT_FOUND STREQUAL {'TRUE' if accepted else 'FALSE'})\n"
        'message(FATAL_ERROR "Incorrect sdformat acceptance")\nendif()\n'
        + (
            f'if(NOT SDFORMAT_LIBRARIES STREQUAL "{target}" OR NOT DART_SDFORMAT_PACKAGE STREQUAL "{package}")\n'
            'message(FATAL_ERROR "Incorrect sdformat selection")\nendif()\n'
            if accepted
            else ""
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("unversioned", [False, True])
def test_sdformat_prefers_newest_supported_major(tmp_path, unversioned):
    config_package(tmp_path, "sdformat14", "14.8.0", "sdformat14::sdformat14")
    config_package(tmp_path, "sdformat15", "15.3.0", "sdformat15::sdformat15")
    config_package(
        tmp_path,
        "sdformat" if unversioned else "sdformat16",
        "16.1.0",
        "sdformat::sdformat16" if unversioned else "sdformat16::sdformat16",
    )
    result = configure(
        tmp_path,
        "include(DARTFindsdformat)\n"
        'if(NOT DART_SDFORMAT_MAJOR STREQUAL "16")\n'
        'message(FATAL_ERROR "Did not select newest sdformat")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("installed_major", [14, 15, 16])
@pytest.mark.parametrize("package", ["sdformat15", "sdformat"])
def test_sdformat_installed_component_pins_major(tmp_path, installed_major, package):
    target = f"{package}::{package}"
    config_package(tmp_path, package, f"{installed_major}.8.0", target)
    # A different package at the requested major must not satisfy the pin.
    if package == "sdformat":
        config_package(tmp_path, "sdformat15", "15.3.0", "sdformat15::sdformat15")
    template = (ROOT / "cmake/dart_Component.cmake.in").read_text()
    for name, value in {
        "PROJECT_NAME": "dart",
        "PROJECT_NAME_UPPERCASE": "DART",
        "component": "io",
        "internal_dependencies": "dart",
        "libraries": "dart-io",
        "external_dependencies": "sdformat",
        "DART_SDFORMAT_PACKAGE": package,
        "DART_SDFORMAT_MAJOR": "15",
    }.items():
        template = template.replace(f"@{name}@", value)
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "dart_ioComponent.cmake").write_text(template)
    shutil.copy(ROOT / "cmake/DARTFindsdformat.cmake", modules)
    result = configure(
        tmp_path,
        "include(dart_ioComponent)\n"
        "get_property(found DIRECTORY PROPERTY DART_io_FOUND)\n"
        f"if(NOT found STREQUAL {'TRUE' if installed_major == 15 else 'FALSE'})\n"
        'message(FATAL_ERROR "Incorrect installed sdformat major")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_io_disabled_does_not_discover_sdformat(tmp_path):
    source = (ROOT / "dart/CMakeLists.txt").read_text()
    start = source.index("if(DART_BUILD_IO)")
    io_addition = source[start : source.index("endif()", start) + len("endif()")]
    result = configure(
        tmp_path,
        "set(DART_BUILD_IO OFF)\n"
        "function(add_subdirectory)\n"
        'message(FATAL_ERROR "IO should not be configured")\nendfunction()\n'
        + io_addition,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("component", ["io", "utils", "utils-urdf"])
@pytest.mark.parametrize("major", [15, 16])
def test_installed_legacy_components_propagate_sdformat_failure(
    tmp_path, component, major
):
    config_package(tmp_path, "sdformat", f"{major}.3.0", "sdformat::sdformat")
    prefix = tmp_path / "prefix"
    (prefix / "include").mkdir()
    package_dir = prefix / "lib/cmake/DART"
    package_dir.mkdir(parents=True)
    substitutions = {
        "PROJECT_NAME": "dart",
        "PROJECT_NAME_UPPERCASE": "DART",
        "PACKAGE_INIT": "macro(set_and_check name value)\nset(${name} ${value})\nendmacro()",
        "PACKAGE_INCLUDE_INSTALL_DIR": (prefix / "include").as_posix(),
        "PACKAGE_INCLUDE_DIRS": "",
        "DART_USE_SYSTEM_IMGUI": "ON",
        "DART_VERSION": "6.20.0",
        "DART_MAJOR_VERSION": "6",
        "DART_SDFORMAT_PACKAGE": "sdformat",
        "DART_SDFORMAT_MAJOR": "15",
    }

    def render(template, **values):
        for name, value in (substitutions | values).items():
            template = template.replace(f"@{name}@", value)
        return template

    (package_dir / "DARTConfig.cmake").write_text(
        render((ROOT / "cmake/DARTConfig.cmake.in").read_text())
    )
    template = (ROOT / "cmake/dart_Component.cmake.in").read_text()
    for name, dependencies, target in (
        ("dart", "", "dart"),
        ("io", "dart", "dart-io"),
        ("utils", "io", "dart-utils"),
        ("utils-urdf", "utils", "dart-utils-urdf"),
    ):
        (package_dir / f"dart_{name}Component.cmake").write_text(
            render(
                template,
                component=name,
                internal_dependencies=dependencies,
                libraries=target,
                external_dependencies="sdformat" if name == "io" else "",
            )
        )
        (package_dir / f"dart_{name}Targets.cmake").write_text(
            f"add_library({target} INTERFACE IMPORTED)\n"
        )
    shutil.copy(ROOT / "cmake/DARTFindsdformat.cmake", package_dir)
    result = configure(
        tmp_path,
        f"find_package(DART QUIET CONFIG COMPONENTS {component})\n"
        f"if({'NOT DART_FOUND' if major == 15 else 'DART_FOUND'})\n"
        'message(FATAL_ERROR "Legacy component accepted incompatible sdformat")\nendif()\n'
        + (
            'if(NOT SDFORMAT_LIBRARIES STREQUAL "sdformat::sdformat")\n'
            'message(FATAL_ERROR "Selected SDK target did not reach installed callers")\nendif()\n'
            if major == 15
            else ""
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("minimum", [None, "3.5", "3.22"])
def test_legacy_fetch_policy_is_scoped(tmp_path, minimum):
    source = tmp_path / "legacy"
    source.mkdir()
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 2.8.12)\nproject(legacy NONE)\n"
        "if(CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS 3.5)\n"
        'message(FATAL_ERROR "Missing legacy policy minimum")\nendif()\n'
    )
    result = configure(
        tmp_path,
        "include(DARTMacros)\ninclude(FetchContent)\n"
        + (f"set(CMAKE_POLICY_VERSION_MINIMUM {minimum})\n" if minimum else "")
        + f'FetchContent_Declare(legacy SOURCE_DIR "{source.as_posix()}")\n'
        "dart_fetchcontent_makeavailable_legacy(legacy)\n"
        + (
            f'if(NOT CMAKE_POLICY_VERSION_MINIMUM STREQUAL "{minimum}")\n'
            if minimum
            else "if(DEFINED CMAKE_POLICY_VERSION_MINIMUM)\n"
        )
        + 'message(FATAL_ERROR "Policy minimum escaped dependency scope")\nendif()\n'
        + f'if(NOT legacy_SOURCE_DIR STREQUAL "{source.as_posix()}")\n'
        'message(FATAL_ERROR "Missing populated source directory")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


def ode_compiler_args(language, environ=os.environ, which=shutil.which):
    """Return CMake args for the probe compiler, or None when none is available."""
    if environ.get("CXX" if language == "CXX" else "CC"):
        return []  # CMake reads the configured compiler (and its flags) itself
    compiler = which("c++" if language == "CXX" else "cc")
    return [f"-DCMAKE_{language}_COMPILER={compiler}"] if compiler else None


@pytest.mark.parametrize("language", ["C", "CXX"])
def test_ode_compiler_args_honor_environment(language):
    variable = "CXX" if language == "CXX" else "CC"
    host = f"/mingw64/bin/{'c++' if language == 'CXX' else 'cc'}"
    which = lambda name: host
    assert ode_compiler_args(language, {variable: "cl"}, which) == []
    assert ode_compiler_args(language, {variable: "cl"}, lambda name: None) == []
    assert ode_compiler_args(language, {}, which) == [
        f"-DCMAKE_{language}_COMPILER={host}"
    ]
    assert ode_compiler_args(language, {variable: ""}, which) == [
        f"-DCMAKE_{language}_COMPILER={host}"
    ]
    assert ode_compiler_args(language, {}, lambda name: None) is None
    other = "CC" if language == "CXX" else "CXX"
    assert ode_compiler_args(language, {other: "cl"}, which) != []


@pytest.mark.parametrize("language", [None, "C", "CXX"])
def test_configure_initializes_language_before_isolation(
    tmp_path, monkeypatch, language
):
    tools = tmp_path / "sdk-tools"
    tools.mkdir()
    tool = tools / (
        "dart-fixture-sdk-tool.exe" if os.name == "nt" else "dart-fixture-sdk-tool"
    )
    tool.write_text("fixture tool; discovery only\n")
    tool.chmod(0o755)
    monkeypatch.setenv("PATH", str(tools) + os.pathsep + os.environ.get("PATH", ""))
    marker = tmp_path / "toolchain-initialized.txt"
    override = tmp_path / "toolchain-rules.cmake"
    override.write_text(
        f'find_program(fixture_sdk_tool NAMES "{tool.name}" NO_CACHE)\n'
        "if(NOT fixture_sdk_tool)\n"
        'message(FATAL_ERROR "Fixture SDK tool hidden during language initialization")\n'
        "endif()\n"
        f'file(WRITE "{marker.as_posix()}" "${{fixture_sdk_tool}}")\n'
    )
    args = [f"-DCMAKE_USER_MAKE_RULES_OVERRIDE={override.as_posix()}"]
    if language:
        compiler_args = ode_compiler_args(language)
        assert compiler_args is not None, "Fixture initialization needs a host compiler"
        args += compiler_args
        late = tmp_path / "late"
        late.mkdir()
        result = configure(late, f"enable_language({language})\n", cmake_args=args)
        assert result.returncode != 0
        assert "Fixture SDK tool hidden during language initialization" in result.stderr
        assert not marker.exists()
    body = (
        f'find_program(isolated_sdk_tool NAMES "{tool.name}" NO_CACHE)\n'
        "if(isolated_sdk_tool)\n"
        'message(FATAL_ERROR "Fixture SDK tool escaped dependency isolation")\n'
        "endif()\n"
    )
    if not language:
        body += (
            "if(CMAKE_C_COMPILER_LOADED OR CMAKE_CXX_COMPILER_LOADED)\n"
            'message(FATAL_ERROR "Default fixture enabled a language")\n'
            "endif()\n"
        )
    early = tmp_path / "early"
    early.mkdir()
    result = configure(
        early, body, cmake_args=args, **({"language": language} if language else {})
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert marker.exists() == bool(language)
    if language:
        assert marker.read_text() == tool.as_posix()


@pytest.mark.parametrize("language", ["C", "CXX"])
@pytest.mark.parametrize("precision", ["dSINGLE", "dDOUBLE"])
@pytest.mark.parametrize("target_type", [None, "EXECUTABLE", "STATIC_LIBRARY"])
def test_system_ode_precision(tmp_path, language, precision, target_type):
    compiler_args = ode_compiler_args(language)
    if compiler_args is None:
        pytest.skip("No host compiler available for the ODE header probe")
    include = tmp_path / "include"
    (include / "ode").mkdir(parents=True)
    (include / "ode/odeconfig.h").write_text(f"#define {precision}\n")
    config_package(
        tmp_path,
        "ODE",
        "0.16.6",
        "ODE::ODE",
        f'set_target_properties(ODE::ODE PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "{include.as_posix()}" '
        "INTERFACE_COMPILE_DEFINITIONS dLIBCCD_BOX_CYL "
        "INTERFACE_LINK_LIBRARIES missing_ode_link_dependency)\n",
    )
    result = configure(
        tmp_path,
        (f"set(CMAKE_TRY_COMPILE_TARGET_TYPE {target_type})\n" if target_type else "")
        + "set(DART_USE_SYSTEM_ODE ON)\ninclude(DARTFindODE)\n"
        + (
            f'if(NOT CMAKE_TRY_COMPILE_TARGET_TYPE STREQUAL "{target_type}")\n'
            if target_type
            else "if(DEFINED CMAKE_TRY_COMPILE_TARGET_TYPE)\n"
        )
        + 'message(FATAL_ERROR "Probe target type escaped")\nendif()\n',
        cmake_args=compiler_args,
        language=language,
    )
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == (precision == "dDOUBLE"), output
    if precision == "dSINGLE":
        assert "DART requires double-precision ODE" in output
        assert "DART_USE_SYSTEM_ODE=OFF" in output


def uninstall_prefix_name(
    prefix_kind, outside, fsdecode=os.fsdecode, windows=os.name == "nt"
):
    if prefix_kind == "spaces":
        return "install with spaces"
    if prefix_kind == "semicolon":
        return "install;with[bracket"
    if prefix_kind == "utf8":
        # An embedded drive-qualified path is not a legal Windows component.
        return "install-dé" if windows else "dé" + outside.as_posix()
    return fsdecode(b"d\xe9") + outside.as_posix()


@pytest.mark.parametrize("prefix_kind", ["spaces", "utf8", "semicolon"])
def test_uninstall_prefix_never_decodes_raw_bytes_unless_latin1(prefix_kind):
    def fail(raw):
        raise AssertionError("raw bytes decoded")

    outside = PureWindowsPath("C:/tmp/outside")
    for windows in (False, True):
        name = uninstall_prefix_name(prefix_kind, outside, fail, windows)
        if windows:
            assert PureWindowsPath("C:/tmp", name).parent == PureWindowsPath("C:/tmp")


@pytest.mark.parametrize("staged", [False, True])
@pytest.mark.parametrize("prefix_kind", ["spaces", "utf8", "latin1", "semicolon"])
def test_uninstall_manifest_preserves_spaces(tmp_path, staged, prefix_kind):
    if staged and os.name != "posix":
        pytest.skip("CMake DESTDIR staging requires a UNIX platform")
    outside = tmp_path / "outside"
    if prefix_kind == "latin1" and os.name != "posix":
        pytest.skip("Raw Latin-1 filenames require a UNIX platform")
    prefix_name = uninstall_prefix_name(prefix_kind, outside)
    prefix = tmp_path / prefix_name
    destdir = str(tmp_path / "stage") if staged else ""
    decoy = Path(destdir + str(outside / "installed header.hpp"))
    decoy.parent.mkdir(parents=True, exist_ok=True)
    decoy.touch()
    stage = Path(destdir + str(prefix))
    stage.mkdir(parents=True)
    installed = stage / "installed header.hpp"
    installed.touch()
    unlisted = stage / "unlisted"
    unlisted.touch()
    manifest = tmp_path / "install_manifest.txt"
    dangling = stage / "dangling link"
    if os.name == "posix":
        dangling.symlink_to("missing-target")
    manifest.write_bytes(
        os.fsencode(f"{(prefix / installed.name).as_posix()}\n")
        + os.fsencode(f"{(prefix / dangling.name).as_posix()}\n")
        + b"\nrelative-entry\n"
    )
    relative = tmp_path / "relative-entry"
    relative.touch()
    script = tmp_path / "uninstall.cmake"
    script.write_text(
        (ROOT / "cmake/uninstall_target.cmake.in")
        .read_text()
        .replace("@CMAKE_CURRENT_BINARY_DIR@", tmp_path.as_posix())
    )
    result = subprocess.run(
        [shutil.which("cmake"), "-P", str(script)],
        capture_output=True,
        text=True,
        errors="surrogateescape",
        check=False,
        cwd=tmp_path,
        env={**os.environ, "DESTDIR": destdir},
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert decoy.exists()
    assert not installed.exists()
    assert unlisted.exists()
    assert relative.exists()
    assert not dangling.is_symlink()


@pytest.mark.parametrize("force_legacy", [False, True])
def test_logging_float_fixture_builds_without_availability_suppression(
    tmp_path, force_legacy
):
    # Compile the real implementation and a C++14 consumer on either path.
    # Native Apple deployment/symbol checks run in the system-toolchain CI step.
    assert (
        "_LIBCPP_DISABLE_AVAILABILITY" not in (ROOT / "dart/CMakeLists.txt").read_text()
    )
    source = tmp_path / "source"
    source.mkdir()
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(LoggingFloat LANGUAGES CXX)\n"
        "find_package(Threads REQUIRED)\n"
        'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/dart/config.hpp" "#define DART_ACTIVE_LOG_LEVEL 0\\n")\n'
        f'add_library(logging STATIC "{ROOT.as_posix()}/dart/common/Logging.cpp" "{ROOT.as_posix()}/dart/common/Console.cpp")\n'
        "set_target_properties(logging PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)\n"
        f'target_include_directories(logging PUBLIC "{ROOT.as_posix()}" "${{CMAKE_CURRENT_BINARY_DIR}}")\n'
        "target_link_libraries(logging PUBLIC Threads::Threads)\n"
        f'add_executable(check_float "{ROOT.as_posix()}/scripts/check_logging_float.cpp")\n'
        "set_target_properties(check_float PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)\n"
        "target_link_libraries(check_float PRIVATE logging)\n"
        + (
            "target_compile_definitions(logging PRIVATE DART_LOGGING_FORCE_LEGACY_FLOAT=1)\n"
            "target_compile_definitions(check_float PRIVATE DART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES=1)\n"
            if force_legacy
            else ""
        )
        + "enable_testing()\nadd_test(NAME float_fixture COMMAND check_float 0)\n"
    )
    build = tmp_path / "build"
    commands = [
        [shutil.which("cmake"), "-G", "Ninja", "-S", str(source), "-B", str(build)],
        [shutil.which("cmake"), "--build", str(build), "--parallel", "2"],
        [shutil.which("ctest"), "--test-dir", str(build), "-V"],
    ]
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        assert result.returncode == 0, result.stdout + result.stderr
    assert re.search(
        r"fixed_checks=[1-9][0-9]* oracle_checks=0 failures=0", result.stdout
    )


def test_spdlog_finder_is_a_compatibility_noop(tmp_path):
    result = configure(
        tmp_path,
        "set(CMAKE_DISABLE_FIND_PACKAGE_spdlog TRUE)\n"
        "include(DARTFindspdlog)\n"
        "if(NOT spdlog_FOUND)\n"
        'message(FATAL_ERROR "Deprecated dependency should be satisfied")\n'
        "endif()\n"
        "if(TARGET spdlog::spdlog OR TARGET spdlog::spdlog_header_only)\n"
        'message(FATAL_ERROR "Deprecated finder created a dependency")\n'
        "endif()\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "DARTFindspdlog is deprecated" in result.stderr


def test_imgui_archive_upgrade_rebuilds_with_preserved_timestamps(tmp_path):
    tmp_path = tmp_path / "archive fixture"
    tmp_path.mkdir()
    module = (
        Path(__file__).resolve().parents[1] / "dart/gui/imgui/CMakeLists.txt"
    ).read_text()
    original_hash = re.search(r"\b[0-9a-f]{64}\b", module).group()
    url = "https://github.com/ocornut/imgui/archive/refs/tags/${IMGUI_TARGET_VERSION}.tar.gz"
    source = tmp_path / "source"
    source.mkdir()
    (source / "stub.cpp").write_text("// Parent target source.\n")
    (source / "main.cpp").write_text(
        "int archive_version(); int main() { return archive_version(); }\n"
    )
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22)\n"
        "project(dart LANGUAGES CXX)\n"
        'set(DART_BINARY_DIR "${CMAKE_BINARY_DIR}")\n'
        "set(DART_USE_SYSTEM_IMGUI OFF)\n"
        f'include("{(ROOT / "cmake/DARTMacros.cmake").as_posix()}")\n'
        "function(add_component_dependency_packages)\nendfunction()\n"
        "add_library(dart-gui STATIC stub.cpp)\n"
        'include("${CMAKE_CURRENT_SOURCE_DIR}/imgui.cmake")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/fetched-path.txt" "${dart_imgui_SOURCE_DIR}")\n'
        "add_executable(archive-probe main.cpp)\n"
        "target_link_libraries(archive-probe PRIVATE dart-gui)\n"
    )
    build = tmp_path / "build"
    for version in (1, 2):
        payload = tmp_path / f"payload-{version}"
        payload.mkdir()
        for name in (
            "imconfig.h",
            "imgui.h",
            "imgui_internal.h",
            "imstb_rectpack.h",
            "imstb_textedit.h",
            "imstb_truetype.h",
            "imgui_draw.cpp",
            "imgui_tables.cpp",
            "imgui_widgets.cpp",
        ):
            (payload / name).write_text("// Fixture.\n")
        (payload / "imgui.cpp").write_text(
            f"int archive_version() {{ return {version}; }}\n"
        )
        archive = tmp_path / f"imgui-{version}.tar.gz"
        with tarfile.open(archive, "w:gz") as output:
            output.add(payload, arcname="imgui")
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        (source / "imgui.cmake").write_text(
            module.replace(original_hash, digest).replace(url, archive.as_posix())
        )
        # Let pytest report complete CMake output on failure.
        subprocess.run(
            ["cmake", "-G", "Ninja", "-S", str(source), "-B", str(build)],
            check=True,
            text=True,
        )
        # Reproduce the extraction timestamps used by CMake 3.22/3.23 even
        # when this regression runs with a newer CMake.
        extracted = Path((build / "fetched-path.txt").read_text())
        for path in extracted.iterdir():
            os.utime(path, (1_000_000_000, 1_000_000_000))
        subprocess.run(
            ["cmake", "--build", str(build)],
            check=True,
            text=True,
        )
        executable = build / (
            "archive-probe.exe" if os.name == "nt" else "archive-probe"
        )
        assert subprocess.run([str(executable)]).returncode == version


@pytest.mark.parametrize(
    "package,finder,component,dependency,version",
    [
        ("ODE", "ODE", "dart-collision-ode", "ode", "0.16.2"),
        ("BULLET", "Bullet", "dart-collision-bullet", "Bullet", "3.05"),
    ],
)
@pytest.mark.parametrize("require,skip", [(False, False), (True, False), (True, True)])
@pytest.mark.parametrize("versioned", [True, False])
def test_require_optional_dependencies(
    tmp_path, package, finder, component, dependency, version, require, skip, versioned
):
    version_arg = f' "{version}"' if versioned else ""
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        f"set(DART_USE_SYSTEM_{package} ON)\n"
        f"include(DARTFind{finder})\n"
        f'dart_check_optional_package({package} "{component}" "{dependency}"{version_arg})\n',
        cmake_args=(
            f"-DDART_REQUIRE_OPTIONAL_DEPENDENCIES={'ON' if require else 'OFF'}",
            f"-DDART_SKIP_{package}={'ON' if skip else 'OFF'}",
        ),
    )
    output = " ".join((result.stdout + result.stderr).split())
    assert "NOT found" in output
    assert (f"(>= {version})" in output) == versioned
    if require and not skip:
        assert result.returncode != 0
        for text in (
            "DART_REQUIRE_OPTIONAL_DEPENDENCIES",
            f"DART_SKIP_{package}=ON",
            component,
            dependency,
        ):
            assert text in output
    else:
        assert result.returncode == 0, output


def write_nanobind_package(package, version, internals=22):
    """Write a fake nanobind package whose header decides the manual checks."""
    (package / "include/nanobind").mkdir(parents=True)
    (package / "src").mkdir()
    (package / "include/nanobind/nanobind.h").write_text(
        "".join(
            f"#define NB_VERSION_{part} {value}\n"
            for part, value in zip(("MAJOR", "MINOR", "PATCH"), version.split("."))
        )
    )
    (package / "src/nb_internals.h").write_text(
        f"#  define NB_INTERNALS_VERSION {internals}\n"
    )
    (package / "nanobindConfig.cmake").write_text(
        f'set(NB_DIR "{package.as_posix()}")\n'
    )
    # Always compatible, so CMake's 3.1 request never preempts a manual check.
    (package / "nanobindConfigVersion.cmake").write_text(
        f'set(PACKAGE_VERSION "{version}")\n' "set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
    )


@pytest.mark.parametrize("system", [True, False])
@pytest.mark.parametrize(
    "version,internals,missing,accepted",
    [
        ("3.0.2", 21, None, False),
        ("3.1.0", 22, None, True),
        ("3.1.9", 22, None, True),
        ("3.2.0", 22, None, False),
        ("4.0.0", 23, None, False),
        ("3.1.0", 23, None, False),
        ("3.1.0", 220, None, False),
        ("3.1.0", "22\r", None, True),
        ("3.1.0", "22 // ABI version", None, True),
        ("3.1.0", 22, "NB_DIR", False),
        ("3.1.0", 22, "include/nanobind/nanobind.h", False),
        ("3.1.0", 22, "src/nb_internals.h", False),
    ],
)
def test_nanobind_version_and_internals(
    tmp_path, system, version, internals, missing, accepted
):
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "FindPython.cmake").write_text("set(Python_FOUND TRUE)\n")
    package = tmp_path / "nanobind"
    write_nanobind_package(package, version, internals)
    (package / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\n"
        f'set(NB_DIR "{package.as_posix()}" PARENT_SCOPE)\n'
    )
    if missing == "NB_DIR":
        (package / "nanobindConfig.cmake").write_text("")
        (package / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.22.1)\n"
        )
    elif missing:
        (package / missing).unlink()
    for name in ("dartpy", "tests", "examples", "tutorials"):
        directory = tmp_path / "source" / name
        directory.mkdir(parents=True)
        (directory / "CMakeLists.txt").write_text(
            'file(WRITE "${CMAKE_BINARY_DIR}/dartpy-reached" "")\n'
            if name == "dartpy"
            else ""
        )
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        "set(DART_BUILD_DARTPY ON)\n"
        f"set(DART_USE_SYSTEM_NANOBIND {'ON' if system else 'OFF'})\n"
        f'set(nanobind_DIR "{package.as_posix()}")\n'
        f'set(FETCHCONTENT_SOURCE_DIR_NANOBIND "{package.as_posix()}")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n',
    )
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == accepted, output
    assert (tmp_path / "build/dartpy-reached").exists() == accepted
    if not accepted:
        if missing:
            assert "dartpy requires a complete nanobind 3.1.x package" in output
            assert "NB_DIR must contain" in output
            assert "file STRINGS" not in output
        elif version == "3.1.0":
            assert "NB_INTERNALS_VERSION=22" in output
        else:
            assert "dartpy requires nanobind 3.1.x" in output
            assert f"found {version}" in output


def test_nanobind_search_skips_old_package_before_running_its_config(
    tmp_path, monkeypatch
):
    old = tmp_path / "prefix/lib/cmake/nanobind"
    old.mkdir(parents=True)
    (old / "nanobindConfig.cmake").write_text(
        'message(FATAL_ERROR "old nanobind config executed")\n'
    )
    (old / "nanobindConfigVersion.cmake").write_text(
        'set(PACKAGE_VERSION "3.0.2")\n'
        "if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)\n"
        "  set(PACKAGE_VERSION_COMPATIBLE FALSE)\n"
        "else()\n"
        "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
        "endif()\n"
    )
    hinted = tmp_path / "hinted"
    write_nanobind_package(hinted, "3.1.4")
    (hinted / "nanobindConfig.cmake").write_text(
        f'set(NB_DIR "{hinted.as_posix()}")\nset(hinted_config_ran TRUE)\n'
    )
    # Stands in for the nanobind package whose `-m nanobind --cmake_dir` is the hint.
    stub = tmp_path / "pythonpath/nanobind"
    stub.mkdir(parents=True)
    (stub / "__init__.py").write_text("")
    (stub / "__main__.py").write_text(f"print({hinted.as_posix()!r})\n")
    monkeypatch.setenv("PYTHONPATH", str(stub.parent))
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "FindPython.cmake").write_text("set(Python_FOUND TRUE)\n")
    for name in ("dartpy", "tests", "examples", "tutorials"):
        directory = tmp_path / "source" / name
        directory.mkdir(parents=True)
        (directory / "CMakeLists.txt").write_text(
            'file(WRITE "${CMAKE_BINARY_DIR}/dartpy-reached" "")\n'
            if name == "dartpy"
            else ""
        )
    result = configure(
        tmp_path,
        "set(DART_BUILD_DARTPY ON)\n"
        "set(DART_USE_SYSTEM_NANOBIND ON)\n"
        f'set(Python_EXECUTABLE "{Path(sys.executable).as_posix()}")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n'
        "if(NOT hinted_config_ran)\n"
        '  message(FATAL_ERROR "compatible hinted nanobind was not selected")\n'
        "endif()\n",
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "old nanobind config executed" not in output
    assert (tmp_path / "build/dartpy-reached").exists()
