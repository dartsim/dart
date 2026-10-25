"""Exercise DART's installed dependency finders with isolated CMake packages."""

import hashlib
import os
import re
import shlex
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
    # macOS always defaults GLFW to the system one (tested separately).
    content = "include(dart_defs)\nset(APPLE OFF)\n" + options
    switches = ("GOOGLETEST", "GOOGLEBENCHMARK", "TRACY", "GLFW")
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


@pytest.mark.parametrize("apple,expected", [("ON", "ON"), ("OFF", "OFF")])
def test_macos_glfw_defaults_to_the_system_one(tmp_path, apple, expected):
    source = (ROOT / "CMakeLists.txt").read_text()
    options = source[
        source.index("dart_option(\n  DART_USE_SYSTEM_DEPENDENCIES") : source.index(
            "dart_option(\n  DART_ALLOW_SINGLE_PRECISION_LIBCCD"
        )
    ]
    result = configure(
        tmp_path,
        f"include(dart_defs)\nset(APPLE {apple})\n" + options,
        cmake_args=["-DDART_USE_SYSTEM_DEPENDENCIES=OFF"],
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cache = (tmp_path / "build" / "CMakeCache.txt").read_text()
    assert f"DART_USE_SYSTEM_GLFW:BOOL={expected}" in cache


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

TRACY_FETCH_FIXTURE = TRACY_FIXTURE.replace(
    "set(DART_USE_SYSTEM_TRACY ON)",
    "set(DART_USE_SYSTEM_TRACY OFF)\ninclude(DARTMacros)",
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


TRACY_ENABLED = (
    "set_target_properties(Tracy::TracyClient PROPERTIES "
    'INTERFACE_COMPILE_DEFINITIONS "TRACY_ENABLE;TRACY_IMPORTS")\n'
)


@pytest.mark.parametrize(
    "version,accepted",
    [
        (None, False),
        ("0.11.1", False),
        ("0.13.1", False),
        ("0.14.0", False),
        ("0.14.1", True),
        ("0.14.2", True),
    ],
)
def test_system_tracy_minimum_version(tmp_path, version, accepted):
    # Tracy 0.14.1 is the first release whose package has a version file.
    config_package(
        tmp_path, "Tracy", version, "Tracy::TracyClient", extra=TRACY_ENABLED
    )
    result = configure(tmp_path, TRACY_FIXTURE)
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == accepted, output
    if not accepted:
        assert "0.14.1" in output


@pytest.mark.parametrize(
    "definitions,accepted",
    [
        ("TRACY_ENABLE;TRACY_IMPORTS", True),
        ("TRACY_IMPORTS", False),
        (None, False),
    ],
)
def test_system_tracy_requires_tracy_enable(tmp_path, definitions, accepted):
    # Tracy 0.14 builds an empty client library unless TRACY_ENABLE is set.
    extra = ""
    if definitions is not None:
        extra = (
            "set_target_properties(Tracy::TracyClient PROPERTIES "
            f'INTERFACE_COMPILE_DEFINITIONS "{definitions}")\n'
        )
    config_package(tmp_path, "Tracy", "0.14.1", "Tracy::TracyClient", extra=extra)
    result = configure(tmp_path, TRACY_FIXTURE)
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == accepted, output
    if not accepted:
        assert "TRACY_ENABLE" in output and "DART_USE_SYSTEM_TRACY=OFF" in output


def fake_tracy_source(tmp_path):
    source = tmp_path / "tracy-src"
    source.mkdir()
    (source / "client.cpp").write_text("int tracyClient() { return 0; }\n")
    (source / "CMakeLists.txt").write_text(
        # Tracy 0.14 turns TRACY_ENABLE off by default.
        'option(TRACY_ENABLE "Enable profiling" OFF)\n'
        'if(NOT TRACY_ENABLE)\n  message(FATAL_ERROR "TRACY_ENABLE is OFF")\nendif()\n'
        "add_library(TracyClient STATIC client.cpp)\n"
        "add_library(Tracy::TracyClient ALIAS TracyClient)\n"
    )
    return source


@pytest.mark.parametrize("seeded", [True, False])
@pytest.mark.parametrize("requested", [None, "OFF"])
def test_fetched_tracy_is_enabled_and_checked_offline(tmp_path, seeded, requested):
    source = fake_tracy_source(tmp_path) if seeded else tmp_path / "tracy-src"
    args = [
        f"-DFETCHCONTENT_SOURCE_DIR_TRACY={source.as_posix()}" if seeded else "",
        "-DFETCHCONTENT_FULLY_DISCONNECTED=ON",
    ]
    if requested is not None:
        args.append(f"-DTRACY_ENABLE={requested}")
    result = configure(
        tmp_path,
        TRACY_FETCH_FIXTURE,
        cmake_args=[arg for arg in args if arg],
        language="CXX",
    )
    output = result.stdout + result.stderr
    if seeded:
        assert result.returncode == 0, output
    else:
        assert result.returncode != 0
        assert "Cannot fetch tracy" in output and "DART_USE_SYSTEM_TRACY=ON" in output


def test_fetched_tracy_leaves_an_embedding_project_alone(tmp_path):
    source = fake_tracy_source(tmp_path)
    dart = tmp_path / "source" / "dart"
    dart.mkdir(parents=True)
    (dart / "CMakeLists.txt").write_text(TRACY_FETCH_FIXTURE)
    result = configure(
        tmp_path,
        'set(TRACY_ENABLE OFF CACHE BOOL "The parent project\'s setting")\n'
        "set(TRACY_ENABLE OFF)\n"
        "add_subdirectory(dart)\n"
        'if(TRACY_ENABLE)\n  message(FATAL_ERROR "TRACY_ENABLE changed")\nendif()\n',
        cmake_args=[
            f"-DFETCHCONTENT_SOURCE_DIR_TRACY={source.as_posix()}",
            "-DFETCHCONTENT_FULLY_DISCONNECTED=ON",
        ],
        language="CXX",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cache = (tmp_path / "build" / "CMakeCache.txt").read_text()
    assert "TRACY_ENABLE:BOOL=OFF" in cache


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


@pytest.mark.parametrize("missing", ["tinyxml2", "urdfdom"])
def test_io_requires_xml_parser_dependencies(tmp_path, missing):
    if missing != "tinyxml2":
        config_package(tmp_path, "tinyxml2", "10.0.0", "tinyxml2::tinyxml2")
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        f"set(CMAKE_DISABLE_FIND_PACKAGE_{missing} TRUE)\n"
        f'include("{(ROOT / "dart/utils/CMakeLists.txt").as_posix()}")\n',
    )
    assert result.returncode != 0
    assert f"dart-io requires {missing}" in result.stdout + result.stderr
    assert "DART_BUILD_IO=OFF" in result.stdout + result.stderr


@pytest.mark.parametrize("found", [False, True])
@pytest.mark.parametrize("skip", [False, True])
def test_sdf_optional_dependency(tmp_path, found, skip):
    discovery = (ROOT / "cmake/DARTFindDependencies.cmake").read_text()
    start = discovery.index("if(NOT DART_BUILD_IO)")
    discovery = discovery[
        start : discovery.index("\nendif()", start) + len("\nendif()")
    ]
    assert "dart_find_package(sdformat)" in discovery
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        "set(DART_BUILD_IO ON)\n"
        f"set(DART_SKIP_SDFORMAT {'ON' if skip else 'OFF'})\n"
        "function(dart_find_package)\n"
        f"set(SDFORMAT_FOUND {'TRUE' if found else 'FALSE'} PARENT_SCOPE)\n"
        "endfunction()\n" + discovery + "\n"
        "function(dart_add_utils_headers)\nendfunction()\n"
        "function(dart_add_utils_sources)\nendfunction()\n"
        f'set(DART_SOURCE_DIR "{ROOT.as_posix()}")\n'
        "set(CONFIG_INSTALL_DIR lib/cmake/DART)\n"
        f'include("{(ROOT / "dart/utils/sdf/CMakeLists.txt").as_posix()}")\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cache = (tmp_path / "build" / "CMakeCache.txt").read_text()
    enabled = found and not skip
    assert f"HAVE_SDFORMAT:BOOL={'TRUE' if enabled else 'FALSE'}" in cache
    if found and skip:
        assert "please install sdformat" not in result.stdout + result.stderr
    if not found:
        assert "Looking for sdformat - NOT found" in result.stderr


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


# CMake 3.28's FindOpenSSL calls pkg_check_modules() even when configure()
# disables FindPkgConfig; a no-op keeps host pkg-config packages out.
NO_PKG_CONFIG = (
    "if(NOT COMMAND pkg_check_modules)\n"
    "  macro(pkg_check_modules)\n"
    "  endmacro()\n"
    "endif()\n"
)


def fake_openssl(tmp_path, version):
    """Lay out the files CMake's FindOpenSSL probes and return its root option."""
    root = tmp_path / "openssl"
    (root / "include/openssl").mkdir(parents=True)
    (root / "lib").mkdir()
    (root / "include/openssl/ssl.h").touch()
    (root / "lib/libcrypto.so").touch()
    major, minor, fix = map(int, version.split("."))
    # 1.1.x packs the version into OPENSSL_VERSION_NUMBER as 0xMNNFFPPS; 3.x
    # headers define OPENSSL_VERSION_STR, which FindOpenSSL reads instead.
    (root / "include/openssl/opensslv.h").write_text(
        f"#define OPENSSL_VERSION_NUMBER 0x{major}{minor:02x}{fix:02x}00fL\n"
        if major == 1
        else f'#define OPENSSL_VERSION_STR "{version}"\n'
    )
    return (f"-DOPENSSL_ROOT_DIR={root.as_posix()}",)


@pytest.mark.skipif(
    sys.platform != "linux", reason="the fake OpenSSL is a Linux shared library"
)
@pytest.mark.parametrize(
    "version,accepted",
    [
        ("1.1.1", False),
        ("3.0.2", False),
        ("3.0.12", False),
        ("3.0.13", True),
        ("3.5.5", True),
    ],
)
def test_openssl_floor(tmp_path, version, accepted):
    result = configure(
        tmp_path,
        NO_PKG_CONFIG + "include(DARTFindOpenSSL)\n"
        "if(OPENSSL_FOUND AND TARGET OpenSSL::Crypto)\n"
        + ("" if accepted else 'message(FATAL_ERROR "Accepted unsupported version")\n')
        + "else()\n"
        + ('message(FATAL_ERROR "Rejected supported version")\n' if accepted else "")
        + "endif()\n",
        cmake_args=fake_openssl(tmp_path, version),
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(
    sys.platform != "linux", reason="the fake OpenSSL is a Linux shared library"
)
@pytest.mark.parametrize("version", ["1.1.1", "3.0.12"])
def test_assets_reject_old_openssl_with_minimum_version_message(tmp_path, version):
    modules = tmp_path / "modules"
    modules.mkdir()
    # libcurl is not under test; let the OpenSSL check run.
    (modules / "DARTFindCURL.cmake").write_text(
        "set(CURL_FOUND TRUE)\nadd_library(CURL::libcurl INTERFACE IMPORTED)\n"
    )
    result = configure(
        tmp_path,
        NO_PKG_CONFIG + "include(DARTMacros)\n"
        f'include("{(ROOT / "dart/utils/assets/CMakeLists.txt").as_posix()}")\n',
        cmake_args=fake_openssl(tmp_path, version),
    )
    assert result.returncode != 0
    message = "dart-utils-assets requires OpenSSL >= 3.0.13 (Crypto)"
    assert message in result.stdout + result.stderr


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
        (src / f"{name}.cpp").write_text(
            '#include "LinearMath/btScalar.h"\n'
            f'extern "C" int fixture_{name}(void) {{ return sizeof(btScalar) == sizeof(double); }}\n'
        )
    (src / "LinearMath/btScalar.h").write_text(
        "#pragma once\n#define BT_BULLET_VERSION 325\n"
        "#ifdef BT_USE_DOUBLE_PRECISION\ntypedef double btScalar;\n"
        "#else\ntypedef float btScalar;\n#endif\n"
        '#ifdef __cplusplus\nextern "C" {\n#endif\n'
        "int fixture_LinearMath(void);\n"
        "#ifdef __cplusplus\n}\n#endif\n"
    )
    (src / "btBulletCollisionCommon.h").write_text(
        "#include <LinearMath/btScalar.h>\n"
        '#ifdef __cplusplus\nextern "C" {\n#endif\n'
        "int fixture_BulletCollision(void);\n"
        "#ifdef __cplusplus\n}\n#endif\n"
    )
    (src / "btBulletDynamicsCommon.h").write_text("/* Not part of the SDK. */\n")
    (src / "CMakeLists.txt").write_text(
        "foreach(name BulletCollision LinearMath BulletDynamics BulletSoftBody)\n"
        'add_library(${name} SHARED "${name}.cpp")\n'
        'target_include_directories(${name} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")\n'
        "if(USE_DOUBLE_PRECISION)\n"
        "target_compile_definitions(${name} PUBLIC BT_USE_DOUBLE_PRECISION)\nendif()\n"
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
        "Cflags: @BULLET_DOUBLE_DEF@ -I${includedir} -I${prefix}/include\n"
    )
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\nproject(BulletFixture C CXX)\n"
        'option(USE_DOUBLE_PRECISION "Use double Bullet scalars" OFF)\n'
        'set(BULLET_DOUBLE_DEF "")\n'
        'if(USE_DOUBLE_PRECISION)\nset(BULLET_DOUBLE_DEF "-DBT_USE_DOUBLE_PRECISION")\nendif()\n'
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
    bullet_fixture_run(
        CMAKE, "-S", source, "-B", sdk, "-G", "Ninja", "-DUSE_DOUBLE_PRECISION=ON"
    )
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
        f'add_library({target} SHARED "{source.as_posix()}/src/${{name}}.cpp")\n'
        f"set_target_properties({target} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)\n"
        f'target_include_directories({target} PUBLIC "{source.as_posix()}/src")\n'
        f"target_compile_definitions({target} PUBLIC BT_USE_DOUBLE_PRECISION)\n"
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
        language="C CXX",
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
    scalar = source / "src/LinearMath/btScalar.h"
    scalar.write_text(
        scalar.read_text().replace(
            "#define BT_BULLET_VERSION 325\n",
            f"#define BT_BULLET_VERSION {version}\n" if version else "",
        )
    )
    sdk = tmp_path / "provider-build"
    sdk.mkdir()
    original_pc = b"Name: provider-owned\nLibs: -lBulletCollision -lLinearMath\n"
    if mode == "imported":
        bullet_fixture_run(
            CMAKE, "-S", source, "-B", sdk, "-G", "Ninja", "-DUSE_DOUBLE_PRECISION=ON"
        )
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
                f'add_library({target} STATIC "{source.as_posix()}/src/{name}.cpp")\n'
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
        language="C" if mode == "imported" else "C CXX",
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
                    f'add_library({name} STATIC "{source.as_posix()}/src/{name}.cpp")\n'
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
        language="C CXX",
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
    "version,settings,installed,expected",
    [
        # Building DART: DART_BULLET_FLOAT64 prefers the -float64 libraries.
        (325, "set(DART_BULLET_FLOAT64 ON)", True, "float64"),
        (325, "set(DART_BULLET_FLOAT64 OFF)", True, "single"),
        # Double-precision float heightfields need Bullet 3.17.
        (305, "set(DART_BULLET_FLOAT64 ON)", True, "single"),
        (325, "set(DART_BULLET_FLOAT64 ON)", False, "single"),
        # Installed DART: the recorded selection decides, without fallback.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\nset(DART_BULLET_REQUIRE_VERSION 3.17)",
            True,
            "float64",
        ),
        (325, "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)", False, "missing"),
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 FALSE)\nset(DART_BULLET_FLOAT64 ON)",
            True,
            "single",
        ),
        # DART built against Bullet 3.17 calls its float heightfield constructor.
        (
            305,
            "set(DART_BULLET_REQUIRE_FLOAT64 FALSE)\nset(DART_BULLET_REQUIRE_VERSION 3.17)",
            True,
            "missing",
        ),
        # Bundled Bullet ignores a cached system library.
        (
            325,
            "set(DART_USE_SYSTEM_BULLET OFF)\nset(DART_BULLET_FLOAT64 ON)\n"
            "add_library(BulletCollision INTERFACE)\n"
            "add_library(LinearMath INTERFACE)\n"
            'set(DART_BULLET_SOURCE_DIR "${lib}/..")\n'
            'set(BULLET_COLLISION_LIBRARY "${collision}")',
            True,
            "bundled",
        ),
        # An installed DART also checks Bullet that the consumer found first.
        (
            325,
            "unset(DART_USE_SYSTEM_BULLET)\nfind_package(Bullet MODULE)\n"
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)",
            True,
            "float64",
        ),
        # An installed DART rejects an existing target whose library files show
        # the other variant.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            'set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES "${collision};${math}")',
            True,
            "rejected",
        ),
        # ... and accepts one of the same variant silently.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")',
            True,
            "accepted",
        ),
        # ... including per-configuration files, as DART's own target has.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "target_link_libraries(Bullet INTERFACE\n"
            '  optimized "${lib}/${prefix}BulletCollision-float64${ext}"\n'
            '  debug "${lib}/${prefix}BulletCollision-float64_d${ext}")',
            True,
            "accepted",
        ),
        # ... including the files of the imported targets it links.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            "set_target_properties(collision64 PROPERTIES IMPORTED_LOCATION\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES collision64)",
            True,
            "accepted",
        ),
        # A target that mixes the variants is rejected.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext};${math}")',
            True,
            "rejected",
        ),
        # A target without library files can only be warned about.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\nadd_library(Bullet INTERFACE IMPORTED)",
            True,
            "warned",
        ),
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 FALSE)\nadd_library(Bullet INTERFACE IMPORTED)",
            True,
            "warned",
        ),
        # The -float64 variant links only the collision and math libraries.
        (
            325,
            "set(DART_BULLET_FLOAT64 ON)\n"
            "foreach(stem BulletDynamics-float64 BulletSoftBody-float64)\n"
            '  file(TOUCH "${lib}/${prefix}${stem}${ext}")\n'
            "endforeach()",
            True,
            "float64",
        ),
        # Only library file names, not directories, mark the variant.
        (
            325,
            "set(DART_BULLET_FLOAT64 OFF)\n"
            'file(MAKE_DIRECTORY "${lib}/prefix-float64")\n'
            'set(collision "${lib}/prefix-float64/${prefix}BulletCollision${ext}")\n'
            'set(math "${lib}/prefix-float64/${prefix}LinearMath${ext}")\n'
            'file(TOUCH "${collision}" "${math}")',
            True,
            "unrecorded",
        ),
        # An installed single-precision DART refuses -float64 libraries.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 FALSE)\n"
            'set(collision "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            'set(math "${lib}/${prefix}LinearMath-float64${ext}")',
            True,
            "missing",
        ),
        # Debug variants: link matching -float64 ones, or keep FindBullet's
        # per-configuration single-precision selection without them.
        (
            325,
            "set(DART_BULLET_FLOAT64 ON)\n"
            'set(collision_debug "${lib}/${prefix}BulletCollision_d${ext}")\n'
            'set(math_debug "${lib}/${prefix}LinearMath_d${ext}")\n'
            "foreach(stem BulletCollision-float64_d LinearMath-float64_d)\n"
            '  file(TOUCH "${lib}/${prefix}${stem}${ext}")\n'
            "endforeach()",
            True,
            "float64-debug",
        ),
        (
            325,
            "set(DART_BULLET_FLOAT64 ON)\n"
            'set(collision_debug "${lib}/${prefix}BulletCollision_d${ext}")\n'
            'set(math_debug "${lib}/${prefix}LinearMath_d${ext}")',
            True,
            "single-debug",
        ),
        # Multi-config layouts keep Debug libraries in their own directory.
        (
            325,
            "set(DART_BULLET_FLOAT64 ON)\n"
            'set(collision_debug "${lib}/debug/${prefix}BulletCollision_d${ext}")\n'
            'set(math_debug "${lib}/debug/${prefix}LinearMath_d${ext}")\n'
            'file(MAKE_DIRECTORY "${lib}/debug")\n'
            "foreach(stem BulletCollision_d LinearMath_d BulletCollision-float64_d\n"
            "    LinearMath-float64_d)\n"
            '  file(TOUCH "${lib}/debug/${prefix}${stem}${ext}")\n'
            "endforeach()",
            True,
            "float64-debug",
        ),
        # pkg-config flags skip FindBullet's Debug libraries, whatever their
        # names.
        (
            325,
            "set(DART_BULLET_FLOAT64 OFF)\n"
            'set(collision_debug "${lib}/${prefix}BulletCollisionDbg${ext}")\n'
            'set(math_debug "${lib}/${prefix}LinearMathDbg${ext}")',
            True,
            "pc-release",
        ),
        # Explicit libraries that mix the variants are rejected, including
        # FindBullet's older BulletMath name for LinearMath.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            'set(collision "${lib}/${prefix}BulletCollision-float64${ext}")',
            False,
            "rejected",
        ),
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 FALSE)\n"
            'set(collision "${lib}/${prefix}BulletCollision-float64${ext}")',
            True,
            "rejected",
        ),
        (
            325,
            "set(DART_BULLET_FLOAT64 OFF)\n"
            'set(collision "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            'file(TOUCH "${collision}")',
            False,
            "rejected",
        ),
        (
            325,
            "set(DART_BULLET_FLOAT64 OFF)\n"
            'set(collision "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            'set(math "${lib}/${prefix}BulletMath${ext}")\n'
            'file(TOUCH "${collision}" "${math}")',
            False,
            "rejected",
        ),
        # Explicitly selected -float64 libraries are recorded as such.
        (
            325,
            "set(DART_BULLET_FLOAT64 OFF)\n"
            'set(collision "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            'set(math "${lib}/${prefix}LinearMath-float64${ext}")',
            True,
            "recorded",
        ),
        # Building DART records the variant of a parent's Bullet target.
        (
            325,
            "add_library(parent_bullet INTERFACE IMPORTED)\n"
            "set_target_properties(parent_bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(Bullet ALIAS parent_bullet)",
            True,
            "recorded",
        ),
        # ... including per-configuration imported targets that it links,
        # with or without a listed configuration, and namespaced targets in
        # generator expressions.
        (
            325,
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            "set_target_properties(collision64 PROPERTIES IMPORTED_CONFIGURATIONS CUSTOM\n"
            '  IMPORTED_LOCATION_CUSTOM "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(parent_bullet INTERFACE IMPORTED)\n"
            "set_target_properties(parent_bullet PROPERTIES INTERFACE_LINK_LIBRARIES collision64)\n"
            "add_library(Bullet ALIAS parent_bullet)",
            True,
            "recorded",
        ),
        # An unlisted Release-only location is valid only when Release is selected.
        (
            325,
            "set(CMAKE_BUILD_TYPE Debug)\n"
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            "set_target_properties(collision64 PROPERTIES IMPORTED_LOCATION_RELEASE\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES collision64)",
            True,
            "rejected",
        ),
        (
            325,
            "set(CMAKE_BUILD_TYPE Release)\n"
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            "set_target_properties(collision64 PROPERTIES IMPORTED_LOCATION_RELEASE\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES collision64)",
            True,
            "recorded",
        ),
        (
            325,
            "add_library(Bullet::BulletCollision UNKNOWN IMPORTED)\n"
            "set_target_properties(Bullet::BulletCollision PROPERTIES IMPORTED_LOCATION\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "$<LINK_ONLY:Bullet::BulletCollision>")',
            True,
            "recorded",
        ),
        # dart.pc can name the release files and include directories of a
        # parent's -float64 target.
        (
            325,
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            'set_target_properties(collision64 PROPERTIES IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"\n'
            '  IMPORTED_LOCATION_RELEASE "${lib}/${prefix}BulletCollision-float64${ext}"\n'
            '  IMPORTED_LOCATION_DEBUG "${lib}/${prefix}BulletCollision-float64-dbg${ext}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES collision64\n"
            '  INTERFACE_INCLUDE_DIRECTORIES "${lib}/include;$<BUILD_INTERFACE:${lib}/build>")',
            True,
            "recorded-pc",
        ),
        # ... and the build type's locations, Debug here.
        (
            325,
            "set(CMAKE_BUILD_TYPE Debug)\n"
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            'set_target_properties(collision64 PROPERTIES IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"\n'
            '  IMPORTED_LOCATION_RELEASE "${lib}/${prefix}BulletCollision-float64${ext}"\n'
            '  IMPORTED_LOCATION_DEBUG "${lib}/${prefix}BulletCollision-float64-dbg${ext}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES collision64)",
            True,
            "recorded-pc-debug",
        ),
        # ... skipping entries for other configurations.
        (
            325,
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES\n"
            '  INTERFACE_INCLUDE_DIRECTORIES "${lib}/include"\n'
            '  INTERFACE_LINK_LIBRARIES "$<$<CONFIG:Debug>:${lib}/${prefix}BulletCollision-float64-dbg${ext};'
            "${lib}/${prefix}LinearMath-float64-dbg${ext}>;"
            '$<$<NOT:$<CONFIG:Debug>>:${lib}/${prefix}BulletCollision-float64${ext}>")',
            True,
            "recorded-pc",
        ),
        (
            325,
            "set(CMAKE_BUILD_TYPE Debug)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES\n"
            '  INTERFACE_LINK_LIBRARIES "$<$<CONFIG:Debug>:${lib}/${prefix}BulletCollision-float64-dbg${ext};'
            "${lib}/${prefix}LinearMath-float64-dbg${ext}>;"
            '$<$<NOT:$<CONFIG:Debug>>:${lib}/${prefix}BulletCollision-float64${ext}>")',
            True,
            "recorded-pc-debug-list",
        ),
        # Entries that are empty in this build, or under conditions that it
        # does not evaluate, are skipped.
        (
            325,
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES\n"
            '  INTERFACE_INCLUDE_DIRECTORIES "${lib}/include"\n'
            '  INTERFACE_LINK_LIBRARIES "$<BUILD_INTERFACE:${lib}/${prefix}BulletCollision-float64${ext}>;'
            "$<INSTALL_INTERFACE:${collision}>;$<0:${math}>;$<$<BOOL:OFF>:${math}>;"
            '$<$<PLATFORM_ID:Nowhere>:${math}>")',
            True,
            "recorded-pc",
        ),
        # ... while a true $<BOOL:...> condition applies.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "$<$<BOOL:ON>:${math}>")',
            True,
            "rejected",
        ),
        # CMake links the import library of a Windows shared library.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(collision64 SHARED IMPORTED)\n"
            "set_target_properties(collision64 PROPERTIES\n"
            '  IMPORTED_IMPLIB "${lib}/BulletCollision-float64.lib"\n'
            '  IMPORTED_LOCATION "${lib}/BulletCollision.dll")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES collision64)",
            True,
            "accepted",
        ),
        # The compile-only probe must use inherited definitions when this
        # project builds the C++ Bullet library itself.
        (
            325,
            'file(WRITE "${CMAKE_BINARY_DIR}/bt/dummy.cpp" '
            '"#include <LinearMath/btScalar.h>\\n'
            'static_assert(sizeof(btScalar) == sizeof(double));\\nint bt_dummy;\\n")\n'
            'file(WRITE "${CMAKE_BINARY_DIR}/bt/CMakeLists.txt"\n'
            '  "add_definitions(-DBT_USE_DOUBLE_PRECISION)\\n'
            "add_library(BulletCollision STATIC dummy.cpp)\\n"
            'add_library(helper STATIC dummy.cpp)\\n")\n'
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\n"
            "enable_language(CXX)\n"
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)\n"
            'add_subdirectory("${CMAKE_BINARY_DIR}/bt" bt-build)\n'
            "foreach(t BulletCollision helper)\n"
            'target_include_directories(${t} PRIVATE "${lib}/../src")\n'
            "endforeach()\n"
            "add_library(Bullet INTERFACE)\n"
            "target_link_libraries(Bullet INTERFACE helper BulletCollision)",
            True,
            "built-double",
        ),
        (
            325,
            'file(WRITE "${CMAKE_BINARY_DIR}/bt/dummy.c" "int bt_dummy;\\n")\n'
            'file(WRITE "${CMAKE_BINARY_DIR}/bt/CMakeLists.txt"\n'
            '  "add_library(helper STATIC dummy.c)\\n")\n'
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\n"
            "enable_language(C)\n"
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)\n"
            'add_subdirectory("${CMAKE_BINARY_DIR}/bt" bt-build)\n'
            "add_library(Bullet INTERFACE)\n"
            "target_link_libraries(Bullet INTERFACE helper\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")',
            True,
            "built-none",
        ),
        # Building DART also rejects a target that mixes the variants, which
        # the files of indirectly linked targets reveal; link cycles end.
        (
            325,
            "add_library(math32 UNKNOWN IMPORTED)\n"
            'set_target_properties(math32 PROPERTIES IMPORTED_LOCATION "${math}")\n'
            "add_library(wrapper INTERFACE IMPORTED)\n"
            'set_target_properties(wrapper PROPERTIES INTERFACE_LINK_LIBRARIES "math32;Bullet")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext};wrapper")',
            True,
            "rejected",
        ),
        # ... including targets listed inside one generator expression.
        (
            325,
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            "set_target_properties(collision64 PROPERTIES IMPORTED_LOCATION\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(math32 UNKNOWN IMPORTED)\n"
            'set_target_properties(math32 PROPERTIES IMPORTED_LOCATION "${math}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "$<BUILD_INTERFACE:collision64;math32>")',
            True,
            "rejected",
        ),
        (
            325,
            "add_library(collision64 UNKNOWN IMPORTED)\n"
            "set_target_properties(collision64 PROPERTIES IMPORTED_LOCATION\n"
            '  "${lib}/${prefix}BulletCollision-float64${ext}")\n'
            "add_library(math32 UNKNOWN IMPORTED)\n"
            'set_target_properties(math32 PROPERTIES IMPORTED_LOCATION "${math}")\n'
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "$<1:collision64;math32>")',
            True,
            "rejected",
        ),
        # Unsupported active expressions cannot prove a required ABI.
        (
            325,
            "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            "set_target_properties(Bullet PROPERTIES INTERFACE_LINK_LIBRARIES\n"
            '  "$<$<AND:$<NOT:$<CONFIG:Debug>>,1>:collision64;math32>")',
            True,
            "rejected",
        ),
    ],
)
def test_bullet_prefers_float64_libraries(
    tmp_path, version, settings, installed, expected
):
    include = tmp_path / "bullet" / "src"
    header = include / "LinearMath" / "btScalar.h"
    header.parent.mkdir(parents=True)
    header.write_text(f"#define BT_BULLET_VERSION {version}\n")
    if expected == "built-double":
        header.write_text(
            header.read_text()
            + "#ifdef BT_USE_DOUBLE_PRECISION\nusing btScalar = double;\n"
            "#else\nusing btScalar = float;\n#endif\n"
        )
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "FindBullet.cmake").write_text(
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        f'set(BULLET_INCLUDE_DIRS "{include.as_posix()}")\n'
        f'set(BULLET_INCLUDE_DIR "{include.as_posix()}")\n'
        'set(BULLET_COLLISION_LIBRARY "${collision}")\n'
        'set(BULLET_MATH_LIBRARY "${math}")\n'
        'set(BULLET_LIBRARIES "${collision}" "${math}")\n'
        "if(DEFINED collision_debug)\n"
        '  set(BULLET_COLLISION_LIBRARY_DEBUG "${collision_debug}")\n'
        '  set(BULLET_MATH_LIBRARY_DEBUG "${math_debug}")\n'
        "  set(BULLET_LIBRARIES optimized ${collision} debug ${collision_debug}\n"
        "    optimized ${math} debug ${math_debug})\n"
        "endif()\n"
    )
    stems = "BulletCollision LinearMath"
    if installed:
        stems += " BulletCollision-float64 LinearMath-float64"
    lib = (tmp_path / "bullet" / "lib").as_posix()
    suffix = "-float64" if expected == "float64" else ""
    if expected == "bundled":
        # Bundled targets' precision and pkg-config flags are not recorded.
        check = (
            'NOT linked STREQUAL "BulletCollision;LinearMath"'
            ' OR NOT DART_BULLET_USES_FLOAT64 STREQUAL ""'
            ' OR NOT DART_BULLET_PC_LIBS STREQUAL ""'
        )
    elif expected == "missing":
        check = "TARGET Bullet OR BULLET_FOUND"
    elif expected == "warned":
        check = "NOT BULLET_FOUND"
    elif expected == "rejected":
        check = "BULLET_FOUND"
    elif expected == "float64-debug":
        # target_link_libraries stores optimized/debug as $<CONFIG> expressions.
        check = (
            'NOT "${linked}" MATCHES "CONFIG:DEBUG>:[^>]*BulletCollision-float64_d"'
            ' OR NOT "${linked}" MATCHES "CONFIG:DEBUG>:[^>]*LinearMath-float64_d"'
            ' OR "${linked}" MATCHES "BulletCollision_d"'
            " OR NOT DART_BULLET_USES_FLOAT64 STREQUAL TRUE"
            # pkg-config names the release libraries.
            ' OR NOT DART_BULLET_PC_LIBS MATCHES "BulletCollision-float64"'
            ' OR DART_BULLET_PC_LIBS MATCHES "float64_d"'
        )
    elif expected == "single-debug":
        check = (
            'NOT "${linked}" MATCHES "CONFIG:DEBUG>:[^>]*BulletCollision_d"'
            ' OR "${linked}" MATCHES "-float64"'
            " OR NOT DART_BULLET_USES_FLOAT64 STREQUAL FALSE"
        )
    elif expected == "pc-release":
        check = 'NOT DART_BULLET_PC_LIBS STREQUAL " ${collision} ${math}"'
    elif expected == "recorded-pc-debug":
        check = 'NOT DART_BULLET_PC_LIBS STREQUAL " ${lib}/${prefix}BulletCollision-float64-dbg${ext}"'
    elif expected == "recorded-pc-debug-list":
        check = (
            "NOT DART_BULLET_PC_LIBS STREQUAL"
            ' " ${lib}/${prefix}BulletCollision-float64-dbg${ext}'
            ' ${lib}/${prefix}LinearMath-float64-dbg${ext}"'
        )
    elif expected == "built-double":
        check = 'NOT DART_BULLET_BUILT_PRECISION STREQUAL "double"'
    elif expected == "built-none":
        check = 'NOT DART_BULLET_BUILT_PRECISION STREQUAL "" OR NOT DART_BULLET_USES_FLOAT64 STREQUAL TRUE'
    elif expected == "recorded-pc":
        expected_cflags = " -I${lib}/include"
        if "$<BUILD_INTERFACE:${lib}/build>" in settings:
            expected_cflags += " -I${lib}/build"
        check = (
            "NOT DART_BULLET_USES_FLOAT64 STREQUAL TRUE"
            ' OR NOT DART_BULLET_PC_LIBS STREQUAL " ${lib}/${prefix}BulletCollision-float64${ext}"'
            f' OR NOT DART_BULLET_PC_CFLAGS STREQUAL "{expected_cflags}"'
        )
    elif expected == "recorded":
        check = "NOT DART_BULLET_USES_FLOAT64 STREQUAL TRUE"
    elif expected == "accepted":
        check = "NOT BULLET_FOUND"
    elif expected == "unrecorded":
        check = "NOT DART_BULLET_USES_FLOAT64 STREQUAL FALSE"
    else:
        # FindBullet's results stay single precision for consumers' targets.
        check = (
            "NOT linked STREQUAL expected"
            " OR NOT BULLET_COLLISION_LIBRARY STREQUAL collision"
        )
        if "DART_BULLET_REQUIRE_FLOAT64" not in settings:
            recorded = "TRUE" if suffix else "FALSE"
            minimum = "3.17" if version >= 317 else "3.05"
            check += (
                f" OR NOT DART_BULLET_USES_FLOAT64 STREQUAL {recorded}"
                f" OR NOT DART_BULLET_MINIMUM_VERSION STREQUAL {minimum}"
                ' OR NOT DART_BULLET_PC_LIBS STREQUAL " ${expected_pc}"'
                ' OR NOT DART_BULLET_PC_CFLAGS STREQUAL " -I${BULLET_INCLUDE_DIRS}"'
            )
    result = configure(
        tmp_path,
        f'set(lib "{lib}")\n'
        'set(prefix "")\n'
        "if(CMAKE_FIND_LIBRARY_PREFIXES)\n"
        "  list(GET CMAKE_FIND_LIBRARY_PREFIXES 0 prefix)\n"
        "endif()\n"
        "list(GET CMAKE_FIND_LIBRARY_SUFFIXES 0 ext)\n"
        'file(MAKE_DIRECTORY "${lib}")\n'
        f"foreach(stem {stems})\n"
        '  file(TOUCH "${lib}/${prefix}${stem}${ext}")\n'
        "endforeach()\n"
        'set(collision "${lib}/${prefix}BulletCollision${ext}")\n'
        'set(math "${lib}/${prefix}LinearMath${ext}")\n'
        f"set(DART_USE_SYSTEM_BULLET ON)\n{settings}\n"
        "if(TARGET Bullet)\n"
        "  get_target_property(bt_target Bullet ALIASED_TARGET)\n"
        "  if(NOT bt_target)\nset(bt_target Bullet)\nendif()\n"
        "  get_target_property(bt_includes ${bt_target} INTERFACE_INCLUDE_DIRECTORIES)\n"
        "  if(NOT bt_includes)\n"
        f'    target_include_directories(${{bt_target}} INTERFACE "{include.as_posix()}")\n'
        "  endif()\nendif()\n"
        'file(MAKE_DIRECTORY "${lib}/include/LinearMath")\n'
        f'file(WRITE "${{lib}}/include/LinearMath/btScalar.h" "#define BT_BULLET_VERSION {version}\\n")\n'
        "include(DARTFindBullet)\n"
        f'set(expected "${{lib}}/${{prefix}}BulletCollision{suffix}${{ext}}" '
        f'"${{lib}}/${{prefix}}LinearMath{suffix}${{ext}}")\n'
        'list(JOIN expected " " expected_pc)\n'
        "if(TARGET Bullet)\n"
        "  get_target_property(linked Bullet INTERFACE_LINK_LIBRARIES)\n"
        "endif()\n"
        f"if({check})\n"
        '  message(FATAL_ERROR "Bullet links ${linked}")\n'
        "endif()\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    # CMake wraps warning text, so compare whitespace-normalized output.
    warning = "must not load both Bullet precisions" in " ".join(result.stderr.split())
    assert warning == (expected in ("warned", "rejected")), result.stderr
    if expected == "built-double":
        cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")


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


def fake_curl(tmp_path, lookup, version):
    # Config mode finds a CURL package; module mode reads curl/curlver.h at the
    # cache entries that stand in for find_path and find_library.
    if lookup == "config":
        config_package(tmp_path, "CURL", version, "CURL::libcurl")
        # Let only the requested minimum decide, not a same-major rule.
        (tmp_path / "prefix/lib/cmake/CURL/CURLConfigVersion.cmake").write_text(
            f'set(PACKAGE_VERSION "{version}")\n'
            "if(NOT PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)\n"
            "  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
            "endif()\n"
        )
        return ()
    include = tmp_path / "include"
    (include / "curl").mkdir(parents=True)
    (include / "curl" / "curlver.h").write_text(
        f'#define LIBCURL_VERSION "{version}"\n'
    )
    library = tmp_path / "library"
    library.touch()
    return (
        f"-DCURL_INCLUDE_DIR={include.as_posix()}",
        f"-DCURL_LIBRARY={library.as_posix()}",
    )


@pytest.mark.parametrize("lookup", ["config", "module"])
@pytest.mark.parametrize("version,accepted", [("8.4.0", False), ("8.5.0", True)])
def test_curl_floor(tmp_path, lookup, version, accepted):
    # 8.5.0 is Ubuntu 24.04's libcurl.
    result = configure(
        tmp_path,
        "include(DARTFindCURL)\n"
        "if(CURL_FOUND)\n"
        + ("" if accepted else 'message(FATAL_ERROR "Accepted unsupported version")\n')
        + "else()\n"
        + ('message(FATAL_ERROR "Rejected supported version")\n' if accepted else "")
        + "endif()\n",
        cmake_args=fake_curl(tmp_path, lookup, version),
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("lookup", ["config", "module"])
def test_assets_reject_old_curl_with_minimum_version_message(tmp_path, lookup):
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        # CMake 3.28's FindOpenSSL calls pkg_check_modules, which configure()
        # leaves undefined by disabling PkgConfig.
        "set(CMAKE_DISABLE_FIND_PACKAGE_OpenSSL TRUE)\n"
        f'include("{(ROOT / "dart/utils/assets/CMakeLists.txt").as_posix()}")\n',
        cmake_args=fake_curl(tmp_path, lookup, "8.4.0"),
    )
    assert result.returncode != 0
    assert (
        "dart-utils-assets requires libcurl >= 8.5.0" in result.stdout + result.stderr
    )


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
        "set(DART_USE_SYSTEM_FILAMENT ON)\nset(DART_USE_SYSTEM_GLFW ON)\n"
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


_GUI_PROVIDER = (ROOT / "cmake/DARTFilamentProvider.cmake").read_text()
GLFW_FIXTURE = (
    "include(DARTMacros)\n"
    + _GUI_PROVIDER[
        _GUI_PROVIDER.index("if(DART_USE_SYSTEM_GLFW)") : _GUI_PROVIDER.index(
            "find_package(PNG)"
        )
    ]
)


@pytest.mark.parametrize(
    "version,accepted",
    [(None, False), ("3.3.10", False), ("3.4.0", True), ("3.5.1", True)],
)
def test_system_glfw_requires_3_4(tmp_path, version, accepted):
    if version is not None:
        config_package(tmp_path, "glfw3", version, "glfw")
    result = configure(tmp_path, "set(DART_USE_SYSTEM_GLFW ON)\n" + GLFW_FIXTURE)
    output = result.stdout + result.stderr
    assert (result.returncode == 0) == accepted, output
    if not accepted:
        flat = " ".join(output.split())
        assert "requires GLFW 3.4" in flat
        assert "DART_USE_SYSTEM_GLFW=OFF" in flat


@pytest.mark.parametrize("shared", ["ON", "OFF"])
def test_fetched_glfw_is_static_hidden_and_scoped(tmp_path, shared):
    # A stand-in source tree reports the settings DART gives GLFW's configure.
    seed = tmp_path / "glfw-seed"
    seed.mkdir()
    (seed / "CMakeLists.txt").write_text(
        'message(STATUS "GLFW: ${GLFW_LIBRARY_TYPE} ${GLFW_BUILD_WAYLAND} '
        "${GLFW_BUILD_DOCS} ${GLFW_BUILD_EXAMPLES} ${GLFW_BUILD_TESTS} "
        "${GLFW_INSTALL} ${CMAKE_C_VISIBILITY_PRESET} ${CMAKE_UNITY_BUILD} "
        '[${CMAKE_C_FLAGS}]")\n'
    )
    # macOS rejects the fetched GLFW (tested above).
    result = configure(
        tmp_path,
        f"set(APPLE OFF)\nset(BUILD_SHARED_LIBS {shared})\n"
        "set(DART_USE_SYSTEM_GLFW OFF)\n"
        + GLFW_FIXTURE
        + "if(DEFINED GLFW_INSTALL OR CMAKE_C_VISIBILITY_PRESET OR CMAKE_C_FLAGS)\n"
        '  message(FATAL_ERROR "GLFW settings leaked")\nendif()\n',
        cmake_args=[f"-DFETCHCONTENT_SOURCE_DIR_GLFW={seed.as_posix()}"],
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    # Only a static dart-gui exports GLFW, so only static builds install it.
    install = "OFF" if shared == "ON" else "ON"
    assert f"GLFW: STATIC OFF OFF OFF OFF {install} hidden OFF [ -w]" in output


@pytest.mark.parametrize("shared", ["ON", "OFF"])
def test_fetched_glfw_is_rejected_on_macos(tmp_path, shared):
    result = configure(
        tmp_path,
        f"set(APPLE ON)\nset(BUILD_SHARED_LIBS {shared})\n"
        "set(DART_USE_SYSTEM_GLFW OFF)\n" + GLFW_FIXTURE,
        cmake_args=["-DFETCHCONTENT_FULLY_DISCONNECTED=ON"],
    )
    flat = " ".join((result.stdout + result.stderr).split())
    assert result.returncode != 0
    assert "DART_USE_SYSTEM_GLFW=OFF is not supported on macOS" in flat


def test_fetched_glfw_fails_fast_offline(tmp_path):
    result = configure(
        tmp_path,
        "set(APPLE OFF)\nset(DART_USE_SYSTEM_GLFW OFF)\n" + GLFW_FIXTURE,
        cmake_args=["-DFETCHCONTENT_FULLY_DISCONNECTED=ON"],
    )
    flat = " ".join((result.stdout + result.stderr).split())
    assert result.returncode != 0
    assert "Cannot fetch glfw with FETCHCONTENT_FULLY_DISCONNECTED=ON" in flat
    assert "DART_USE_SYSTEM_GLFW=ON" in flat


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
        "set(sdformat_VERSION stale)\n"
        "set(SDFORMAT_VERSION stale)\n"
        "include(DARTFindsdformat)\n"
        f"if(NOT SDFORMAT_FOUND STREQUAL {'TRUE' if accepted else 'FALSE'})\n"
        'message(FATAL_ERROR "Incorrect sdformat acceptance")\nendif()\n'
        + (
            f'if(NOT SDFORMAT_LIBRARIES STREQUAL "{target}" OR NOT DART_SDFORMAT_PACKAGE STREQUAL "{package}" OR NOT sdformat_VERSION STREQUAL "{version}" OR NOT SDFORMAT_VERSION STREQUAL "{version}")\n'
            'message(FATAL_ERROR "Incorrect sdformat selection")\nendif()\n'
            if accepted
            else 'if(NOT sdformat_VERSION STREQUAL "" OR NOT SDFORMAT_VERSION STREQUAL "")\n'
            'message(FATAL_ERROR "Rejected sdformat kept its version")\nendif()\n'
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
    discovery = (ROOT / "cmake/DARTFindDependencies.cmake").read_text()
    start = discovery.index("if(NOT DART_BUILD_IO)")
    discovery = discovery[
        start : discovery.index("\nendif()", start) + len("\nendif()")
    ]
    assert "dart_find_package(sdformat)" in discovery
    result = configure(
        tmp_path,
        "set(DART_BUILD_IO OFF)\n"
        "function(add_subdirectory)\n"
        'message(FATAL_ERROR "IO should not be configured")\nendfunction()\n'
        + io_addition
        + '\nset(HAVE_SDFORMAT TRUE CACHE BOOL "previously enabled")\n'
        + discovery
        + '\nif(HAVE_SDFORMAT)\nmessage(FATAL_ERROR "Stale SDF support")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("component", ["io", "utils", "utils-urdf", "io-sdf"])
@pytest.mark.parametrize("major", [None, 15, 16])
@pytest.mark.parametrize("static", [False, True])
@pytest.mark.parametrize("sdf_enabled", [False, True])
def test_installed_io_components_only_require_sdformat_for_static_builds(
    tmp_path, component, major, static, sdf_enabled
):
    if major is not None:
        config_package(tmp_path, "sdformat", f"{major}.3.0", "sdformat::sdformat")
    prefix = tmp_path / "prefix"
    (prefix / "include").mkdir(parents=True)
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
        ("io-sdf", "io", "dart-io-sdf"),
        ("utils", "io", "dart-utils"),
        ("utils-urdf", "utils", "dart-utils-urdf"),
    ):
        if name == "io-sdf" and not sdf_enabled:
            continue
        (package_dir / f"dart_{name}Component.cmake").write_text(
            render(
                template,
                component=name,
                internal_dependencies=dependencies,
                libraries=target,
                external_dependencies=(
                    "sdformat" if static and sdf_enabled and name == "io" else ""
                ),
            )
        )
        (package_dir / f"dart_{name}Targets.cmake").write_text(
            f"add_library({target} INTERFACE IMPORTED)\n"
        )
    shutil.copy(ROOT / "cmake/DARTFindsdformat.cmake", package_dir)
    expected_found = (component != "io-sdf" or sdf_enabled) and (
        not static or not sdf_enabled or major == 15
    )
    required = not sdf_enabled and component == "io-sdf"
    result = configure(
        tmp_path,
        f"find_package(DART {'REQUIRED' if required else 'QUIET'} CONFIG COMPONENTS {component})\n"
        f"if({'NOT DART_FOUND' if expected_found else 'DART_FOUND'})\n"
        'message(FATAL_ERROR "Incorrect installed IO component availability")\nendif()\n'
        + (
            'if(NOT SDFORMAT_LIBRARIES STREQUAL "sdformat::sdformat")\n'
            'message(FATAL_ERROR "Selected SDK target did not reach installed callers")\nendif()\n'
            if static and sdf_enabled and major == 15 and expected_found
            else ""
        ),
    )
    if required:
        assert result.returncode != 0
        assert "missing: io-sdf" in result.stdout + result.stderr
    else:
        assert result.returncode == 0, result.stdout + result.stderr


def bullet_module(tmp_path, version=325):
    include = tmp_path / "discovered"
    (include / "LinearMath").mkdir(parents=True)
    (include / "LinearMath/btScalar.h").write_text(
        f"#define BT_BULLET_VERSION {version}\n"
    )
    modules = tmp_path / "modules"
    modules.mkdir(exist_ok=True)
    # Discovery-only names are separate from the target graph used by ABI probes.
    (modules / "FindBullet.cmake").write_text(
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        f'set(BULLET_INCLUDE_DIRS "{include.as_posix()}")\n'
        f'set(BULLET_INCLUDE_DIR "{include.as_posix()}")\n'
        "set(BULLET_COLLISION_LIBRARY BulletCollision)\n"
        "set(BULLET_MATH_LIBRARY LinearMath)\n"
    )
    return include


@pytest.mark.parametrize(
    "edge,accepted",
    [
        ("$<TARGET_NAME_IF_EXISTS:BulletCollision>", True),
        ("$<TARGET_NAME_IF_EXISTS:existing_single>", False),
        ("$<$<PLATFORM_ID:Linux>:libLinearMath.so>", False),
        ("$<$<PLATFORM_ID:Nowhere>:libLinearMath.so>", True),
        ("$<$<PLATFORM_ID:LINUX>:libLinearMath.so>", True),
        ("$<$<AND:$<BOOL:1>,$<BOOL:1>>:libLinearMath.so>", False),
        ("$<TARGET_PROPERTY:unresolved,INTERFACE_LINK_LIBRARIES>", False),
    ],
)
def test_bullet_active_graph(tmp_path, edge, accepted):
    include = bullet_module(tmp_path)
    result = configure(
        tmp_path,
        "set(CMAKE_SYSTEM_NAME Linux)\n"
        "add_library(existing_single INTERFACE IMPORTED)\n"
        "target_link_libraries(existing_single INTERFACE libLinearMath.so)\n"
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        f'target_link_libraries(Bullet INTERFACE libBulletCollision-float64.so "{edge}")\n'
        "set(DART_BULLET_REQUIRE_FLOAT64 TRUE)\ninclude(DARTFindBullet)\n"
        f'if({"NOT " if accepted else ""}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong graph acceptance")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    "precisions",
    [
        ("double", "single"),
        ("single", "double"),
        ("double", "double"),
        ("single", "single"),
    ],
)
def test_bullet_built_graph_precision(tmp_path, precisions):
    include = bullet_module(tmp_path)
    source = tmp_path / "part.cpp"
    source.write_text(
        "#ifdef BT_USE_DOUBLE_PRECISION\nusing scalar = double;\n"
        "#else\nusing scalar = float;\n#endif\nvoid fixture(scalar) {}\n"
    )
    setup = "set(DART_USE_SYSTEM_BULLET ON)\nset(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
    for name, precision in zip(("BulletCollision", "LinearMath"), precisions):
        setup += f'add_library({name} STATIC "{source.as_posix()}")\n'
        if precision == "double":
            setup += f"target_compile_definitions({name} PRIVATE BT_USE_DOUBLE_PRECISION=0)\n"
    result = configure(
        tmp_path,
        setup + "add_library(Bullet INTERFACE)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision LinearMath)\n"
        "include(DARTFindBullet)\n"
        f'if({"NOT " if precisions[0] == precisions[1] else ""}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong built ABI acceptance")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")


@pytest.mark.parametrize("configuration", ["", "Release"])
def test_bullet_unlisted_imported_configuration_matches_cmake(tmp_path, configuration):
    include = bullet_module(tmp_path)
    library = (tmp_path / "libBulletCollision-float64.so").as_posix()
    graph = (
        "add_library(collision UNKNOWN IMPORTED)\n"
        f'set_target_properties(collision PROPERTIES IMPORTED_LOCATION_RELEASE "{library}")\n'
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        "target_link_libraries(Bullet INTERFACE collision)\n"
    )
    args = (f"-DCMAKE_BUILD_TYPE={configuration}", "-UCMAKE_CONFIGURATION_TYPES")
    result = configure(
        tmp_path,
        graph + "include(DARTFindBullet)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/found.txt" "${BULLET_FOUND}")\n',
        cmake_args=args,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    control = tmp_path / "control"
    control.mkdir()
    result = configure(
        control,
        graph
        + 'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/selected.txt" CONTENT "$<TARGET_LINKER_FILE:collision>")\n',
        cmake_args=args,
    )
    if configuration:
        assert result.returncode == 0, result.stdout + result.stderr
        assert (control / "build/selected.txt").read_text() == library
    else:
        assert result.returncode != 0
        assert "IMPORTED_LOCATION not set" in result.stderr
    assert (tmp_path / "build/found.txt").read_text() == (
        "TRUE" if result.returncode == 0 else "FALSE"
    )


@pytest.mark.parametrize("mapping", [None, "", "MISSING;DEBUG", ";DEBUG"])
@pytest.mark.parametrize("system", [None, "Windows"])
def test_bullet_imported_default_configuration(tmp_path, mapping, system):
    include = bullet_module(tmp_path)
    settings = ""
    if mapping is not None:
        settings = f'set_property(TARGET collision PROPERTY MAP_IMPORTED_CONFIG_RELEASE "{mapping}")\n'
    result = configure(
        tmp_path,
        "set(CMAKE_BUILD_TYPE Release)\n"
        "add_library(collision SHARED IMPORTED)\n"
        'set_target_properties(collision PROPERTIES IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"\n'
        'IMPORTED_LOCATION "/default/libBulletCollision-float64.so"\n'
        'IMPORTED_LOCATION_RELEASE "/release/libBulletCollision-float64.so"\n'
        'IMPORTED_LOCATION_DEBUG "/debug/libBulletCollision-float64.so")\n'
        "if(WIN32)\n"
        "set_target_properties(collision PROPERTIES\n"
        'IMPORTED_LOCATION "/default/BulletCollision-float64.dll"\n'
        'IMPORTED_LOCATION_RELEASE "/release/BulletCollision-float64.dll"\n'
        'IMPORTED_LOCATION_DEBUG "/debug/BulletCollision-float64.dll"\n'
        'IMPORTED_IMPLIB "/default/BulletCollision-float64.lib"\n'
        'IMPORTED_IMPLIB_RELEASE "/release/BulletCollision-float64.lib"\n'
        'IMPORTED_IMPLIB_DEBUG "/debug/BulletCollision-float64.lib")\nendif()\n'
        + settings
        + "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        "target_link_libraries(Bullet INTERFACE collision)\ninclude(DARTFindBullet)\n"
        'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/actual.txt" CONTENT "$<TARGET_LINKER_FILE:collision>")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/selected.txt" "${DART_BULLET_PC_LIBS}")\n',
        cmake_args=(f"-DCMAKE_SYSTEM_NAME={system}",) if system else (),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    actual = (tmp_path / "build/actual.txt").read_text()
    selected = (tmp_path / "build/selected.txt").read_text()
    assert actual in selected
    if system == "Windows" or os.name == "nt":
        assert actual.endswith(".lib")
        assert ".dll" not in selected


@pytest.mark.parametrize(
    "discovered,selected,floor", [(305, 325, "3.17"), (325, 305, "3.05")]
)
def test_bullet_selected_header_floor(tmp_path, discovered, selected, floor):
    bullet_module(tmp_path, discovered)
    include = tmp_path / "selected"
    (include / "LinearMath").mkdir(parents=True)
    (include / "LinearMath/btScalar.h").write_text(
        f"#define BT_BULLET_VERSION {selected}\n"
    )
    result = configure(
        tmp_path,
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "$<BUILD_INTERFACE:{include.as_posix()}>")\n'
        "target_link_libraries(Bullet INTERFACE libBulletCollision.so)\n"
        "include(DARTFindBullet)\n"
        f'if(NOT DART_BULLET_MINIMUM_VERSION STREQUAL "{floor}")\n'
        'message(FATAL_ERROR "Wrong selected header floor: ${DART_BULLET_MINIMUM_VERSION}")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


def cmake_fixture_run(*args, env=None):
    result = subprocess.run(
        [str(a) for a in args], capture_output=True, text=True, env=env
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result


def build_bullet_abi_fixture(
    tmp_path, precision, linkage="SHARED", suffix="", version=325
):
    """Real scalar-dependent Collision and LinearMath symbols, never touched files."""
    source = tmp_path / "source"
    source.mkdir(parents=True)
    include = source / "include"
    (include / "LinearMath").mkdir(parents=True)
    (include / "LinearMath/btScalar.h").write_text(
        f"#pragma once\n#define BT_BULLET_VERSION {version}\n"
        "#ifdef BT_USE_DOUBLE_PRECISION\nusing btScalar = double;\n"
        "#else\nusing btScalar = float;\n#endif\n"
    )
    (include / "btBulletCollisionCommon.h").write_text(
        '#pragma once\n#include "LinearMath/btScalar.h"\n'
        "struct btVector3 { btVector3(btScalar, btScalar, btScalar) {} };\n"
        "struct btStaticPlaneShape { btStaticPlaneShape(const btVector3&, btScalar); };\n"
    )
    (include / "LinearMath/btConvexHullComputer.h").write_text(
        '#pragma once\n#include "btScalar.h"\n'
        "struct btConvexHullComputer { btScalar compute(const float*, int, int, btScalar, btScalar); };\n"
    )
    (source / "collision.cpp").write_text(
        '#include "btBulletCollisionCommon.h"\n'
        '#include "LinearMath/btConvexHullComputer.h"\n'
        "btStaticPlaneShape::btStaticPlaneShape(const btVector3&, btScalar) {\n"
        "btConvexHullComputer hull; hull.compute(static_cast<const float*>(nullptr), 0, 0, 0., 0.); }\n"
    )
    (source / "math.cpp").write_text(
        '#include "LinearMath/btConvexHullComputer.h"\n'
        "btScalar btConvexHullComputer::compute(const float*, int, int, btScalar, btScalar) { return 0; }\n"
    )
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\nproject(BulletFixture LANGUAGES CXX)\n"
        "set(CMAKE_CXX_STANDARD 17)\nset(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)\n"
        f"add_library(BulletCollision {linkage} collision.cpp)\n"
        f"add_library(LinearMath {linkage} math.cpp)\n"
        "foreach(t BulletCollision LinearMath)\n"
        f'set_target_properties(${{t}} PROPERTIES OUTPUT_NAME "${{t}}{suffix}")\n'
        "target_include_directories(${t} PRIVATE include)\n"
        + (
            "target_compile_definitions(${t} PRIVATE BT_USE_DOUBLE_PRECISION)\n"
            if precision == "double"
            else ""
        )
        + "endforeach()\ntarget_link_libraries(BulletCollision PRIVATE LinearMath)\n"
        "install(TARGETS BulletCollision LinearMath DESTINATION lib)\n"
        "install(DIRECTORY include/ DESTINATION include)\n"
        'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/link-files.txt" CONTENT '
        '"$<TARGET_LINKER_FILE_NAME:BulletCollision>\n$<TARGET_LINKER_FILE_NAME:LinearMath>\n")\n'
        'install(FILES "${CMAKE_BINARY_DIR}/link-files.txt" DESTINATION .)\n'
    )
    prefix = tmp_path / "install"
    cmake_fixture_run(
        shutil.which("cmake"),
        "-G",
        "Ninja",
        "-S",
        source,
        "-B",
        tmp_path / "build",
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
    )
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
    cmake_fixture_run(shutil.which("cmake"), "--install", tmp_path / "build")
    return prefix


def bullet_imports(prefix, suffix="", math_prefix=None):
    math_prefix = math_prefix or prefix
    prefix_text = prefix.as_posix().replace("\\", "\\\\")
    collision_file = (
        prefix / "lib" / (prefix / "link-files.txt").read_text().splitlines()[0]
    )
    math_file = (
        math_prefix
        / "lib"
        / (math_prefix / "link-files.txt").read_text().splitlines()[1]
    )
    assert collision_file.is_file() and math_file.is_file()
    assert not collision_file.name.endswith(".dll") and not math_file.name.endswith(
        ".dll"
    )
    collision_text = collision_file.as_posix().replace("\\", "\\\\")
    math_text = math_file.as_posix().replace("\\", "\\\\")
    return (
        "add_library(collision UNKNOWN IMPORTED)\nadd_library(math UNKNOWN IMPORTED)\n"
        f'set(collision_file "{collision_text}")\n'
        f'set(math_file "{math_text}")\n'
        'set_target_properties(collision PROPERTIES IMPORTED_LOCATION "${collision_file}")\n'
        'set_target_properties(math PROPERTIES IMPORTED_LOCATION "${math_file}")\n'
        "add_library(collision_wrapper INTERFACE IMPORTED)\n"
        "target_link_libraries(collision_wrapper INTERFACE collision)\n"
        "add_library(Bullet INTERFACE IMPORTED)\n"
        "target_link_libraries(Bullet INTERFACE collision_wrapper math)\n"
        f'target_include_directories(Bullet INTERFACE "{prefix_text}/include")\n'
    )


BULLET_CONSUMER = """
#include <btBulletCollisionCommon.h>
#include <LinearMath/btConvexHullComputer.h>
int main() {
  btVector3 v(0., 0., 1.);
  btStaticPlaneShape plane(v, 0.);
  btConvexHullComputer hull;
  return static_cast<int>(hull.compute(static_cast<const float*>(nullptr), 0, 0, 0., 0.));
}
"""


def install_bullet_dart_fixture(
    tmp_path, prefix, linkage="SHARED", suffix="", pc_settings="", discovery_version=305
):
    """Install the production component and probe output with a public dependency."""
    source = tmp_path / "source"
    source.mkdir(parents=True)
    stage = tmp_path / "install"
    (source / "core.cpp").write_text(
        BULLET_CONSUMER.replace("int main()", "int dart_fixture()")
    )
    backend = (
        (ROOT / "dart/collision/bullet/CMakeLists.txt")
        .read_text()
        .split("# Search all header and source files")[0]
    )
    (source / "backend.cmake").write_text(backend)
    pc_block = (
        (ROOT / "CMakeLists.txt")
        .read_text()
        .split("# Generate the DART pkg-config\n")[1]
        .split("# Install a Catkin")[0]
    )
    (source / "pkgconfig.cmake").write_text(pc_block)
    (source / "discovery/LinearMath").mkdir(parents=True)
    (source / "discovery/LinearMath/btScalar.h").write_text(
        f"#define BT_BULLET_VERSION {discovery_version}\n"
    )
    (source / "FindBullet.cmake").write_text(
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        f'set(BULLET_INCLUDE_DIRS "{source.as_posix()}/discovery")\n'
        f'set(BULLET_INCLUDE_DIR "{source.as_posix()}/discovery")\n'
        'set(BULLET_COLLISION_LIBRARY "${collision_file}")\n'
        'set(BULLET_MATH_LIBRARY "${math_file}")\n'
    )
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\nproject(dart LANGUAGES CXX)\n"
        "set(CMAKE_CXX_STANDARD 17)\nset(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)\n"
        f'set(CMAKE_MODULE_PATH "{source.as_posix()}" "{ROOT.as_posix()}/cmake")\n'
        "set(DART_USE_SYSTEM_BULLET ON)\ninclude(DARTMacros)\n"
        + bullet_imports(prefix, suffix)
        + "include(backend.cmake)\n"
        f"add_library(dart {linkage} core.cpp)\n"
        'target_link_libraries(dart PUBLIC "$<BUILD_INTERFACE:Bullet>" "$<INSTALL_INTERFACE:$<1:Bullet>>")\n'
        "if(BT_USE_DOUBLE_PRECISION)\ntarget_compile_definitions(dart PUBLIC BT_USE_DOUBLE_PRECISION)\nendif()\n"
        "set(PROJECT_NAME_UPPERCASE DART)\n"
        f'set(DART_SOURCE_DIR "{ROOT.as_posix()}")\n'
        "set(CONFIG_INSTALL_DIR lib/cmake/DART)\nset(LIBRARY_INSTALL_DIR lib)\n"
        "include(Components)\ninitialize_component_helpers(dart)\n"
        "add_component(dart dart)\nadd_component_targets(dart dart dart)\n"
        "add_component_dependency_packages(dart dart Bullet)\ninstall_component_exports(dart)\n"
        f'install(FILES "{ROOT.as_posix()}/cmake/DARTFindPackageVersion.cmake" DESTINATION lib/cmake/DART)\n'
        + "set(CMAKE_INSTALL_LIBDIR lib)\ninclude(GNUInstallDirs)\nset(DART_BINARY_DIR ${CMAKE_BINARY_DIR})\n"
        'set(DART_VERSION 7.0.0)\nset(DART_PKG_DESC "DART package fixture")\n'
        "set(CMAKE_DISABLE_FIND_PACKAGE_PkgConfig TRUE)\n"
        + pc_settings
        + "include(pkgconfig.cmake)\n"
    )
    cmake_fixture_run(
        shutil.which("cmake"),
        "-G",
        "Ninja",
        "-S",
        source,
        "-B",
        tmp_path / "build",
        f"-DCMAKE_INSTALL_PREFIX={stage}",
    )
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
    cmake_fixture_run(shutil.which("cmake"), "--install", tmp_path / "build")
    return stage


@pytest.mark.parametrize("precision", ["single", "double"])
@pytest.mark.parametrize("linkage", ["STATIC", "SHARED"])
@pytest.mark.parametrize("suffix", ["", "-float64"])
def test_bullet_installed_scalar_abi(tmp_path, precision, linkage, suffix):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", precision, linkage, suffix)
    other = build_bullet_abi_fixture(
        tmp_path / "other",
        "double" if precision == "single" else "single",
        linkage,
        suffix,
    )
    stage = install_bullet_dart_fixture(tmp_path / "dart", prefix, linkage, suffix)
    component = stage / "lib/cmake/DART/dart_dartComponent.cmake"
    for name, selected, math_prefix, language, accepted in [
        ("same", prefix, prefix, "CXX", True),
        ("other", other, other, "CXX", False),
        ("mixed", prefix, other, "CXX", False),
        ("no-language", prefix, prefix, "NONE", False),
    ]:
        source = tmp_path / ("consumer-" + name)
        source.mkdir()
        (source / "main.cpp").write_text(BULLET_CONSUMER)
        (source / "FindBullet.cmake").write_text(
            "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
            f'set(BULLET_INCLUDE_DIRS "{selected.as_posix()}/include")\n'
            f'set(BULLET_INCLUDE_DIR "{selected.as_posix()}/include")\n'
            'set(BULLET_COLLISION_LIBRARY "${collision_file}")\n'
            'set(BULLET_MATH_LIBRARY "${math_file}")\n'
        )
        (source / "CMakeLists.txt").write_text(
            f"cmake_minimum_required(VERSION 3.22.1)\nproject(Consumer LANGUAGES {language})\n"
            "set(CMAKE_CXX_STANDARD 17)\nset(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)\n"
            f'set(CMAKE_MODULE_PATH "{source.as_posix()}")\n'
            + bullet_imports(selected, suffix, math_prefix=math_prefix)
            + f'include("{component.as_posix()}")\n'
            "get_property(found DIRECTORY PROPERTY DART_dart_FOUND)\n"
            + (
                'if(NOT found)\nmessage(FATAL_ERROR "Matching ABI rejected")\nendif()\n'
                f'include("{stage.as_posix()}/lib/cmake/DART/dart_dartTargets.cmake")\n'
                "add_executable(consumer main.cpp)\ntarget_link_libraries(consumer PRIVATE dart)\n"
                if accepted
                else 'if(found OR BULLET_FOUND OR Bullet_FOUND)\nmessage(FATAL_ERROR "Unverified or mismatching ABI accepted")\nendif()\n'
            )
        )
        cmake_fixture_run(
            shutil.which("cmake"), "-G", "Ninja", "-S", source, "-B", source / "build"
        )
        if accepted:
            cmake_fixture_run(shutil.which("cmake"), "--build", source / "build")
            cmake_fixture_run(
                source
                / ("build/consumer.exe" if os.name == "nt" else "build/consumer"),
                env={
                    **os.environ,
                    "PATH": os.pathsep.join(
                        (str(prefix / "lib"), str(stage / "bin"), os.environ["PATH"])
                    ),
                },
            )

    assert f'set(DART_BULLET_REQUIRE_PRECISION "{precision}")' in component.read_text()
    # Only a fetched Bullet that DART installs is searched in DART's prefix.
    assert 'set(DART_BULLET_SEARCH_DIRS "")' in component.read_text()


@pytest.mark.parametrize("debug", [False, True])
@pytest.mark.parametrize("earlier", [False, True])
def test_bullet_installed_fetched_sdk_precedes_consumer_bullet(
    tmp_path, earlier, debug
):
    """An installed DART links the fetched Bullet that it installed beside itself."""
    source = bullet_fallback_source(tmp_path)
    if debug:
        # Like upstream Bullet, a Debug build installs only _Debug libraries.
        top = source / "CMakeLists.txt"
        top.write_text(
            top.read_text().replace(
                "add_subdirectory(src)",
                'set(CMAKE_DEBUG_POSTFIX "_Debug" CACHE STRING "")\nadd_subdirectory(src)',
            )
        )
    src = source / "src"
    # Scalar-dependent Collision and LinearMath symbols for the consumer's probe.
    (src / "LinearMath/btConvexHullComputer.h").write_text(
        '#pragma once\n#include "btScalar.h"\n'
        "struct btConvexHullComputer { btScalar compute(const float*, int, int, btScalar, btScalar); };\n"
    )
    with (src / "btBulletCollisionCommon.h").open("a") as header:
        header.write(
            "#ifdef __cplusplus\n"
            "struct btVector3 { btVector3(btScalar, btScalar, btScalar) {} };\n"
            "struct btStaticPlaneShape { btStaticPlaneShape(const btVector3&, btScalar); };\n"
            "#endif\n"
        )
    with (src / "BulletCollision.cpp").open("a") as collision:
        collision.write(
            '#include "btBulletCollisionCommon.h"\n'
            "btStaticPlaneShape::btStaticPlaneShape(const btVector3&, btScalar) {}\n"
        )
    with (src / "LinearMath.cpp").open("a") as math:
        math.write(
            '#include "LinearMath/btConvexHullComputer.h"\n'
            "btScalar btConvexHullComputer::compute(const float*, int, int, btScalar, btScalar) { return 0; }\n"
        )
    # Pixi, conda-forge and distribution prefixes hold unsuffixed
    # single-precision Bullet, which a consumer's search paths can list first.
    decoy = build_bullet_abi_fixture(tmp_path / "decoy", "single")
    # CMake's FindBullet also requires the dynamics and soft-body libraries.
    collision = (decoy / "link-files.txt").read_text().splitlines()[0]
    for name in ("BulletDynamics", "BulletSoftBody"):
        shutil.copy(
            decoy / "lib" / collision,
            decoy / "lib" / collision.replace("BulletCollision", name),
        )
    project = tmp_path / "dart"
    project.mkdir()
    (project / "core.cpp").write_text(
        BULLET_CONSUMER.replace("int main()", "int dart_fixture()")
    )
    (project / "backend.cmake").write_text(
        (ROOT / "dart/collision/bullet/CMakeLists.txt")
        .read_text()
        .split("# Search all header and source files")[0]
    )
    (project / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\nproject(dart LANGUAGES C CXX)\n"
        "set(CMAKE_CXX_STANDARD 17)\nset(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)\n"
        f'set(CMAKE_MODULE_PATH "{(ROOT / "cmake").as_posix()}")\n'
        f'set(FETCHCONTENT_SOURCE_DIR_BULLET "{source.as_posix()}")\n'
        "set(CONFIG_INSTALL_DIR share/dart/cmake)\n"
        + bullet_fallback_content()
        + "include(backend.cmake)\nadd_library(dart SHARED core.cpp)\n"
        'target_link_libraries(dart PUBLIC "$<BUILD_INTERFACE:Bullet>" "$<INSTALL_INTERFACE:$<1:Bullet>>")\n'
        "if(BT_USE_DOUBLE_PRECISION)\ntarget_compile_definitions(dart PUBLIC BT_USE_DOUBLE_PRECISION)\nendif()\n"
        f'set(PROJECT_NAME_UPPERCASE DART)\nset(DART_SOURCE_DIR "{ROOT.as_posix()}")\n'
        "include(Components)\ninitialize_component_helpers(dart)\n"
        "add_component(dart dart)\nadd_component_targets(dart dart dart)\n"
        "add_component_dependency_packages(dart dart Bullet)\ninstall_component_exports(dart)\n"
        f'install(FILES "{(ROOT / "cmake/DARTFindPackageVersion.cmake").as_posix()}"\n'
        "  DESTINATION share/dart/cmake)\n"
        # The production config file, which includes the components in a function.
        "set(DART_USE_SYSTEM_IMGUI ON)\nset(DART_MAJOR_VERSION 7)\n"
        "include(CMakePackageConfigHelpers)\n"
        f'configure_package_config_file("{(ROOT / "cmake/DARTConfig.cmake.in").as_posix()}"\n'
        '  "${CMAKE_BINARY_DIR}/DARTConfig.cmake" INSTALL_DESTINATION share/dart/cmake\n'
        "  PATH_VARS INCLUDE_INSTALL_DIR)\n"
        'install(FILES "${CMAKE_BINARY_DIR}/DARTConfig.cmake" DESTINATION share/dart/cmake)\n'
    )
    cmake = shutil.which("cmake")
    build = tmp_path / "dart-build"
    cmake_fixture_run(
        cmake,
        "-G",
        "Ninja",
        "-S",
        project,
        "-B",
        build,
        f"-DCMAKE_INSTALL_PREFIX={(tmp_path / 'configured').as_posix()}",
        *(["-DCMAKE_BUILD_TYPE=Debug"] if debug else []),
    )
    cmake_fixture_run(cmake, "--build", build)
    # The recorded directories follow the components into a relocated prefix.
    stage = tmp_path / "install"
    cmake_fixture_run(cmake, "--install", build, "--prefix", stage)
    config = stage / "share/dart/cmake"
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    (consumer / "main.cpp").write_text(BULLET_CONSUMER)
    # Like a consumer that finds DART through DART_DIR, search only the decoy,
    # which the consumer may also find for itself before DART.
    (consumer / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\nproject(Consumer LANGUAGES CXX)\n"
        "set(CMAKE_CXX_STANDARD 17)\n"
        f'set(CMAKE_PREFIX_PATH "{decoy.as_posix()}")\n'
        "set(CMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH FALSE)\n"
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)\n"
        "set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH FALSE)\n"
        + ("find_package(Bullet MODULE REQUIRED)\n" if earlier else "")
        + "function(write_bullet_variables file)\n"
        "get_cmake_property(names VARIABLES)\n"
        'list(FILTER names INCLUDE REGEX "^(BULLET|Bullet)_")\n'
        'set(text "")\n'
        'foreach(name IN LISTS names)\nstring(APPEND text "${name}=${${name}}\\n")\nendforeach()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/${file}" "${text}")\n'
        "endfunction()\n"
        "write_bullet_variables(before.txt)\n"
        "find_package(DART REQUIRED CONFIG)\n"
        "write_bullet_variables(after.txt)\n"
        "get_target_property(includes Bullet INTERFACE_INCLUDE_DIRECTORIES)\n"
        "get_target_property(libraries Bullet INTERFACE_LINK_LIBRARIES)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/selected.txt" "${includes};${libraries}")\n'
        "add_executable(consumer main.cpp)\ntarget_link_libraries(consumer PRIVATE dart)\n"
    )
    cmake_fixture_run(
        cmake,
        "-G",
        "Ninja",
        "-S",
        consumer,
        "-B",
        consumer / "build",
        f"-DDART_DIR={config.as_posix()}",
        # A Debug-only installation serves every configuration.
        *(["-DCMAKE_BUILD_TYPE=Release"] if debug else []),
    )
    # DART's finder leaves the consumer's own FindBullet results unchanged.
    before = (consumer / "build/before.txt").read_text()
    assert (consumer / "build/after.txt").read_text() == before
    assert ("BULLET_FOUND=TRUE" in before and decoy.as_posix() in before) == earlier
    selected = (consumer / "build/selected.txt").read_text().split(";")
    assert len(selected) == 3, selected
    assert all(Path(path).is_relative_to(stage) for path in selected), selected
    assert all(("_Debug" in Path(path).name) == debug for path in selected[1:])
    cmake_fixture_run(cmake, "--build", consumer / "build")
    cmake_fixture_run(
        consumer / "build" / ("consumer.exe" if os.name == "nt" else "consumer"),
        env={
            **os.environ,
            "PATH": os.pathsep.join((str(stage / "bin"), os.environ["PATH"])),
        },
    )


BULLET_DEBUG_PAIR = (
    "optimized;BulletCollision;debug;BulletCollision{0};"
    "optimized;LinearMath;debug;LinearMath{0}"
)


@pytest.mark.parametrize(
    "headers,names,expected",
    [
        (True, ("BulletCollision", "LinearMath"), "BulletCollision;LinearMath"),
        # Bullet's Debug postfix, and DART's, which replaces it on reconfiguration.
        (
            True,
            (
                "BulletCollision",
                "LinearMath",
                "BulletCollision_Debug",
                "LinearMath_Debug",
            ),
            BULLET_DEBUG_PAIR.format("_Debug"),
        ),
        (
            True,
            ("BulletCollision", "LinearMath", "BulletCollisiond", "LinearMathd"),
            BULLET_DEBUG_PAIR.format("d"),
        ),
        # A Debug-only installation serves every configuration.
        (
            True,
            ("BulletCollision_Debug", "LinearMath_Debug"),
            "BulletCollision_Debug;LinearMath_Debug",
        ),
        (True, ("BulletCollisiond", "LinearMathd"), "BulletCollisiond;LinearMathd"),
        # Incomplete and mixed sets leave the search to the consumer's paths.
        (False, ("BulletCollision", "LinearMath"), None),
        (False, ("BulletCollision_Debug", "LinearMath_Debug"), None),
        (True, ("LinearMath",), None),
        (True, ("BulletCollision", "BulletCollision_Debug"), None),
        (True, ("BulletCollision", "LinearMath_Debug"), None),
    ],
)
def test_bullet_recorded_sdk_is_used_whole(tmp_path, headers, names, expected):
    """An installed DART uses its fetched Bullet whole, or searches as usual."""

    def sdk(include, lib, names):
        if include:
            (include / "LinearMath").mkdir(parents=True)
            (include / "btBulletCollisionCommon.h").touch()
            (include / "LinearMath/btScalar.h").write_text(
                "#define BT_BULLET_VERSION 325\n"
            )
        lib.mkdir(parents=True)
        for name in names:
            (lib / f"lib{name}.a").touch()

    def libraries(lib, items):
        return ";".join(
            (
                item
                if item in ("optimized", "debug")
                else (lib / f"lib{item}.a").as_posix()
            )
            for item in items.split(";")
        )

    # Another Bullet, Debug libraries included, on the consumer's search paths.
    foreign = tmp_path / "prefix"
    sdk(
        foreign / "include/bullet",
        foreign / "lib",
        ("BulletCollision", "LinearMath", "BulletCollision_Debug", "LinearMath_Debug"),
    )
    dart = tmp_path / "dart-prefix"
    sdk(dart / "include" if headers else None, dart / "lib", names)
    result = configure(
        tmp_path,
        'set(DART_BULLET_REQUIRE_FLOAT64 "")\nset(DART_BULLET_REQUIRE_VERSION 3.17)\n'
        f'set(DART_BULLET_SEARCH_DIRS "{dart.as_posix()}/include;{dart.as_posix()}/lib")\n'
        "include(DARTFindBullet)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/selected.txt"\n'
        '  "${BULLET_FOUND}\\n${BULLET_INCLUDE_DIRS}\\n${BULLET_LIBRARIES}")\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if expected:
        include = dart / "include"
        selected = libraries(dart / "lib", expected)
    else:
        include = foreign / "include/bullet"
        selected = libraries(foreign / "lib", BULLET_DEBUG_PAIR.format("_Debug"))
    assert (tmp_path / "build/selected.txt").read_text() == "\n".join(
        ("TRUE", include.as_posix(), selected)
    )
    if not expected:
        assert dart.as_posix() not in (tmp_path / "build/CMakeCache.txt").read_text()


def pkgconfig_fixture_executable():
    executable = Path(shutil.which("pkg-config"))
    raw = executable.with_name(executable.name + ".bin")
    # Conda's wrapper appends global module paths despite PKG_CONFIG_LIBDIR.
    return str(raw if raw.is_file() else executable)


def pkgconfig_consumer_options(package, *, static=False, env=None):
    flags = []
    content = ""
    for kind in ("--cflags", "--libs"):
        result = cmake_fixture_run(
            pkgconfig_fixture_executable(),
            "--define-prefix",
            *(["--static"] if static else []),
            kind,
            package,
            env=env,
        )
        selected = shlex.split(result.stdout)
        flags.extend(selected)
        for flag in selected:
            if kind == "--cflags":
                # Raw -I preserves literal Unix backslashes in serialization tests.
                command = "target_compile_options"
            elif flag.startswith("-L"):
                command, flag = "target_link_directories", flag[2:]
            else:
                command = "target_link_libraries"
                if flag.startswith("-l"):
                    flag = flag[2:]
            content += f"{command}(consumer PRIVATE [=[{flag}]=])\n"
    return content, flags


@pytest.mark.parametrize(
    "cflags", ["/Zc:inline /DWIN32 /D_WINDOWS /EHsc", "-fno-strict-aliasing -pthread"]
)
def test_pkgconfig_consumer_preserves_flag_roles(tmp_path, cflags):
    pcdir = tmp_path / "pkgconfig"
    pcdir.mkdir()
    (pcdir / "fixture.pc").write_text(
        "Name: fixture\nDescription: compiler and linker flag roles\nVersion: 1\n"
        f"Cflags: {cflags} -DFIXTURE_FLAG=1\nLibs: -L{tmp_path.as_posix()} -lfixture\n"
    )
    env = {**os.environ, "PKG_CONFIG_PATH": str(pcdir), "PKG_CONFIG_LIBDIR": str(pcdir)}
    env.pop("PKG_CONFIG_SYSROOT_DIR", None)
    options, _ = pkgconfig_consumer_options("fixture", env=env)
    source = tmp_path / "consumer.cpp"
    source.write_text("int main() { return 0; }\n")
    result = configure(
        tmp_path,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_executable(consumer "{source.as_posix()}")\n'
        + options
        + 'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/compile.txt" CONTENT "$<TARGET_PROPERTY:consumer,COMPILE_OPTIONS>")\n'
        'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/link.txt" CONTENT "$<TARGET_PROPERTY:consumer,LINK_LIBRARIES>")\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    compile_flags = (tmp_path / "build/compile.txt").read_text().split(";")
    link_flags = (tmp_path / "build/link.txt").read_text().split(";")
    assert all(flag in compile_flags for flag in shlex.split(cflags))
    assert all(flag not in link_flags for flag in shlex.split(cflags))
    assert "fixture" in link_flags and "fixture" not in compile_flags


@pytest.mark.parametrize("linkage", ["STATIC", "SHARED"])
@pytest.mark.parametrize("layout", ["plain", "spaces", "sysroot", "backslash"])
def test_bullet_pkgconfig_public_consumer(tmp_path, linkage, layout):
    folder = "bullet with spaces" if layout == "spaces" else "bullet"
    prefix = build_bullet_abi_fixture(tmp_path / folder, "double", linkage, "-float64")
    pc_settings = ""
    if layout == "sysroot":
        builder = tmp_path / "builder-sysroot"
        sys_prefix = builder / "opt/bullet"
        shutil.copytree(prefix, sys_prefix)
        prefix = sys_prefix
        pc_settings = (
            f'set(CMAKE_SYSROOT_COMPILE "{builder.as_posix()}")\n'
            f'set(CMAKE_SYSROOT_LINK "{builder.as_posix()}")\n'
            "include(DARTFindBullet)\n"
            "unset(CMAKE_SYSROOT_COMPILE)\nunset(CMAKE_SYSROOT_LINK)\n"
        )
    if layout == "backslash":
        # CMake normalizes literal Unix backslashes in compiler include paths.
        # Test this serialization boundary with the same compiled libraries.
        literal = tmp_path / "bullet\\literal with spaces"
        shutil.copytree(prefix, literal)
        pc_settings = (
            'get_filename_component(collision_name "${collision_file}" NAME)\n'
            'get_filename_component(math_name "${math_file}" NAME)\n'
            f"set(raw_prefix [=[{literal.as_posix()}]=])\n"
            '_dart_bullet_record(float64 INCLUDES "${raw_prefix}/include" '
            'LIBRARIES "${raw_prefix}/lib/${collision_name}" "${raw_prefix}/lib/${math_name}")\n'
        )
    relocatable_sysroot = layout == "sysroot" and sys.platform == "linux"
    if layout == "sysroot" and not relocatable_sysroot:
        # Non-ELF sysroots must reject serialization, then still build a native consumer.
        with pytest.raises(
            AssertionError, match="Cannot serialize the selected Bullet library"
        ):
            install_bullet_dart_fixture(
                tmp_path / "unsupported-sysroot",
                prefix,
                linkage,
                "-float64",
                pc_settings,
            )
        pc_settings = ""
    stage = install_bullet_dart_fixture(
        tmp_path / "dart", prefix, linkage, "-float64", pc_settings
    )
    pcfile = stage / "lib/pkgconfig/dart.pc"
    text = pcfile.read_text()
    assert "bullet-float64" not in text.split("Requires:")[1].splitlines()[0]
    relocated = tmp_path / "relocated dart"
    shutil.move(stage, relocated)
    pcdir = relocated / "lib/pkgconfig"
    env = {**os.environ, "PKG_CONFIG_PATH": str(pcdir), "PKG_CONFIG_LIBDIR": str(pcdir)}
    env.pop("PKG_CONFIG_SYSROOT_DIR", None)
    if relocatable_sysroot:
        assert str(builder) not in text
        consumer_root = tmp_path / "consumer-sysroot"
        shutil.copytree(prefix, consumer_root / "opt/bullet")
        # pkg-config applies the sysroot to the package itself as well.
        shutil.copytree(relocated, consumer_root / str(relocated).lstrip("/"))
        env["PKG_CONFIG_SYSROOT_DIR"] = str(consumer_root)
    source = tmp_path / "consumer.cpp"
    source.write_text(
        BULLET_CONSUMER.split("  btConvexHullComputer hull;")[0] + "  return 0;\n}\n"
    )
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    for static in (False, True):
        options, flags = pkgconfig_consumer_options("dart", static=static, env=env)
        content = (
            "cmake_minimum_required(VERSION 3.22.1)\nproject(Consumer LANGUAGES CXX)\n"
            "set(CMAKE_CXX_STANDARD 17)\n"
            f'add_executable(consumer "{source.as_posix()}")\n' + options
        )
        (consumer / "CMakeLists.txt").write_text(content)
        if relocatable_sysroot:
            assert all(str(builder) not in flag for flag in flags)
            assert "-I" + str(consumer_root / "opt/bullet/include") in flags
            assert "-L" + str(consumer_root / "opt/bullet/lib") in flags
        cmake_fixture_run(
            shutil.which("cmake"),
            "-G",
            "Ninja",
            "-S",
            consumer,
            "-B",
            consumer / str(static),
            env=env,
        )
        cmake_fixture_run(
            shutil.which("cmake"), "--build", consumer / str(static), env=env
        )
        assert sum("BulletCollision" in flag for flag in flags) == 1
        assert sum("LinearMath" in flag for flag in flags) == 1
    assert "BulletCollision" in text.split("Libs:")[1].splitlines()[0]
    assert "BulletCollision" not in text.split("Libs.private:")[1].splitlines()[0]


def test_bullet_pkgconfig_sysroot_boundary(tmp_path):
    bullet_module(tmp_path)
    result = configure(
        tmp_path,
        'set(CMAKE_SYSROOT "/builder/root")\n'
        'set(CMAKE_SYSROOT_COMPILE "/compile/root")\n'
        'set(CMAKE_SYSROOT_LINK "/link/root")\n'
        "include(DARTFindBullet)\n"
        '_dart_bullet_pc_path(path "/compile/root2/include" COMPILE)\n'
        'if(NOT path STREQUAL "/compile/root2/include")\nmessage(FATAL_ERROR "Stripped a partial prefix")\nendif()\n'
        '_dart_bullet_pc_path(path "/compile/root/include" COMPILE)\n'
        'if(NOT path STREQUAL "/include")\nmessage(FATAL_ERROR "Wrong compile root")\nendif()\n'
        '_dart_bullet_pc_path(path "/link/root/lib/libBulletCollision.a" LINK)\n'
        'if(NOT path STREQUAL "/lib/libBulletCollision.a")\nmessage(FATAL_ERROR "Wrong link root")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("version,floor", [(305, "3.05"), (325, "3.17")])
def test_bullet_installed_selected_version(tmp_path, version, floor):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "single", version=version)
    # Unrelated discovery deliberately advertises the opposite header version.
    stage = install_bullet_dart_fixture(
        tmp_path / "dart", prefix, discovery_version=325 if version == 305 else 305
    )
    component = stage / "lib/cmake/DART/dart_dartComponent.cmake"
    assert f'set(DART_BULLET_REQUIRE_VERSION "{floor}")' in component.read_text()
    assert f"bullet >= {floor}" in (stage / "lib/pkgconfig/dart.pc").read_text()
    for selected_version in (304, 305, 325):
        consumer = tmp_path / f"consumer-{selected_version}"
        consumer.mkdir()
        bullet_module(consumer, 325 if selected_version == 305 else 305)
        linked = consumer / "linked/LinearMath"
        linked.mkdir(parents=True)
        (linked / "btScalar.h").write_text(
            f"#define BT_BULLET_VERSION {selected_version}\n"
        )
        result = configure(
            consumer,
            "add_library(Bullet INTERFACE IMPORTED)\n"
            f'target_include_directories(Bullet INTERFACE "{linked.parent.as_posix()}")\n'
            "target_link_libraries(Bullet INTERFACE libBulletCollision.so)\n"
            f'set(DART_BULLET_REQUIRE_VERSION "{floor}")\n'
            "set(Bullet_VERSION 99.0)\nset(BULLET_VERSION 99.0 CACHE STRING old)\n"
            "include(DARTFindBullet)\n"
            f'if({"NOT " if selected_version >= (317 if version >= 317 else 305) else ""}BULLET_FOUND)\n'
            'message(FATAL_ERROR "Wrong installed version check")\nendif()\n',
        )
        assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    "built,imported",
    [
        ("double", "double"),
        ("double", "single"),
        ("single", "double"),
        ("single", "single"),
    ],
)
def test_bullet_hybrid_scalar_abi(tmp_path, built, imported):
    prefix = build_bullet_abi_fixture(tmp_path / "imported", imported)
    bullet_module(tmp_path)
    source = tmp_path / "local.cpp"
    source.write_text((tmp_path / "imported/source/collision.cpp").read_text())
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\nset(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_library(BulletCollision STATIC "{source.as_posix()}")\n'
        f'target_include_directories(BulletCollision PRIVATE "{prefix.as_posix()}/include")\n'
        + (
            "target_compile_definitions(BulletCollision PRIVATE BT_USE_DOUBLE_PRECISION)\n"
            if built == "double"
            else ""
        )
        + "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")\n'
        f'find_library(math_file NAMES LinearMath PATHS "{prefix.as_posix()}/lib" NO_DEFAULT_PATH REQUIRED)\n'
        'target_link_libraries(Bullet INTERFACE BulletCollision "${math_file}")\n'
        f"set(DART_BULLET_REQUIRE_PRECISION {built})\ninclude(DARTFindBullet)\n"
        f'if({"NOT " if built == imported else ""}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong hybrid ABI acceptance")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")


@pytest.mark.parametrize("precision", ["single", "double"])
@pytest.mark.parametrize("hybrid", [False, True])
def test_bullet_scalar_abi_ignores_header_warnings(tmp_path, precision, hybrid):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", precision)
    header = prefix / "include/btBulletCollisionCommon.h"
    header.write_text(
        header.read_text() + "inline int header_conversion(double x) { return x; }\n"
    )
    bullet_module(tmp_path)
    setup = bullet_imports(prefix).replace(
        "add_library(Bullet INTERFACE IMPORTED)", "add_library(Bullet INTERFACE)"
    )
    if hybrid:
        source = tmp_path / "bullet/source/collision.cpp"
        setup += (
            f'add_library(BulletCollision STATIC "{source.as_posix()}")\n'
            f'target_include_directories(BulletCollision PRIVATE "{prefix.as_posix()}/include")\n'
            + (
                "target_compile_definitions(BulletCollision PRIVATE BT_USE_DOUBLE_PRECISION)\n"
                if precision == "double"
                else ""
            )
            + 'set_property(TARGET Bullet PROPERTY INTERFACE_LINK_LIBRARIES BulletCollision "${math_file}")\n'
        )
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        # Per-code forced errors override warning suppression; exercise ordinary warnings.
        "if(MSVC)\nset(strict_flags /WX /W4)\n"
        "else()\nset(strict_flags -Werror -Wfloat-conversion)\nendif()\n"
        'string(JOIN " " CMAKE_CXX_FLAGS ${strict_flags})\n'
        + setup
        + "include(CheckCXXSourceCompiles)\n"
        f'set(CMAKE_REQUIRED_INCLUDES "{prefix.as_posix()}/include")\n'
        'check_cxx_source_compiles("#include <btBulletCollisionCommon.h>\nint main() { return 0; }" strict_control)\n'
        'if(strict_control)\nmessage(FATAL_ERROR "Header did not trigger strict warnings")\nendif()\n'
        'set(CMAKE_REQUIRED_FLAGS "caller flags")\n'
        "set(CMAKE_REQUIRED_DEFINITIONS -DCALLER_DEFINITION)\n"
        "set(CMAKE_REQUIRED_LIBRARIES caller_library)\n"
        "set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)\n"
        f"set(DART_BULLET_REQUIRE_PRECISION {precision})\ninclude(DARTFindBullet)\n"
        'if(NOT BULLET_FOUND OR NOT Bullet_FOUND)\nmessage(FATAL_ERROR "Header warnings rejected valid ABI")\nendif()\n'
        'if(NOT CMAKE_REQUIRED_FLAGS STREQUAL "caller flags"\n'
        '   OR NOT CMAKE_REQUIRED_DEFINITIONS STREQUAL "-DCALLER_DEFINITION"\n'
        '   OR NOT CMAKE_REQUIRED_LIBRARIES STREQUAL "caller_library"\n'
        f'   OR NOT CMAKE_REQUIRED_INCLUDES STREQUAL "{prefix.as_posix()}/include"\n'
        '   OR NOT CMAKE_TRY_COMPILE_TARGET_TYPE STREQUAL "STATIC_LIBRARY")\n'
        'message(FATAL_ERROR "Probe leaked check state")\nendif()\n'
        + f"set(DART_BULLET_REQUIRE_PRECISION {'single' if precision == 'double' else 'double'})\n"
        "include(DARTFindBullet)\n"
        'if(BULLET_FOUND OR Bullet_FOUND)\nmessage(FATAL_ERROR "Wrong ABI accepted under strict flags")\nendif()\n',
    )
    configure_log = tmp_path / "build/CMakeFiles/CMakeConfigureLog.yaml"
    assert result.returncode == 0, (
        result.stdout
        + result.stderr
        + (
            "\nCMake configure log:\n" + configure_log.read_text(errors="replace")
            if configure_log.is_file()
            else "\nCMake configure log was not produced.\n"
        )
    )


@pytest.mark.parametrize(
    "stem",
    [
        "LinearMath",
        "BulletMath",
        "LinearMath_d",
        "LinearMath_Debug",
        "LinearMath-float64",
        "LinearMath-float64-dbg",
    ],
)
def test_bullet_hybrid_abi_uses_library_basename(tmp_path, stem):
    prefix = build_bullet_abi_fixture(tmp_path / "imported", "double")
    original = prefix / "lib" / (prefix / "link-files.txt").read_text().splitlines()[1]
    filename = original.name.replace("LinearMath", stem)
    for directory in ("neutral", "BulletCollision-build"):
        case = tmp_path / directory
        case.mkdir()
        library = case / filename
        shutil.copy2(original, library)
        assert library.read_bytes() == original.read_bytes()
        bullet_module(case)
        result = configure(
            case,
            "set(DART_USE_SYSTEM_BULLET ON)\n"
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
            f'add_library(BulletCollision STATIC "{(tmp_path / "imported/source/collision.cpp").as_posix()}")\n'
            f'target_include_directories(BulletCollision PRIVATE "{prefix.as_posix()}/include")\n'
            "target_compile_definitions(BulletCollision PRIVATE BT_USE_DOUBLE_PRECISION)\n"
            "add_library(Bullet INTERFACE IMPORTED)\n"
            f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")\n'
            f'target_link_libraries(Bullet INTERFACE BulletCollision "{library.as_posix()}")\n'
            "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
            'if(NOT BULLET_FOUND OR NOT _dart_bullet_precision STREQUAL "double")\n'
            'message(FATAL_ERROR "Library ancestor changed hybrid ABI")\nendif()\n',
        )
        assert result.returncode == 0, result.stdout + result.stderr
        cmake_fixture_run(shutil.which("cmake"), "--build", case / "build")


def test_bullet_bundled_inherited_directory_precision(tmp_path):
    prefix = build_bullet_abi_fixture(tmp_path / "fixture", "double")
    source = tmp_path / "source"
    sub = source / "sub"
    sub.mkdir(parents=True)
    for name in ("collision.cpp", "math.cpp"):
        (sub / name).write_text(
            (tmp_path / "fixture/source" / name).read_text()
            + "static_assert(sizeof(btScalar) == sizeof(double));\n"
        )
    (sub / "CMakeLists.txt").write_text(
        "set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)\n"
        "add_library(BulletCollision SHARED collision.cpp)\n"
        "add_library(LinearMath SHARED math.cpp)\n"
        "foreach(t BulletCollision LinearMath)\n"
        f'target_include_directories(${{t}} PRIVATE "{prefix.as_posix()}/include")\n'
        "endforeach()\ntarget_link_libraries(BulletCollision PRIVATE LinearMath)\n"
    )
    (source / "backend.cmake").write_text(
        (ROOT / "dart/collision/bullet/CMakeLists.txt")
        .read_text()
        .split("# Search all header and source files")[0]
    )
    (source / "pkgconfig.cmake").write_text(
        (ROOT / "CMakeLists.txt")
        .read_text()
        .split("# Generate the DART pkg-config\n")[1]
        .split("# Install a Catkin")[0]
    )
    result = configure(
        tmp_path,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "set(DART_USE_SYSTEM_BULLET OFF)\n"
        f'set(DART_BULLET_SOURCE_DIR "{source.as_posix()}")\n'
        "add_definitions(-DBT_USE_DOUBLE_PRECISION)\nadd_subdirectory(sub)\n"
        "add_library(Bullet INTERFACE)\n"
        f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision LinearMath)\n"
        "include(DARTMacros)\ninclude(backend.cmake)\n"
        'if(NOT BULLET_FOUND OR NOT DART_BULLET_PRECISION STREQUAL "double"\n'
        '   OR NOT DART_BULLET_BUILT_PRECISION STREQUAL "double" OR NOT BT_USE_DOUBLE_PRECISION)\n'
        'message(FATAL_ERROR "Lost inherited bundled precision")\nendif()\n'
        "set(external_dependencies Bullet)\n"
        f'configure_file("{ROOT.as_posix()}/cmake/dart_Component.cmake.in" "${{CMAKE_BINARY_DIR}}/component.cmake" @ONLY)\n'
        f'set(DART_SOURCE_DIR "{ROOT.as_posix()}")\n'
        "set(DART_BINARY_DIR ${CMAKE_BINARY_DIR})\n"
        "set(CMAKE_INSTALL_LIBDIR lib)\ninclude(GNUInstallDirs)\n"
        "include(pkgconfig.cmake)\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
    component = (tmp_path / "build/component.cmake").read_text()
    assert 'set(DART_BULLET_REQUIRE_PRECISION "double")' in component
    pc = (tmp_path / "build/cmake/dart.pc").read_text()
    assert "-DBT_USE_DOUBLE_PRECISION" in pc.split("Cflags:")[1].splitlines()[0]


@pytest.mark.parametrize("debug_precision", ["single", "double"])
@pytest.mark.parametrize("imported_configs", ["Release;Debug", "RELEASE;DEBUG"])
def test_bullet_hybrid_configuration_abi(tmp_path, debug_precision, imported_configs):
    release = build_bullet_abi_fixture(tmp_path / "release", "double")
    debug = build_bullet_abi_fixture(tmp_path / "debug", debug_precision)
    bullet_module(tmp_path)
    (tmp_path / "consumer.cpp").write_text(BULLET_CONSUMER)
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\nset(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "set(CMAKE_CXX_STANDARD 17)\n"
        f'add_library(BulletCollision STATIC "{tmp_path.as_posix()}/release/source/collision.cpp")\n'
        f'target_include_directories(BulletCollision PUBLIC "{release.as_posix()}/include")\n'
        "target_compile_definitions(BulletCollision PUBLIC BT_USE_DOUBLE_PRECISION)\n"
        "add_library(math UNKNOWN IMPORTED)\n"
        f'find_library(release_math NAMES LinearMath PATHS "{release.as_posix()}/lib" NO_DEFAULT_PATH REQUIRED)\n'
        f'find_library(debug_math NAMES LinearMath PATHS "{debug.as_posix()}/lib" NO_DEFAULT_PATH REQUIRED)\n'
        f'set_target_properties(math PROPERTIES IMPORTED_CONFIGURATIONS "{imported_configs}" '
        'IMPORTED_LOCATION_RELEASE "${release_math}" IMPORTED_LOCATION_DEBUG "${debug_math}")\n'
        "target_link_libraries(BulletCollision PRIVATE math)\n"
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{release.as_posix()}/include")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision math)\n"
        "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        f'if({"NOT " if debug_precision == "double" else ""}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong hybrid per-configuration ABI acceptance")\nendif()\n'
        f'add_executable(consumer "{tmp_path.as_posix()}/consumer.cpp")\n'
        "target_link_libraries(consumer PRIVATE Bullet)\n",
        cmake_args=(
            "-G",
            "Ninja Multi-Config",
            "-DCMAKE_CONFIGURATION_TYPES=Release;Debug",
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if debug_precision == "double":
        for config in ("Release", "Debug"):
            cmake_fixture_run(
                shutil.which("cmake"), "--build", tmp_path / "build", "--config", config
            )


@pytest.mark.parametrize("configuration", ["multi", "Release", "Debug", ""])
@pytest.mark.parametrize("debug_precision", ["single", "double"])
@pytest.mark.parametrize(
    "edge",
    ["keywords", "raw-genex", "target-genex", "inactive-target", "mapped-target"],
)
def test_bullet_hybrid_conditional_graph_abi(
    tmp_path, debug_precision, edge, configuration
):
    release = build_bullet_abi_fixture(tmp_path / "release", "double")
    debug = build_bullet_abi_fixture(tmp_path / "debug", debug_precision)
    bullet_module(tmp_path)
    (tmp_path / "consumer.cpp").write_text(BULLET_CONSUMER)
    setup = ""
    if edge == "keywords":
        links = 'optimized "${release_math}" debug "${debug_math}"'
    elif edge == "raw-genex":
        links = '"$<$<NOT:$<CONFIG:Debug>>:${release_math}>" "$<$<CONFIG:Debug>:${debug_math}>"'
    elif edge == "mapped-target":
        setup = (
            "add_library(mapped INTERFACE IMPORTED)\n"
            "set_target_properties(mapped PROPERTIES MAP_IMPORTED_CONFIG_RELEASE RelWithDebInfo MAP_IMPORTED_CONFIG_DEBUG RelWithDebInfo)\n"
            'target_link_libraries(mapped INTERFACE "$<$<CONFIG:RelWithDebInfo>:${debug_math}>" "$<$<NOT:$<CONFIG:RelWithDebInfo>>:${release_math}>")\n'
        )
        links = "mapped"
    else:
        setup = (
            "add_library(release_math_target UNKNOWN IMPORTED)\n"
            "add_library(debug_math_target UNKNOWN IMPORTED)\n"
            'set_target_properties(release_math_target PROPERTIES IMPORTED_LOCATION "${release_math}")\n'
            'set_target_properties(debug_math_target PROPERTIES IMPORTED_LOCATION "${debug_math}")\n'
        )
        links = '"$<$<NOT:$<CONFIG:Debug>>:release_math_target>" "$<$<CONFIG:Debug>:debug_math_target>"'
        if edge == "inactive-target":
            setup += (
                "add_library(broken INTERFACE IMPORTED)\n"
                'target_link_libraries(broken INTERFACE "$<UNSUPPORTED:missing>")\n'
                'target_link_libraries(release_math_target INTERFACE "$<$<CONFIG:Debug>:broken>")\n'
            )
            links += ' "$<$<CONFIG:Unused>:broken>"'
    multi = configuration == "multi"
    configs = ("Release", "Debug") if multi else (configuration,)
    selected_debug = configuration == "Debug" or (
        edge == "mapped-target" and configuration != ""
    )
    accepted = debug_precision == "double" or (not multi and not selected_debug)
    pc_library = "debug_math" if selected_debug else "release_math"
    cmake_args = (
        ("-G", "Ninja Multi-Config", "-DCMAKE_CONFIGURATION_TYPES=Release;Debug")
        if multi
        else (f"-DCMAKE_BUILD_TYPE={configuration}", "-UCMAKE_CONFIGURATION_TYPES")
    )
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\nset(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "set(CMAKE_CXX_STANDARD 17)\n"
        f'add_library(BulletCollision STATIC "{tmp_path.as_posix()}/release/source/collision.cpp")\n'
        f'target_include_directories(BulletCollision PUBLIC "{release.as_posix()}/include")\n'
        "target_compile_definitions(BulletCollision PUBLIC BT_USE_DOUBLE_PRECISION)\n"
        f'find_library(release_math NAMES LinearMath PATHS "{release.as_posix()}/lib" NO_DEFAULT_PATH REQUIRED)\n'
        f'find_library(debug_math NAMES LinearMath PATHS "{debug.as_posix()}/lib" NO_DEFAULT_PATH REQUIRED)\n'
        + setup
        + f"target_link_libraries(BulletCollision PRIVATE {links})\n"
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{release.as_posix()}/include")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision)\n"
        "set(CMAKE_TRY_COMPILE_CONFIGURATION Release)\n"
        "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        f'if({"NOT " if accepted else ""}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong conditional graph ABI acceptance")\nendif()\n'
        'if(NOT CMAKE_TRY_COMPILE_CONFIGURATION STREQUAL "Release")\n'
        'message(FATAL_ERROR "Probe changed caller configuration")\nendif()\n'
        f'if(NOT DART_BULLET_PC_LIBS STREQUAL " ${{{pc_library}}}")\n'
        'message(FATAL_ERROR "Probe changed packaging libraries: ${DART_BULLET_PC_LIBS}")\nendif()\n'
        f'add_executable(consumer "{tmp_path.as_posix()}/consumer.cpp")\n'
        "target_link_libraries(consumer PRIVATE Bullet)\n",
        cmake_args=cmake_args,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if accepted:
        for config in configs:
            cmake_fixture_run(
                shutil.which("cmake"), "--build", tmp_path / "build", "--config", config
            )
            executable = (
                tmp_path
                / "build"
                / (config if multi else "")
                / ("consumer.exe" if os.name == "nt" else "consumer")
            )
            env = {
                **os.environ,
                "PATH": os.pathsep.join(
                    [
                        str(release / "lib"),
                        str(debug / "lib"),
                        os.environ.get("PATH", ""),
                    ]
                ),
            }
            cmake_fixture_run(executable, env=env)


@pytest.mark.parametrize("configuration", ["multi", "Release", "Debug", ""])
def test_bullet_active_unsupported_configuration_edge(tmp_path, configuration):
    include = bullet_module(tmp_path)
    active = "Release" if configuration == "multi" else configuration
    args = (
        ("-G", "Ninja Multi-Config", "-DCMAKE_CONFIGURATION_TYPES=Release;Debug")
        if configuration == "multi"
        else (f"-DCMAKE_BUILD_TYPE={configuration}", "-UCMAKE_CONFIGURATION_TYPES")
    )
    graph = (
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        f'target_link_libraries(Bullet INTERFACE "$<$<CONFIG:{active}>:$<UNSUPPORTED:missing>>")\n'
    )
    result = configure(
        tmp_path,
        graph + "include(DARTFindBullet)\n"
        "if(BULLET_FOUND OR NOT _DART_BULLET_GRAPH_UNKNOWN)\n"
        'message(FATAL_ERROR "Active unsupported edge was not rejected")\nendif()\n',
        cmake_args=args,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    # CMake itself must evaluate the expression in this configuration too.
    control = tmp_path / "control"
    control.mkdir()
    result = configure(
        control,
        graph
        + 'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/active-$<CONFIG>.txt" CONTENT "$<TARGET_GENEX_EVAL:Bullet,$<TARGET_PROPERTY:Bullet,INTERFACE_LINK_LIBRARIES>>")\n',
        cmake_args=args,
    )
    assert result.returncode != 0
    assert "UNSUPPORTED:missing" in result.stderr


@pytest.mark.parametrize(
    "mapping", ["", ";DEBUG", "DEBUG;;RELWITHDEBINFO", "DEBUG;", "DEBUG"]
)
@pytest.mark.parametrize("configuration", ["Release", ""])
@pytest.mark.parametrize("predicate", ["", "Debug"])
def test_bullet_mapped_configuration_predicates_match_cmake(
    tmp_path, mapping, configuration, predicate
):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double")
    bullet_module(tmp_path)
    (tmp_path / "consumer.cpp").write_text(BULLET_CONSUMER)
    graph = (
        "set(DART_USE_SYSTEM_BULLET ON)\nset(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + bullet_imports(prefix)
        + "target_compile_definitions(Bullet INTERFACE BT_USE_DOUBLE_PRECISION)\n"
        "add_library(owner INTERFACE IMPORTED)\n"
        "target_link_libraries(Bullet INTERFACE owner)\n"
        f'set_property(TARGET owner PROPERTY MAP_IMPORTED_CONFIG_{configuration.upper()} "{mapping}")\n'
        "add_library(broken INTERFACE IMPORTED)\n"
        'target_link_libraries(broken INTERFACE "$<UNSUPPORTED:missing>")\n'
        f'target_link_libraries(owner INTERFACE "$<$<CONFIG:{predicate}>:broken>")\n'
    )
    args = (f"-DCMAKE_BUILD_TYPE={configuration}", "-UCMAKE_CONFIGURATION_TYPES")
    control = tmp_path / "control"
    control.mkdir()
    result = configure(
        control,
        graph + f'add_executable(consumer "{tmp_path.as_posix()}/consumer.cpp")\n'
        "target_link_libraries(consumer PRIVATE Bullet)\n",
        cmake_args=args,
    )
    accepted = result.returncode == 0
    if accepted:
        cmake_fixture_run(shutil.which("cmake"), "--build", control / "build")
        executable = (
            control / "build" / ("consumer.exe" if os.name == "nt" else "consumer")
        )
        env = {
            **os.environ,
            "PATH": os.pathsep.join([str(prefix / "lib"), os.environ.get("PATH", "")]),
        }
        cmake_fixture_run(executable, env=env)
    else:
        assert "UNSUPPORTED:missing" in result.stderr
    result = configure(
        tmp_path,
        graph + "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/finder.txt" "${BULLET_FOUND};${_DART_BULLET_GRAPH_UNKNOWN}")\n',
        cmake_args=args,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (tmp_path / "build/finder.txt").read_text() == (
        "TRUE;FALSE" if accepted else "FALSE;TRUE"
    )


@pytest.mark.parametrize("debug_precision", ["single", "double"])
@pytest.mark.parametrize("generator", ["Ninja", "Ninja Multi-Config"])
def test_bullet_imported_configuration_abi(tmp_path, debug_precision, generator):
    release = build_bullet_abi_fixture(tmp_path / "release", "double")
    debug = build_bullet_abi_fixture(tmp_path / "debug", debug_precision)
    bullet_module(tmp_path)
    (tmp_path / "consumer.cpp").write_text(BULLET_CONSUMER)
    # A decorative configuration list does not turn Ninja into a multi-config generator.
    accepted = debug_precision == "double" or generator == "Ninja"
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\nset(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "set(CMAKE_BUILD_TYPE Release)\nadd_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{release.as_posix()}/include")\n'
        "foreach(name BulletCollision LinearMath)\nadd_library(${name} UNKNOWN IMPORTED)\n"
        f'find_library(release_${{name}} NAMES ${{name}} PATHS "{release.as_posix()}/lib" NO_DEFAULT_PATH REQUIRED)\n'
        f'find_library(debug_${{name}} NAMES ${{name}} PATHS "{debug.as_posix()}/lib" NO_DEFAULT_PATH REQUIRED)\n'
        'set_target_properties(${name} PROPERTIES IMPORTED_CONFIGURATIONS "RELEASE;DEBUG" '
        'IMPORTED_LOCATION_RELEASE "${release_${name}}" IMPORTED_LOCATION_DEBUG "${debug_${name}}")\n'
        "target_link_libraries(Bullet INTERFACE ${name})\nendforeach()\n"
        "target_compile_definitions(Bullet INTERFACE BT_USE_DOUBLE_PRECISION)\n"
        "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        f'if({"NOT " if accepted else ""}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong per-configuration ABI acceptance")\nendif()\n'
        f'add_executable(consumer "{tmp_path.as_posix()}/consumer.cpp")\n'
        "target_link_libraries(consumer PRIVATE Bullet)\n",
        cmake_args=("-G", generator, "-DCMAKE_CONFIGURATION_TYPES=Release;Debug"),
    )
    assert result.returncode == 0, result.stdout + result.stderr

    if accepted:
        configs = (
            ("Release", "Debug") if generator == "Ninja Multi-Config" else ("Release",)
        )
        for config in configs:
            cmake_fixture_run(
                shutil.which("cmake"), "--build", tmp_path / "build", "--config", config
            )
            executable = (
                tmp_path
                / "build"
                / (config if generator == "Ninja Multi-Config" else "")
                / ("consumer.exe" if os.name == "nt" else "consumer")
            )
            env = {
                **os.environ,
                "PATH": os.pathsep.join(
                    [
                        str(release / "lib"),
                        str(debug / "lib"),
                        os.environ.get("PATH", ""),
                    ]
                ),
            }
            cmake_fixture_run(executable, env=env)


@pytest.mark.parametrize("mode", ["ambiguous", "broken"])
def test_bullet_scalar_abi_requires_unique_link_success(tmp_path, mode):
    prefix = build_bullet_abi_fixture(tmp_path / "single", "single", "STATIC")
    other = build_bullet_abi_fixture(tmp_path / "double", "double", "STATIC")
    bullet_module(tmp_path)
    extra = ""
    if mode == "ambiguous":
        extra = f'file(GLOB extra "{other.as_posix()}/lib/*")\ntarget_link_libraries(Bullet INTERFACE ${{extra}})\n'
    else:
        for library in (prefix / "lib").iterdir():
            library.write_text("This is not a library.\n")
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\nset(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + bullet_imports(prefix)
        + extra
        + "set(DART_BULLET_REQUIRE_PRECISION single)\ninclude(DARTFindBullet)\n"
        'if(BULLET_FOUND OR Bullet_FOUND)\nmessage(FATAL_ERROR "Unproven ABI accepted")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "scalar ABI (empty means unverified)" in " ".join(result.stderr.split())


def test_bullet_imported_abi_with_unrelated_built_helper(tmp_path):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double")
    bullet_module(tmp_path)
    (tmp_path / "helper.cpp").write_text("void helper() {}\n")
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + bullet_imports(prefix)
        + f'add_library(helper STATIC "{tmp_path.as_posix()}/helper.cpp")\n'
        "target_link_libraries(Bullet INTERFACE helper)\n"
        "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        'if(NOT BULLET_FOUND)\nmessage(FATAL_ERROR "Unrelated helper prevented ABI validation")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_bullet_interface_definition_is_not_built_abi(tmp_path):
    include = bullet_module(tmp_path)
    (tmp_path / "single.cpp").write_text(
        "#ifdef BT_USE_DOUBLE_PRECISION\n#error This fixture is single precision\n#endif\nvoid single(float) {}\n"
    )
    result = configure(
        tmp_path,
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_library(BulletCollision STATIC "{tmp_path.as_posix()}/single.cpp")\n'
        "target_compile_definitions(BulletCollision INTERFACE BT_USE_DOUBLE_PRECISION)\n"
        "add_library(Bullet INTERFACE)\ntarget_link_libraries(Bullet INTERFACE BulletCollision)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        "include(DARTFindBullet)\n"
        'if(BULLET_FOUND)\nmessage(FATAL_ERROR "Interface-only definition misclassified the implementation")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")


@pytest.mark.parametrize("fallback", [False, True])
def test_bullet_pkgconfig_unsupported_sysroot_only_blocks_fallback(tmp_path, fallback):
    include = bullet_module(tmp_path)
    pc_block = (
        (ROOT / "CMakeLists.txt")
        .read_text()
        .split("# Generate the DART pkg-config\n")[1]
        .split("if(HAVE_ODE)")[0]
    )
    suffix = "-float64" if fallback else ""
    result = configure(
        tmp_path,
        "set(CMAKE_SYSTEM_NAME Darwin)\nset(CMAKE_SYSROOT /builder/root)\n"
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        f'target_link_libraries(Bullet INTERFACE "/builder/root/lib/libBulletCollision{suffix}.dylib")\n'
        "include(DARTFindBullet)\nset(HAVE_BULLET TRUE)\n" + pc_block,
    )
    assert (result.returncode != 0) == fallback, result.stdout + result.stderr
    if fallback:
        assert "Cannot serialize the selected Bullet library" in result.stderr


def test_bullet_selected_headers_cannot_reuse_discovery_version(tmp_path):
    bullet_module(tmp_path)
    result = configure(
        tmp_path,
        "add_library(Bullet INTERFACE IMPORTED)\n"
        "target_link_libraries(Bullet INTERFACE libBulletCollision.so)\n"
        "set(Bullet_VERSION 99.0)\nset(BULLET_VERSION 99.0 CACHE STRING old)\n"
        "set(DART_BULLET_REQUIRE_VERSION 3.17)\ninclude(DARTFindBullet)\n"
        'if(BULLET_FOUND OR Bullet_FOUND)\nmessage(FATAL_ERROR "Reused unrelated version metadata")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    "inner,accepted",
    [
        ("$<INSTALL_INTERFACE:/unused>", True),
        ("$<TARGET_PROPERTY:unknown,INTERFACE_INCLUDE_DIRECTORIES>", False),
    ],
)
def test_bullet_nested_include_expression(tmp_path, inner, accepted):
    include = bullet_module(tmp_path)
    result = configure(
        tmp_path,
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}" "$<BUILD_INTERFACE:{inner}>")\n'
        "target_link_libraries(Bullet INTERFACE libBulletCollision-float64.so)\n"
        "include(DARTFindBullet)\n"
        f'if({"NOT " if accepted else ""}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong nested include acceptance")\nendif()\n',
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


@pytest.mark.parametrize(
    "configuration,layout,provider",
    [
        ("Release", "plain", "directories"),
        ("Debug", "plain", "directories"),
        ("Release", "spaces", "directories"),
        ("Debug", "spaces", "directories"),
        ("Release", "plain", "options"),
        ("Debug", "plain", "options"),
        ("Release", "spaces", "options"),
        ("Debug", "spaces", "options"),
        ("Release", "spaces", "precedence"),
        ("Release", "spaces", "normal-directories"),
        ("Debug", "spaces", "normal-options"),
        ("Debug", "spaces", "mapped-directories"),
    ],
)
def test_bullet_named_libraries_pkgconfig_consumer(
    tmp_path, configuration, layout, provider
):
    normal = provider.startswith("normal-")
    provider = provider.removeprefix("normal-")
    folder = "bullet with spaces" if layout == "spaces" else "bullet"
    prefix = build_bullet_abi_fixture(tmp_path / folder, "double", "STATIC", "-float64")
    other = None
    if provider == "precedence":
        other = build_bullet_abi_fixture(
            tmp_path / "competing single SDK", "single", "STATIC", "-float64"
        )
    bullet_module(tmp_path)
    source = tmp_path / "consumer.cpp"
    source.write_text(BULLET_CONSUMER)
    graph = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f"add_library(Bullet INTERFACE{'' if normal else ' IMPORTED'})\n"
        f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")\n'
    )
    owner = "Bullet"
    if (
        configuration == "Debug" and provider == "options"
    ) or provider == "mapped-directories":
        owner = "usage_owner"
        graph += (
            "add_library(usage_owner INTERFACE IMPORTED)\n"
            "set_property(TARGET usage_owner PROPERTY MAP_IMPORTED_CONFIG_DEBUG Release)\n"
            "target_link_libraries(Bullet INTERFACE usage_owner)\n"
        )
    if provider in ("directories", "mapped-directories"):
        active = "Release" if provider == "mapped-directories" else configuration
        graph += (
            f'target_link_directories({owner} INTERFACE "$<$<CONFIG:{active}>:{prefix.as_posix()}/lib>" '
            '"$<$<CONFIG:Unused>:/missing/bullet>")\n'
        )
    else:
        graph += (
            (
                "if(MSVC)\nset(search_prefix -lIbPaTh:)\nelse()\nset(search_prefix -L)\nendif()\n"
                if configuration == "Debug"
                else "if(MSVC)\nset(search_prefix /LIBPATH:)\nelse()\nset(search_prefix -L)\nendif()\n"
            )
            + f'target_link_options({owner} INTERFACE "$<$<CONFIG:Release>:${{search_prefix}}{prefix.as_posix()}/lib>" '
            '"$<$<CONFIG:Unused>:SHELL:unsupported-unused-option>")\n'
        )
        if other:
            graph += f'target_link_directories(Bullet INTERFACE "{other.as_posix()}/lib" "{prefix.as_posix()}/lib")\n'
    graph += (
        "target_link_libraries(Bullet INTERFACE BulletCollision-float64 LinearMath-float64)\n"
        f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
        f"get_property(before_options TARGET {owner} PROPERTY INTERFACE_LINK_OPTIONS)\n"
        'set(CMAKE_REQUIRED_FLAGS "caller-flags")\n'
        'set(CMAKE_REQUIRED_LIBRARIES "caller-library")\n'
    )
    hook = tmp_path / "project-hook.cmake"
    hook.write_text(
        "# Harmless caller hook; the selected CXX compiler is already initialized.\n"
    )
    graph += (
        f'set(CMAKE_PROJECT_INCLUDE "{hook.as_posix()}")\n'
        'set(before_compiler "${CMAKE_CXX_COMPILER}")\n'
        "get_property(before_languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
    )
    if normal or provider == "mapped-directories":
        # Establish the ordinary result before the independent scalar ABI probe.
        control = tmp_path / "ordinary-control"
        control.mkdir()
        result = configure(
            control, graph, cmake_args=(f"-DCMAKE_BUILD_TYPE={configuration}",)
        )
        assert result.returncode == 0, result.stdout + result.stderr
        cmake_fixture_run(shutil.which("cmake"), "--build", control / "build")
        cmake_fixture_run(control / "build/consumer")
    result = configure(
        tmp_path,
        graph + "include(DARTFindBullet)\n_dart_bullet_probe_precision(precision)\n"
        'if(NOT BULLET_FOUND OR NOT precision STREQUAL "double")\n'
        'message(FATAL_ERROR "Named libraries failed ABI validation")\nendif()\n'
        'if(DART_BULLET_PC_ERROR)\nmessage(FATAL_ERROR "${DART_BULLET_PC_ERROR}")\nendif()\n'
        f"get_property(after_options TARGET {owner} PROPERTY INTERFACE_LINK_OPTIONS)\n"
        'if(NOT "${before_options}" STREQUAL "${after_options}" OR NOT CMAKE_REQUIRED_FLAGS STREQUAL "caller-flags" OR NOT CMAKE_REQUIRED_LIBRARIES STREQUAL "caller-library")\n'
        'message(FATAL_ERROR "Finder changed caller state or target options")\nendif()\n'
        "get_property(after_languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        f'if(NOT CMAKE_PROJECT_INCLUDE STREQUAL "{hook.as_posix()}" OR NOT CMAKE_CXX_COMPILER STREQUAL before_compiler OR NOT "${{before_languages}}" STREQUAL "${{after_languages}}")\n'
        'message(FATAL_ERROR "Finder changed caller hook/compiler/languages")\nendif()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/bullet.pc" "Name: bullet-fixture\\nDescription: Bullet fixture\\nVersion: 1\\nCflags: -DBT_USE_DOUBLE_PRECISION${DART_BULLET_PC_CFLAGS}\\nLibs:${DART_BULLET_PC_LIBS}\\n")\n',
        cmake_args=(f"-DCMAKE_BUILD_TYPE={configuration}",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
    cmake_fixture_run(tmp_path / "build/consumer")
    env = {
        **os.environ,
        "PKG_CONFIG_PATH": str(tmp_path / "build"),
        "PKG_CONFIG_LIBDIR": str(tmp_path / "build"),
    }
    env.pop("PKG_CONFIG_SYSROOT_DIR", None)
    options, flags = pkgconfig_consumer_options("bullet", env=env)
    assert "-lBulletCollision-float64" in flags
    assert "-lLinearMath-float64" in flags
    assert "-L" + (prefix / "lib").as_posix() in flags
    assert all("/missing/bullet" not in flag for flag in flags)
    if other:
        wanted = "-L" + (prefix / "lib").as_posix()
        competing = "-L" + (other / "lib").as_posix()
        assert flags.index(wanted) < flags.index(competing)
        assert flags.count(wanted) == 1
    consumer = tmp_path / "pc-consumer"
    consumer.mkdir()
    result = configure(
        consumer,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_executable(consumer "{source.as_posix()}")\n' + options,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", consumer / "build")
    cmake_fixture_run(consumer / "build/consumer")
    if other:
        # Reversing actual search priority must select the incompatible archive.
        negative = tmp_path / "wrong-priority"
        negative.mkdir()
        result = configure(
            negative,
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
            f'add_executable(consumer "{source.as_posix()}")\n'
            f'target_link_directories(consumer PRIVATE "{other.as_posix()}/lib")\n'
            + options,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        result = subprocess.run(
            [shutil.which("cmake"), "--build", str(negative / "build")],
            capture_output=True,
            text=True,
        )
        assert result.returncode != 0, result.stdout + result.stderr
        assert "btStaticPlaneShape" in result.stdout + result.stderr


@pytest.mark.parametrize(
    "kind", ["shell", "linker", "genex", "relative", "split", "quoted", "macro"]
)
def test_bullet_link_options_only_block_pkgconfig_fallback(tmp_path, monkeypatch, kind):
    prefix = build_bullet_abi_fixture(
        tmp_path / "bullet", "double", "STATIC", "-float64"
    )
    bullet_module(tmp_path)
    source = tmp_path / "consumer.cpp"
    source.write_text(BULLET_CONSUMER)
    forms = {
        "shell": '"SHELL:${search_prefix}\\"${sdk_lib}\\""',
        "linker": '"LINKER:${search_prefix}${sdk_lib}"',
        "genex": '"$<IF:$<BOOL:1>,${search_prefix}${sdk_lib},>"',
        "relative": '"${search_prefix}relative-bullet"',
        "split": '"${search_prefix}" "${sdk_lib}"',
        "quoted": '"${search_prefix}\\"${sdk_lib}\\""',
        "macro": '"${search_prefix}$SYSROOT/fixture"',
    }
    graph = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + bullet_imports(prefix, "-float64")
        + "if(MSVC)\nset(search_prefix /LIBPATH:)\nelse()\nset(search_prefix -L)\nendif()\n"
        f'set(sdk_lib "{prefix.as_posix()}/lib")\n'
        f"target_link_options(Bullet INTERFACE {forms[kind]})\n"
        "get_target_property(before_options Bullet INTERFACE_LINK_OPTIONS)\n"
        f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    # Split /LIBPATH: is malformed native syntax; its packaging check stands alone.
    ordinary_valid = not (os.name == "nt" and kind == "split")
    result = configure(
        tmp_path,
        graph + "include(DARTFindBullet)\n"
        "if(NOT BULLET_FOUND OR _DART_BULLET_GRAPH_UNKNOWN OR NOT DART_BULLET_PC_ERROR)\n"
        'message(FATAL_ERROR "Packaging error changed ABI/discovery or was omitted")\nendif()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/metadata-error.txt" "${DART_BULLET_PC_ERROR}")\n'
        "get_target_property(after_options Bullet INTERFACE_LINK_OPTIONS)\n"
        'if(NOT "${before_options}" STREQUAL "${after_options}")\nmessage(FATAL_ERROR "Finder changed original options")\nendif()\n'
        + (
            "_dart_bullet_probe_precision(precision)\n"
            'if(NOT precision STREQUAL "double")\nmessage(FATAL_ERROR "Valid ordinary target lost scalar ABI")\nendif()\n'
            if ordinary_valid
            else ""
        ),
        cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    error = (tmp_path / "build/metadata-error.txt").read_text()
    assert "Bullet" in error and "Release" in error
    if ordinary_valid:
        cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
        cmake_fixture_run(tmp_path / "build/consumer")
    pc_block = (
        (ROOT / "CMakeLists.txt")
        .read_text()
        .split("# Generate the DART pkg-config\n")[1]
        .split("if(HAVE_ODE)")[0]
    )
    for available in (False, True) if kind == "shell" else (False,):
        case = tmp_path / ("real-module" if available else "fallback")
        case.mkdir()
        pcdir = case / "pkgconfig"
        pcdir.mkdir()
        if available:
            path = prefix.as_posix().replace(" ", "\\ ")
            (pcdir / "bullet-float64.pc").write_text(
                "Name: Bullet float64 fixture\nDescription: real scalar SDK\nVersion: 3.25\n"
                f"Cflags: -DBT_USE_DOUBLE_PRECISION -I{path}/include\n"
                f"Libs: -L{path}/lib -lBulletCollision-float64 -lLinearMath-float64\n"
            )
        env = {
            **os.environ,
            "PKG_CONFIG_PATH": str(pcdir),
            "PKG_CONFIG_LIBDIR": str(pcdir),
        }
        env.pop("PKG_CONFIG_SYSROOT_DIR", None)
        exists = subprocess.run(
            [pkgconfig_fixture_executable(), "--exists", "bullet-float64"],
            capture_output=True,
            text=True,
            env=env,
        )
        assert (exists.returncode == 0) == available, exists.stdout + exists.stderr
        for name in ("PKG_CONFIG_PATH", "PKG_CONFIG_LIBDIR"):
            monkeypatch.setenv(name, env[name])
        monkeypatch.delenv("PKG_CONFIG_SYSROOT_DIR", raising=False)
        bullet_module(case)
        result = configure(
            case,
            graph + "include(DARTFindBullet)\nset(HAVE_BULLET TRUE)\n"
            "set(CMAKE_DISABLE_FIND_PACKAGE_PkgConfig FALSE)\n"
            f'set(PKG_CONFIG_EXECUTABLE "{Path(pkgconfig_fixture_executable()).as_posix()}")\n'
            + pc_block
            + 'if(NOT BULLET_FLOAT64_FOUND OR NOT DART_BULLET_PC_ERROR)\nmessage(FATAL_ERROR "Real module did not bypass only fallback serialization")\nendif()\n',
            cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
        )
        assert (result.returncode == 0) == available, result.stdout + result.stderr
        if not available:
            assert " ".join(error.split()) in " ".join(
                (result.stdout + result.stderr).split()
            )
        else:
            options, _ = pkgconfig_consumer_options("bullet-float64", env=env)
            consumer = case / "pc-consumer"
            consumer.mkdir()
            result = configure(
                consumer,
                "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
                f'add_executable(consumer "{source.as_posix()}")\n' + options,
            )
            assert result.returncode == 0, result.stdout + result.stderr
            cmake_fixture_run(shutil.which("cmake"), "--build", consumer / "build")
            cmake_fixture_run(consumer / "build/consumer")


@pytest.mark.parametrize(
    "mode", ["unset", "empty", "notfound-suffix", "inactive-config"]
)
def test_bullet_link_option_property_and_error_state(tmp_path, mode):
    prefix = build_bullet_abi_fixture(
        tmp_path / "bullet", "double", "STATIC", "-float64"
    )
    bullet_module(tmp_path)
    source = tmp_path / "consumer.cpp"
    source.write_text(BULLET_CONSUMER)
    directory = tmp_path / "search-directory-NOTFOUND"
    directory.mkdir()
    replacement = {
        "unset": "set_property(TARGET Bullet PROPERTY INTERFACE_LINK_OPTIONS)\n",
        "empty": 'set_property(TARGET Bullet PROPERTY INTERFACE_LINK_OPTIONS "")\n',
        "notfound-suffix": f'set_property(TARGET Bullet PROPERTY INTERFACE_LINK_OPTIONS "${{search_prefix}}{directory.as_posix()}")\n',
        "inactive-config": 'set_property(TARGET Bullet PROPERTY INTERFACE_LINK_OPTIONS "$<$<CONFIG:Debug>:SHELL:${search_prefix}\\"${sdk_lib}\\">")\n',
    }
    graph = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + bullet_imports(prefix, "-float64")
        + "if(MSVC)\nset(search_prefix /LIBPATH:)\nelse()\nset(search_prefix -L)\nendif()\n"
        f'set(sdk_lib "{prefix.as_posix()}/lib")\n'
        'set_property(TARGET Bullet PROPERTY INTERFACE_LINK_OPTIONS "SHELL:${search_prefix}\\"${sdk_lib}\\"")\n'
        "include(DARTFindBullet)\n"
        'if(NOT DART_BULLET_PC_ERROR OR NOT BULLET_FOUND OR _DART_BULLET_GRAPH_UNKNOWN)\nmessage(FATAL_ERROR "Missing initial metadata-only error")\nendif()\n'
        + replacement[mode]
        + 'set(_dart_bullet_options "SHELL:cache-poison" CACHE STRING "" FORCE)\n'
        'set(_dart_bullet_value "SHELL:cache-poison" CACHE STRING "" FORCE)\n'
        "include(DARTFindBullet)\n_dart_bullet_probe_precision(precision)\n"
        'if(DART_BULLET_PC_ERROR OR NOT BULLET_FOUND OR _DART_BULLET_GRAPH_UNKNOWN OR NOT precision STREQUAL "double")\nmessage(FATAL_ERROR "Inactive/unset option or stale metadata poisoned selected ABI")\nendif()\n'
        "get_property(options_set TARGET Bullet PROPERTY INTERFACE_LINK_OPTIONS SET)\n"
        f'if(NOT options_set STREQUAL "{0 if mode == "unset" else 1}")\nmessage(FATAL_ERROR "Option SET state changed")\nendif()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/metadata.txt" "${DART_BULLET_PC_LIBS}")\n'
        f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    result = configure(tmp_path, graph, cmake_args=("-DCMAKE_BUILD_TYPE=Release",))
    assert result.returncode == 0, result.stdout + result.stderr
    if mode == "notfound-suffix":
        assert "-L" + directory.as_posix() in shlex.split(
            (tmp_path / "build/metadata.txt").read_text()
        )
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
    cmake_fixture_run(tmp_path / "build/consumer")
    if mode == "inactive-config":
        # The unused Debug packaging error must not survive the final Release walk.
        multi = tmp_path / "multi"
        cmake_fixture_run(
            shutil.which("cmake"),
            "-G",
            "Ninja Multi-Config",
            "-S",
            tmp_path / "source",
            "-B",
            multi,
            "-DCMAKE_BUILD_TYPE=Release",
        )
        cmake_fixture_run(
            shutil.which("cmake"), "--build", multi, "--config", "Release"
        )
        cmake_fixture_run(multi / "Release/consumer")


@pytest.mark.parametrize(
    "mode", ["bare", "flag", "absolute", "wrapper", "mapped", "whole"]
)
def test_bullet_pkgconfig_deferred_raw_dependency(tmp_path, mode):
    if mode == "whole" and sys.platform != "linux":
        pytest.skip("GNU whole-archive constructor control")
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double", "STATIC")
    fixture = tmp_path / "archives"
    fixture.mkdir()
    (fixture / "a.cpp").write_text(
        'extern "C" int raw_tail();\nextern "C" int raw_a() { return raw_tail(); }\n'
    )
    (fixture / "b.cpp").write_text(
        'extern "C" int raw_a();\nextern "C" int raw_b() { return raw_a(); }\n'
    )
    (fixture / "tail.cpp").write_text('extern "C" int raw_tail() { return 0; }\n')
    (fixture / "registration.cpp").write_text(
        "extern int registered;\nnamespace { struct Registration {\n"
        "Registration() { registered = 17; }\n} registration; }\n"
    )
    (fixture / "marker.cpp").write_text("int registered = 0;\n")
    names = ("RawA", "RawB", "RawTail", "Registration", "Marker")
    (fixture / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\nproject(RawArchives LANGUAGES CXX)\n"
        "add_library(RawA STATIC a.cpp)\nadd_library(RawB STATIC b.cpp)\n"
        "add_library(RawTail STATIC tail.cpp)\nadd_library(Registration STATIC registration.cpp)\n"
        "add_library(Marker STATIC marker.cpp)\n"
        'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/archives.txt" CONTENT "'
        + "\\n".join(f"$<TARGET_FILE:{name}>" for name in names)
        + '\\n")\n'
    )
    cmake_fixture_run(
        shutil.which("cmake"), "-G", "Ninja", "-S", fixture, "-B", fixture / "build"
    )
    cmake_fixture_run(shutil.which("cmake"), "--build", fixture / "build")
    archives = dict(
        zip(names, (fixture / "build/archives.txt").read_text().splitlines())
    )
    raw = (
        Path(archives["RawTail"]).as_posix()
        if mode == "absolute"
        else "${raw_tail_flag}" if mode == "flag" else "RawTail"
    )
    if mode == "whole":
        raw += f' -Wl,--whole-archive "{Path(archives["Registration"]).as_posix()}" -Wl,--no-whole-archive'
    graph = bullet_imports(prefix)
    if mode == "flag":
        graph += (
            "if(MSVC)\nset(raw_tail_flag -DEFAULTLIB:RawTail.lib)\n"
            "else()\nset(raw_tail_flag -lRawTail)\nendif()\n"
        )
    for name in ("RawA", "RawB", *(["Marker"] if mode == "whole" else [])):
        graph += f'add_library({name} STATIC IMPORTED)\nset_property(TARGET {name} PROPERTY IMPORTED_LOCATION "{Path(archives[name]).as_posix()}")\n'
    if mode == "wrapper":
        graph += "add_library(raw_wrapper INTERFACE IMPORTED)\n"
        graph += f"target_link_libraries(raw_wrapper INTERFACE {raw})\n"
        raw = "raw_wrapper"
    elif mode == "mapped":
        raw = f'"$<$<CONFIG:Release>:{raw}>" "$<$<CONFIG:Debug>:missing_raw_tail>"'
    graph += (
        f"target_link_libraries(RawA INTERFACE {raw})\n"
        "target_link_libraries(RawB INTERFACE RawA)\n"
        "target_link_libraries(Bullet INTERFACE RawA RawB)\n"
        f'target_link_directories(Bullet INTERFACE "{Path(archives["RawTail"]).parent.as_posix()}")\n'
    )
    if mode == "whole":
        graph += "target_link_libraries(RawA INTERFACE Marker)\n"
    source = tmp_path / "consumer.cpp"
    source.write_text(
        'extern "C" int raw_b();\n'
        + ("extern int registered;\n" if mode == "whole" else "")
        + BULLET_CONSUMER.replace(
            "int main() {",
            "int main() { if (raw_b()) return 2;"
            + (" if (registered != 17) return 17;" if mode == "whole" else ""),
        )
    )
    common = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + graph
        + f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    control = tmp_path / "control"
    control.mkdir()
    result = configure(control, common, cmake_args=("-DCMAKE_BUILD_TYPE=Release",))
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", control / "build")
    cmake_fixture_run(control / "build/consumer")
    bullet_module(tmp_path)
    result = configure(
        tmp_path,
        common + "include(DARTFindBullet)\n_dart_bullet_probe_precision(precision)\n"
        'if(NOT BULLET_FOUND OR NOT precision STREQUAL "double")\nmessage(FATAL_ERROR "Raw dependency failed ABI validation")\nendif()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/bullet.pc" "Name: bullet\nDescription: raw dependency\nVersion: 1\nCflags: -DBT_USE_DOUBLE_PRECISION${DART_BULLET_PC_CFLAGS}\nLibs: ${DART_BULLET_PC_LIBS}\n")\n',
        cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    env = {
        **os.environ,
        "PKG_CONFIG_PATH": str(tmp_path / "build"),
        "PKG_CONFIG_LIBDIR": str(tmp_path / "build"),
    }
    env.pop("PKG_CONFIG_SYSROOT_DIR", None)
    options, flags = pkgconfig_consumer_options("bullet", static=True, env=env)
    consumer = tmp_path / "pc-consumer"
    consumer.mkdir()
    result = configure(
        consumer,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_executable(consumer "{source.as_posix()}")\n' + options,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", consumer / "build")
    cmake_fixture_run(consumer / "build/consumer")
    if sys.platform == "linux":
        # A flat native link ensures CMake cannot repair the emitted graph.
        command = [
            shutil.which("c++"),
            str(source),
            *flags,
            "-o",
            str(tmp_path / "native"),
        ]
        cmake_fixture_run(*command)
        cmake_fixture_run(tmp_path / "native")
        broken = flags.copy()
        tail = archives["RawTail"] if mode == "absolute" else "-lRawTail"
        broken.remove(tail)
        broken.insert(broken.index(archives["RawB"]), tail)
        negative = subprocess.run(
            [*command[:2], *broken, "-o", str(tmp_path / "negative")],
            capture_output=True,
            text=True,
        )
        assert negative.returncode != 0, negative.stdout + negative.stderr
        assert "raw_tail" in negative.stdout + negative.stderr
        if mode == "whole":
            broken = [
                flag
                for flag in flags
                if flag not in ("-Wl,--whole-archive", "-Wl,--no-whole-archive")
            ]
            cmake_fixture_run(
                *command[:2], *broken, "-o", tmp_path / "without-registration"
            )
            assert (
                subprocess.run([str(tmp_path / "without-registration")]).returncode
                == 17
            )


@pytest.mark.parametrize(
    "mode",
    [
        "default",
        "early-tail",
        "early-next",
        "acyclic",
        "diamond",
        "two-cycles",
        "three",
        "mapped",
        "fallback",
        "mapped-zero",
        "legacy",
        "max",
        "reverse",
        "transparent",
        "inactive",
    ],
)
def test_bullet_pkgconfig_static_cycles(tmp_path, mode):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double", "STATIC")
    scans = (
        3
        if mode in ("three", "mapped", "fallback", "legacy")
        else 4 if mode == "max" else 2
    )
    needed = 3 if scans > 2 else 2
    order = ["CycleB", "CycleA"] if mode == "reverse" else ["CycleA", "CycleB"]
    chain = [order[i % 2] for i in range(2 * needed - 1)]
    if mode == "inactive":
        scans = 1
        chain = order
    if mode in ("acyclic", "diamond"):
        scans = 1
        chain = order if mode == "acyclic" else [order[0]]
    fixture = tmp_path / "cycles"
    fixture.mkdir()
    sources = {"CycleA": [], "CycleB": [], "Tail": ["tail.cpp"]}
    (fixture / "tail.cpp").write_text('extern "C" int cycle_tail() { return 0; }\n')
    transitive_tail = mode in ("early-next", "acyclic", "diamond")
    if transitive_tail:
        sources["Next"] = ["next.cpp"]
        (fixture / "tail.cpp").write_text(
            'extern "C" int final_tail();\nextern "C" int cycle_tail() { return final_tail(); }\n'
        )
        (fixture / "next.cpp").write_text('extern "C" int final_tail() { return 0; }\n')
    if mode == "diamond":
        sources[order[1]] = ["diamond.cpp"]
        (fixture / "diamond.cpp").write_text(
            'extern "C" int cycle_tail();\nextern "C" int diamond_step() { return cycle_tail(); }\n'
        )
    for i, library in enumerate(chain):
        name = f"step{i}.cpp"
        following = f"cycle_step{i + 1}" if i + 1 < len(chain) else "cycle_tail"
        (fixture / name).write_text(
            f'extern "C" int {following}();\nextern "C" int cycle_step{i}() {{ return {following}(); }}\n'
        )
        sources[library].append(name)
    if mode == "two-cycles":
        sources.update(CycleC=["other0.cpp", "other2.cpp"], CycleD=["other1.cpp"])
        for i in range(3):
            following = f"other_step{i + 1}" if i < 2 else "cycle_tail"
            (fixture / f"other{i}.cpp").write_text(
                f'extern "C" int {following}();\nextern "C" int other_step{i}() {{ return {following}(); }}\n'
            )
    (fixture / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\nproject(Cycles LANGUAGES CXX)\n"
        + "".join(
            f"add_library({name} STATIC {' '.join(files)})\n"
            for name, files in sources.items()
        )
        + 'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/archives.txt" CONTENT "'
        + "\\n".join(f"$<TARGET_FILE:{name}>" for name in sources)
        + '\\n")\n'
    )
    cmake_fixture_run(
        shutil.which("cmake"), "-G", "Ninja", "-S", fixture, "-B", fixture / "build"
    )
    cmake_fixture_run(shutil.which("cmake"), "--build", fixture / "build")
    archives = dict(
        zip(sources, (fixture / "build/archives.txt").read_text().splitlines())
    )
    graph = bullet_imports(prefix)
    for name, archive in archives.items():
        graph += f'add_library({name} STATIC IMPORTED)\nset_property(TARGET {name} PROPERTY IMPORTED_LOCATION "{Path(archive).as_posix()}")\n'
    if mode == "two-cycles":
        graph += "target_link_libraries(CycleC INTERFACE Tail CycleD)\ntarget_link_libraries(CycleD INTERFACE CycleC)\n"
    first, second = order
    graph += (
        f"target_link_libraries({first} INTERFACE {second} Tail)\n"
        if mode == "inactive"
        else f"target_link_libraries({first} INTERFACE Tail {second})\n"
    )
    back = first if mode != "inactive" else f"$<$<CONFIG:Debug>:{first}>"
    graph += f'target_link_libraries({second} INTERFACE "{back}")\n'
    if transitive_tail:
        graph += "target_link_libraries(Tail INTERFACE Next)\n"
    if mode == "acyclic":
        graph += f"set_property(TARGET {first} PROPERTY INTERFACE_LINK_LIBRARIES {second})\nset_property(TARGET {second} PROPERTY INTERFACE_LINK_LIBRARIES Tail)\n"
    elif mode == "diamond":
        graph += f"set_property(TARGET {first} PROPERTY INTERFACE_LINK_LIBRARIES Tail)\nset_property(TARGET {second} PROPERTY INTERFACE_LINK_LIBRARIES Tail)\n"
    if mode == "transparent":
        graph += (
            "add_library(alias_a ALIAS CycleA)\nadd_library(alias_b ALIAS CycleB)\n"
            "add_library(wrapper_a INTERFACE IMPORTED)\nadd_library(wrapper_b INTERFACE IMPORTED)\n"
            "target_link_libraries(wrapper_a INTERFACE alias_a)\ntarget_link_libraries(wrapper_b INTERFACE alias_b)\n"
            "set_property(TARGET CycleA PROPERTY INTERFACE_LINK_LIBRARIES Tail wrapper_b)\n"
            "set_property(TARGET CycleB PROPERTY INTERFACE_LINK_LIBRARIES wrapper_a)\n"
        )
    if mode in ("three", "max"):
        graph += "set_property(TARGET CycleA PROPERTY IMPORTED_LINK_INTERFACE_MULTIPLICITY 3)\n"
    if mode == "max":
        graph += "set_property(TARGET CycleB PROPERTY IMPORTED_LINK_INTERFACE_MULTIPLICITY 4)\n"
    if mode in ("mapped", "fallback", "mapped-zero", "legacy"):
        selected = (
            ""
            if mode == "fallback"
            else f"IMPORTED_LINK_INTERFACE_MULTIPLICITY_DEBUG {0 if mode == 'mapped-zero' else 3} "
        )
        graph += (
            f'set_target_properties(CycleA PROPERTIES IMPORTED_LOCATION_DEBUG "{Path(archives["CycleA"]).as_posix()}" '
            "MAP_IMPORTED_CONFIG_RELEASE DEBUG "
            + selected
            + f"IMPORTED_LINK_INTERFACE_MULTIPLICITY_RELEASE 8 IMPORTED_LINK_INTERFACE_MULTIPLICITY {3 if mode == 'fallback' else 7})\n"
            "set_property(TARGET CycleB PROPERTY IMPORTED_LINK_INTERFACE_MULTIPLICITY 2)\n"
        )
    if mode == "legacy":
        graph += (
            "set_property(TARGET CycleA PROPERTY INTERFACE_LINK_LIBRARIES)\n"
            "set_property(TARGET CycleB PROPERTY INTERFACE_LINK_LIBRARIES)\n"
            "set_property(TARGET CycleA PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG Tail CycleB)\n"
            "set_property(TARGET CycleB PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES CycleA)\n"
        )
    before, after = (
        ("-Wl,--no-as-needed", "-Wl,--as-needed")
        if sys.platform == "linux"
        else ("", "")
    )
    early = "Tail" if mode in ("early-tail", "two-cycles") else ""
    extra = "CycleC" if mode == "two-cycles" else ""
    if transitive_tail:
        early = "Next"
    if mode == "acyclic":
        early += f" Tail {second}"
    elif mode == "diamond":
        early += " Tail"
        extra = second
    graph += f"target_link_libraries(Bullet INTERFACE {early} {before} {first} {extra} {after})\n"
    source = tmp_path / "consumer.cpp"
    source.write_text(
        'extern "C" int cycle_step0();\n'
        + BULLET_CONSUMER.replace(
            "int main() {", "int main() { if (cycle_step0()) return 1;"
        )
    )
    if mode == "two-cycles":
        source.write_text(
            'extern "C" int other_step0();\n'
            + source.read_text().replace(
                "int main() {", "int main() { if (other_step0()) return 1;"
            )
        )
    if mode == "diamond":
        source.write_text(
            'extern "C" int diamond_step();\n'
            + source.read_text().replace(
                "int main() {", "int main() { if (diamond_step()) return 1;"
            )
        )
    expected_order = order * scans + (
        ["CycleC", "CycleD"] * 2 if mode == "two-cycles" else []
    )
    common = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + graph
        + f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    control = tmp_path / "control"
    control.mkdir()
    result = configure(control, common, cmake_args=("-DCMAKE_BUILD_TYPE=Release",))
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", control / "build")
    cmake_fixture_run(control / "build/consumer")
    commands = cmake_fixture_run(
        shutil.which("ninja"), "-C", control / "build", "-t", "commands"
    ).stdout
    # Establish CMake's complete component scans independently of the finder.
    if sys.platform == "linux":
        actual = re.findall(r"lib(Cycle[A-D])\.a", commands.splitlines()[-1])
        assert actual == expected_order
    bullet_module(tmp_path)
    result = configure(
        tmp_path,
        common + "include(DARTFindBullet)\n"
        'if(NOT BULLET_FOUND OR _DART_BULLET_GRAPH_UNKNOWN)\nmessage(FATAL_ERROR "Incomplete target ordering")\nendif()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/bullet.pc" "Name: bullet\nDescription: static cycle\nVersion: 1\nLibs: ${DART_BULLET_PC_LIBS}\nCflags: ${DART_BULLET_PC_CFLAGS} -DBT_USE_DOUBLE_PRECISION\n")\n',
        cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    env = {
        **os.environ,
        "PKG_CONFIG_PATH": str(tmp_path / "build"),
        "PKG_CONFIG_LIBDIR": str(tmp_path / "build"),
    }
    env.pop("PKG_CONFIG_SYSROOT_DIR", None)
    options, flags = pkgconfig_consumer_options("bullet", static=True, env=env)
    pc = tmp_path / "pc-consumer"
    pc.mkdir()
    result = configure(
        pc,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_executable(consumer "{source.as_posix()}")\n' + options,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", pc / "build")
    cmake_fixture_run(pc / "build/consumer")
    cycle_flags = [
        flag for flag in flags if flag in [archives[name] for name in expected_order]
    ]
    assert cycle_flags == [archives[name] for name in expected_order]
    assert flags.count(archives["Tail"]) == 1
    assert flags.index(archives["Tail"]) > max(
        i for i, flag in enumerate(flags) if flag in cycle_flags
    )
    if before:
        assert flags.count(before) == flags.count(after) == 1
        assert flags.index(before) < flags.index(cycle_flags[0])
        assert flags.index(after) > flags.index(archives["Tail"])
    if transitive_tail:
        assert flags.count(archives["Next"]) == 1
        assert flags.index(archives["Next"]) > flags.index(archives["Tail"])
    # GNU archive scanning must fail with too few scans or an early dependency.
    if sys.platform == "linux":
        for missing in ("scan", "tail", *(["next"] if transitive_tail else [])):
            broken = flags.copy()
            if missing == "scan":
                if mode == "inactive":
                    continue
                if mode in ("acyclic", "diamond"):
                    continue
                for name in order * (scans - needed + 1):
                    broken.remove(archives[name])
            elif missing == "next":
                broken.remove(archives["Next"])
                broken.insert(broken.index(archives["Tail"]), archives["Next"])
            else:
                broken.remove(archives["Tail"])
                broken.insert(broken.index(cycle_flags[0]), archives["Tail"])
            negative = tmp_path / f"missing-{missing}"
            negative.mkdir()
            links = " ".join(
                f'"{flag}"' for flag in broken if not flag.startswith(("-I", "-D"))
            )
            result = configure(
                negative,
                "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
                f'add_executable(consumer "{source.as_posix()}")\n'
                f'target_include_directories(consumer PRIVATE "{prefix.as_posix()}/include")\n'
                "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
                f"target_link_libraries(consumer PRIVATE {links})\n",
            )
            assert result.returncode == 0, result.stdout + result.stderr
            built = subprocess.run(
                [shutil.which("cmake"), "--build", str(negative / "build")],
                capture_output=True,
                text=True,
            )
            assert built.returncode != 0, built.stdout + built.stderr
            if missing == "next":
                assert "final_tail" in built.stdout + built.stderr
            else:
                assert (
                    "cycle_step" if missing == "scan" else "cycle_tail"
                ) in built.stdout + built.stderr


def test_bullet_pkgconfig_preserves_link_tokens(tmp_path):
    include = bullet_module(tmp_path)
    result = configure(
        tmp_path,
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        "include(DARTFindBullet)\n"
        "_dart_bullet_record(float64 LIBRARIES BulletCollision-float64 -pthread "
        '"/sdk with spaces/libLinearMath-float64.a" -framework Cocoa -framework Foundation)\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/flags.txt" "${DART_BULLET_PC_LIBS}")\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert shlex.split((tmp_path / "build/flags.txt").read_text()) == [
        "-lBulletCollision-float64",
        "-pthread",
        "/sdk with spaces/libLinearMath-float64.a",
        "-framework",
        "Cocoa",
        "-framework",
        "Foundation",
    ]


@pytest.mark.parametrize(
    "mode",
    [
        "global",
        "mapped",
        "default",
        "modern",
        "modern-empty",
        "interface",
        "interface-mapped",
        "inactive",
        "mismatch",
    ],
)
def test_bullet_legacy_imported_interface_consumer(tmp_path, mode):
    prefix = build_bullet_abi_fixture(
        tmp_path / "bullet", "double", "STATIC", "-float64"
    )
    math_prefix = prefix
    if mode == "mismatch":
        math_prefix = build_bullet_abi_fixture(
            tmp_path / "wrong", "single", "STATIC", "-float64"
        )
    bullet_module(tmp_path)
    graph = bullet_imports(prefix, math_prefix=math_prefix)
    graph += "set_property(TARGET Bullet PROPERTY INTERFACE_LINK_LIBRARIES collision)\n"
    if mode.startswith("interface"):
        graph += (
            "add_library(legacy_wrapper INTERFACE IMPORTED)\n"
            "target_link_libraries(Bullet INTERFACE legacy_wrapper)\n"
            "set_target_properties(legacy_wrapper PROPERTIES "
            + (
                "IMPORTED_CONFIGURATIONS DEBUG MAP_IMPORTED_CONFIG_RELEASE DEBUG "
                "IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG missing_legacy)\n"
                if mode == "interface-mapped"
                else "IMPORTED_LINK_INTERFACE_LIBRARIES missing_legacy)\n"
            )
        )
    if mode == "mapped":
        graph += (
            'set_target_properties(collision PROPERTIES IMPORTED_LOCATION "" '
            'IMPORTED_LOCATION_DEBUG "${collision_file}" IMPORTED_CONFIGURATIONS DEBUG '
            "MAP_IMPORTED_CONFIG_RELEASE DEBUG IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG math "
            "IMPORTED_LINK_INTERFACE_LIBRARIES missing_legacy)\n"
        )
    elif mode == "default":
        graph += (
            'set_target_properties(collision PROPERTIES MAP_IMPORTED_CONFIG_RELEASE "" '
            "IMPORTED_LINK_INTERFACE_LIBRARIES math IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE missing_legacy)\n"
        )
    elif mode in ("modern", "modern-empty"):
        graph += (
            "set_target_properties(collision PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES missing_legacy "
            f'INTERFACE_LINK_LIBRARIES "{"math" if mode == "modern" else ""}")\n'
        )
    else:
        edge = (
            '"$<$<CONFIG:Release>:math>;$<$<CONFIG:Debug>:missing_legacy>"'
            if mode == "inactive"
            else "math"
        )
        graph += f"set_property(TARGET collision PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES {edge})\n"
    source = tmp_path / "consumer.cpp"
    source.write_text(
        "int main() { return 0; }\n" if mode == "modern-empty" else BULLET_CONSUMER
    )
    common = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + graph
        + f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    control = tmp_path / "control"
    control.mkdir()
    result = configure(control, common, cmake_args=("-DCMAKE_BUILD_TYPE=Release",))
    assert result.returncode == 0, result.stdout + result.stderr
    built = subprocess.run(
        [shutil.which("cmake"), "--build", str(control / "build")],
        capture_output=True,
        text=True,
    )
    assert (built.returncode == 0) == (mode != "mismatch"), built.stdout + built.stderr
    if mode != "mismatch":
        cmake_fixture_run(control / "build/consumer")
    result = configure(
        tmp_path,
        common + "include(DARTFindBullet)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/flags.txt" "${DART_BULLET_PC_LIBS}")\n'
        + (
            "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
            if mode != "modern-empty"
            else ""
        )
        + f'if({"" if mode == "mismatch" else "NOT "}BULLET_FOUND)\n'
        'message(FATAL_ERROR "Wrong legacy graph acceptance")\nendif()\n',
        cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    flags = shlex.split((tmp_path / "build/flags.txt").read_text())
    assert all("missing_legacy" not in flag for flag in flags)
    assert any("LinearMath" in flag for flag in flags) == (mode != "modern-empty")
    if mode == "mismatch":
        return
    pc_consumer = tmp_path / "pc-consumer"
    pc_consumer.mkdir()
    links = " ".join(f'"{flag}"' for flag in flags)
    result = configure(
        pc_consumer,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_executable(consumer "{source.as_posix()}")\n'
        f'target_include_directories(consumer PRIVATE "{prefix.as_posix()}/include")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        f"target_link_libraries(consumer PRIVATE {links})\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", pc_consumer / "build")
    cmake_fixture_run(pc_consumer / "build/consumer")


@pytest.mark.parametrize(
    "mode",
    [
        "public",
        "private",
        "nested",
        "config-active",
        "config-inactive",
        "mapped",
        "mapped-empty",
        "link-only",
        "mixed",
        "definition-active",
        "definition-inactive",
    ],
)
@pytest.mark.parametrize("carrier", ["definition", "option"])
def test_bullet_transitive_built_precision(tmp_path, mode, carrier):
    configuration = (
        "Debug" if mode in ("config-active", "definition-active") else "Release"
    )
    double = mode not in (
        "config-inactive",
        "definition-inactive",
        "mapped-empty",
        "link-only",
    )
    expected = "double" if double else "single"
    prefix = build_bullet_abi_fixture(tmp_path / "fixture", expected, "STATIC")
    bullet_module(tmp_path)
    source = tmp_path / "source"
    source.mkdir(exist_ok=True)
    for name in ("collision", "math"):
        scalar = (
            "float"
            if mode == "mixed" and name == "math"
            else ("double" if double else "float")
        )
        (source / f"{name}.cpp").write_text(
            (tmp_path / f"fixture/source/{name}.cpp").read_text()
            + f"static_assert(sizeof(btScalar) == sizeof({scalar}));\n"
        )
    imported = " IMPORTED" if mode in ("mapped", "mapped-empty") else ""
    definition = "BT_USE_DOUBLE_PRECISION"
    if mode.startswith("definition-"):
        definition = "$<$<CONFIG:Debug>:BT_USE_DOUBLE_PRECISION>"
    elif mode == "mapped":
        definition = "$<$<CONFIG:Debug>:BT_USE_DOUBLE_PRECISION>"
    elif mode == "mapped-empty":
        definition = "$<$<CONFIG:>:BT_USE_DOUBLE_PRECISION>"
    graph = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        f"add_library(precision_options INTERFACE{imported})\n"
    )
    if carrier == "option":
        graph += "if(MSVC)\nset(define /D)\nelse()\nset(define -D)\nendif()\n"
        option = definition.replace(
            "BT_USE_DOUBLE_PRECISION", "${define}BT_USE_DOUBLE_PRECISION"
        )
        graph += f'target_compile_options(precision_options INTERFACE "{option}")\n'
    else:
        graph += (
            f'target_compile_definitions(precision_options INTERFACE "{definition}")\n'
        )
    if imported:
        graph += f'set_property(TARGET precision_options PROPERTY MAP_IMPORTED_CONFIG_RELEASE "{"DEBUG" if mode == "mapped" else ""}")\n'
    edge = "precision_options"
    if mode == "nested":
        graph += "add_library(nested_options INTERFACE)\ntarget_link_libraries(nested_options INTERFACE precision_options)\n"
        edge = "nested_options"
    elif mode.startswith("config-"):
        edge = "$<$<CONFIG:Debug>:precision_options>"
    elif mode == "link-only":
        edge = "$<LINK_ONLY:precision_options>"
    for name, filename in (("BulletCollision", "collision"), ("LinearMath", "math")):
        graph += (
            f"add_library({name} STATIC {filename}.cpp)\n"
            f'target_include_directories({name} PRIVATE "{prefix.as_posix()}/include")\n'
        )
        if mode != "mixed" or name == "BulletCollision":
            graph += f'target_link_libraries({name} {"PRIVATE" if mode == "private" else "PUBLIC"} "{edge}")\n'
    graph += (
        "target_link_libraries(BulletCollision PRIVATE LinearMath)\n"
        "add_library(Bullet INTERFACE)\n"
        f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision LinearMath)\n"
    )
    # Compile real symbols with static assertions, independent of finder classification.
    control = tmp_path / "control"
    control.mkdir()
    (control / "source").mkdir()
    for filename in ("collision.cpp", "math.cpp"):
        shutil.copy(source / filename, control / "source" / filename)
    result = configure(
        control, graph, cmake_args=(f"-DCMAKE_BUILD_TYPE={configuration}",)
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", control / "build")
    (source / "backend.cmake").write_text(
        (ROOT / "dart/collision/bullet/CMakeLists.txt")
        .read_text()
        .split("# Search all header and source files")[0]
    )
    tail = "include(DARTFindBullet)\n_dart_bullet_probe_precision(precision)\n"
    if mode == "mixed":
        tail += 'if(BULLET_FOUND OR precision)\nmessage(FATAL_ERROR "Accepted a real transitive ABI mismatch")\nendif()\n'
    else:
        (source / "consumer.cpp").write_text(BULLET_CONSUMER)
        tail += (
            "include(DARTMacros)\ninclude(backend.cmake)\n"
            "add_executable(consumer consumer.cpp)\n"
            "target_link_libraries(consumer PRIVATE Bullet)\n"
            "if(BT_USE_DOUBLE_PRECISION)\n"
            "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\nendif()\n"
            f'if(NOT BULLET_FOUND OR NOT precision STREQUAL "{expected}" OR NOT DART_BULLET_BUILT_PRECISION STREQUAL "{expected}")\n'
            'message(FATAL_ERROR "Lost effective built precision: ${precision}/${DART_BULLET_BUILT_PRECISION}")\nendif()\n'
            "set(external_dependencies Bullet)\n"
            f'configure_file("{ROOT.as_posix()}/cmake/dart_Component.cmake.in" "${{CMAKE_BINARY_DIR}}/component.cmake" @ONLY)\n'
        )
    result = configure(
        tmp_path, graph + tail, cmake_args=(f"-DCMAKE_BUILD_TYPE={configuration}",)
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if mode != "mixed":
        cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
        cmake_fixture_run(tmp_path / "build/consumer")
        component = (tmp_path / "build/component.cmake").read_text()
        assert f'set(DART_BULLET_REQUIRE_PRECISION "{expected}")' in component
        if mode == "public":
            # The recorded installed requirement must accept the matching ABI
            # and reject a different scalar ABI, independent of library names.
            requirement = re.search(
                r'set\(DART_BULLET_REQUIRE_PRECISION "[^"\n]+"\)', component
            ).group()
            wrong = build_bullet_abi_fixture(tmp_path / "wrong", "single", "STATIC")
            for name, dependency in (("matching", prefix), ("wrong", wrong)):
                installed = tmp_path / f"installed-{name}"
                installed.mkdir()
                bullet_module(installed)
                result = configure(
                    installed,
                    "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
                    + bullet_imports(dependency)
                    + requirement
                    + "\ninclude(DARTFindBullet)\n"
                    + f'if({"NOT " if name == "matching" else ""}BULLET_FOUND)\n'
                    'message(FATAL_ERROR "Wrong installed transitive precision")\nendif()\n',
                )
                assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    "mode,actual,expected",
    [
        ("plain-single", "single", "single"),
        ("split-define", "double", "double"),
        ("shell-define", "double", "double"),
        ("global-define", "double", "double"),
        ("config-global-define", "double", "double"),
        ("undef-only", "single", "single"),
        ("define-undef", "single", "single"),
        ("undef-define", "double", "double"),
        ("global-define-target-undef", "single", "single"),
        ("private-define-undef", "single", "single"),
        ("private-undef-define", "double", "double"),
        ("header-define", "double", "double"),
        ("forced-define", "double", "unverified"),
        ("response-define", "double", "unverified"),
        ("source-definition", "double", "unverified"),
        ("source-option", "double", "unverified"),
        ("interface-only", "single", "unverified"),
        ("mixed", "double", "unverified"),
        ("empty-config-debug-define", "single", "single"),
        ("empty-config-legacy-define", "double", "double"),
        ("competing-headers", "single", "unverified"),
        ("private-header-double", "double", "double"),
        ("target-pch", "double", "unverified"),
        ("false-private-definition", "double", "double"),
        ("false-source-definition", "double", "unverified"),
        ("false-interface-definition", "single", "unverified"),
        ("target-name-OFF", "double", "double"),
        ("target-name-FALSE", "double", "double"),
        ("target-name-NOTFOUND", "double", "double"),
        ("target-name-value-NOTFOUND", "double", "double"),
        ("target-name-links-NOTFOUND", "double", "double"),
        ("ambient-cache", "double", "double"),
    ],
)
def test_bullet_built_compile_context(tmp_path, mode, actual, expected):
    prefix = build_bullet_abi_fixture(tmp_path / "fixture", actual, "STATIC")
    alternate = None
    if mode in ("competing-headers", "private-header-double"):
        alternate = tmp_path / "alternate"
        shutil.copytree(prefix / "include", alternate / "include")
    define_header = (
        "#ifndef BT_USE_DOUBLE_PRECISION\n#define BT_USE_DOUBLE_PRECISION\n#endif\n"
    )
    if mode in ("header-define", "private-header-double"):
        header = prefix / "include/LinearMath/btScalar.h"
        header.write_text(define_header + header.read_text())
    if mode.startswith("false-"):
        header = prefix / "include/LinearMath/btScalar.h"
        header.write_text(
            "#if defined(OFF) && !defined(BT_USE_DOUBLE_PRECISION)\n"
            "#define BT_USE_DOUBLE_PRECISION\n#endif\n" + header.read_text()
        )
    forced = tmp_path / "precision.h"
    forced.write_text(define_header)
    response = tmp_path / "precision.rsp"
    graph = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        "if(MSVC)\nset(define /D)\nset(undef /U)\nset(force_include /FI)\n"
        "else()\nset(define -D)\nset(undef -U)\nset(force_include -include)\nendif()\n"
    )
    if mode in ("global-define", "global-define-target-undef"):
        graph += 'set(CMAKE_CXX_FLAGS "${define}BT_USE_DOUBLE_PRECISION")\n'
    elif mode == "config-global-define":
        graph += 'set(CMAKE_CXX_FLAGS_RELEASE "${define}BT_USE_DOUBLE_PRECISION")\n'
    elif mode == "empty-config-debug-define":
        graph += 'set(CMAKE_CXX_FLAGS_DEBUG "${define}BT_USE_DOUBLE_PRECISION")\n'
    elif mode == "response-define":
        graph += f'file(WRITE "{response.as_posix()}" "${{define}}BT_USE_DOUBLE_PRECISION\\n")\n'
    if mode.startswith("target-name-"):
        helper = mode.removeprefix("target-name-")
        graph += (
            f"add_library({helper} INTERFACE)\n"
            f'target_compile_options({helper} INTERFACE "${{define}}BT_USE_DOUBLE_PRECISION")\n'
            "add_library(precision_chain INTERFACE)\n"
            f"target_link_libraries(precision_chain INTERFACE {helper})\n"
        )
    if mode == "ambient-cache":
        graph += (
            "add_library(cache_link_poison INTERFACE)\n"
            'target_compile_options(cache_link_poison INTERFACE "${undef}BT_USE_DOUBLE_PRECISION")\n'
            'set(value cache_link_poison CACHE STRING "" FORCE)\n'
            'set(links cache_link_poison CACHE STRING "" FORCE)\n'
        )
    options = {
        "split-define": '"${define}" "BT_USE_DOUBLE_PRECISION"',
        "shell-define": '"SHELL:${define} BT_USE_DOUBLE_PRECISION"',
        "undef-only": '"${undef}BT_USE_DOUBLE_PRECISION"',
        "define-undef": '"${define}BT_USE_DOUBLE_PRECISION" "${undef}BT_USE_DOUBLE_PRECISION"',
        "undef-define": '"${undef}BT_USE_DOUBLE_PRECISION" "${define}BT_USE_DOUBLE_PRECISION"',
        "global-define-target-undef": '"${undef}BT_USE_DOUBLE_PRECISION"',
        "forced-define": f'"${{force_include}}" "{forced.as_posix()}"',
        "response-define": f'"@{response.as_posix()}"',
    }
    for target, name in (("BulletCollision", "collision"), ("LinearMath", "math")):
        precision = "single" if mode == "mixed" and name == "math" else actual
        source = tmp_path / f"{name}.cpp"
        scalar = "double" if precision == "double" else "float"
        source.write_text(
            (tmp_path / f"fixture/source/{name}.cpp").read_text()
            + f"static_assert(sizeof(btScalar) == sizeof({scalar}));\n"
        )
        graph += (
            f'add_library({target} STATIC "{source.as_posix()}")\n'
            f'target_include_directories({target} PRIVATE "{prefix.as_posix()}/include")\n'
        )
        if mode in options:
            graph += f"target_compile_options({target} PUBLIC {options[mode]})\n"
        elif mode.startswith("private-") and mode != "private-header-double":
            graph += f"target_compile_options({target} PRIVATE {options[mode.removeprefix('private-')]})\n"
        elif mode == "empty-config-legacy-define":
            graph += f'set_property(TARGET {target} PROPERTY COMPILE_FLAGS "${{define}}BT_USE_DOUBLE_PRECISION")\n'
        elif mode == "false-private-definition":
            graph += f"target_compile_definitions({target} PRIVATE OFF)\n"
        elif mode.startswith("target-name-"):
            # Exercise both direct and chained false-like link properties.
            usage = "precision_chain" if target == "BulletCollision" else helper
            graph += f"target_link_libraries({target} PRIVATE {usage})\n"
        elif mode == "ambient-cache":
            graph += (
                f'target_compile_options({target} PRIVATE "${{define}}BT_USE_DOUBLE_PRECISION")\n'
                f'set_source_files_properties("{source.as_posix()}" PROPERTIES COMPILE_OPTIONS "")\n'
            )
        elif mode == "false-source-definition":
            graph += f'set_source_files_properties("{source.as_posix()}" PROPERTIES COMPILE_DEFINITIONS OFF)\n'
        elif mode == "false-interface-definition" and target == "BulletCollision":
            graph += f"target_compile_definitions({target} INTERFACE OFF)\n"
        elif mode == "source-definition":
            graph += f'set_source_files_properties("{source.as_posix()}" PROPERTIES COMPILE_DEFINITIONS BT_USE_DOUBLE_PRECISION)\n'
        elif mode == "source-option":
            graph += f'set_source_files_properties("{source.as_posix()}" PROPERTIES COMPILE_OPTIONS "${{define}}BT_USE_DOUBLE_PRECISION")\n'
        elif mode == "mixed" and precision == "double":
            graph += f'target_compile_options({target} PRIVATE "${{define}}BT_USE_DOUBLE_PRECISION")\n'
        elif mode == "interface-only" and target == "BulletCollision":
            graph += f'target_compile_options({target} INTERFACE "${{define}}BT_USE_DOUBLE_PRECISION")\n'
        elif mode == "competing-headers":
            graph += f'target_include_directories({target} PRIVATE "{alternate.as_posix()}/include")\n'
        elif mode == "target-pch":
            graph += (
                f'target_precompile_headers({target} PRIVATE "{forced.as_posix()}")\n'
            )
    selected = alternate if mode == "private-header-double" else prefix
    graph += (
        "target_link_libraries(BulletCollision PRIVATE LinearMath)\n"
        "add_library(Bullet INTERFACE)\n"
        f'target_include_directories(Bullet INTERFACE "{selected.as_posix()}/include")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision LinearMath)\n"
    )
    source = tmp_path / "consumer.cpp"
    source.write_text(BULLET_CONSUMER)
    consumer = (
        f'add_executable(consumer "{source.as_posix()}")\n'
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    if actual == "double":
        consumer += (
            "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        )
    configuration = "" if mode.startswith("empty-config-") else "Release"
    args = (f"-DCMAKE_BUILD_TYPE={configuration}",)
    # Verify the implementation independently, including precise mixed-ABI link failures.
    control = tmp_path / "control"
    control.mkdir()
    result = configure(control, graph + consumer, cmake_args=args)
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(
        shutil.which("cmake"),
        "--build",
        control / "build",
        "--target",
        "BulletCollision",
        "LinearMath",
    )
    result = subprocess.run(
        [
            shutil.which("cmake"),
            "--build",
            str(control / "build"),
            "--target",
            "consumer",
        ],
        capture_output=True,
        text=True,
    )
    output = result.stdout + result.stderr
    (control / "consumer-build.log").write_text(output)
    if mode in ("mixed", "interface-only", "false-interface-definition"):
        assert result.returncode != 0, output
        assert (
            "btConvexHullComputer" if mode == "mixed" else "btStaticPlaneShape"
        ) in output
    else:
        assert result.returncode == 0, output
        cmake_fixture_run(control / "build/consumer")
    bullet_module(tmp_path)
    result = configure(
        tmp_path,
        graph + "include(DARTFindBullet)\n_dart_bullet_probe_precision(observed)\n"
        'file(WRITE "${CMAKE_BINARY_DIR}/precision.txt" "${BULLET_FOUND}\\n${DART_BULLET_BUILT_PRECISION}\\n${observed}\\n")\n',
        cmake_args=args,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    found, built, observed = (tmp_path / "build/precision.txt").read_text().splitlines()
    if expected == "unverified":
        assert (found, built, observed) == ("FALSE", "", "")
    else:
        assert (found, built, observed) == ("TRUE", expected, expected)


def test_bullet_c_only_built_precision_is_unverified(tmp_path):
    include = bullet_module(tmp_path)
    source = tmp_path / "part.c"
    source.write_text(
        "#ifndef BT_USE_DOUBLE_PRECISION\n#error Missing fixture definition\n"
        "#endif\nint bt_dummy;\n"
    )
    result = configure(
        tmp_path,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(C)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        f'add_library(BulletCollision STATIC "{source.as_posix()}")\n'
        "target_compile_definitions(BulletCollision PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "add_library(Bullet INTERFACE)\n"
        f'target_include_directories(Bullet INTERFACE "{include.as_posix()}")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision)\n"
        "get_property(before GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        "include(DARTFindBullet)\n_dart_bullet_probe_precision(observed)\n"
        "get_property(after GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        'if(NOT before STREQUAL after OR "CXX" IN_LIST after)\n'
        'message(FATAL_ERROR "Built target probe enabled CXX")\nendif()\n'
        "if(BULLET_FOUND OR Bullet_FOUND OR DART_BULLET_BUILT_PRECISION OR observed)\n"
        'message(FATAL_ERROR "C-only target claimed a C++ Bullet ABI")\nendif()\n',
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")


@pytest.mark.parametrize("scope", ["direct", "function"])
@pytest.mark.parametrize("abi", ["same", "opposite", "mixed"])
def test_bullet_c_only_installed_discovery(tmp_path, scope, abi):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double", "STATIC")
    other = build_bullet_abi_fixture(tmp_path / "other", "single", "STATIC")
    stage = install_bullet_dart_fixture(tmp_path / "dart", prefix, "STATIC")
    (stage / "include").mkdir()
    with (tmp_path / "dart/source/CMakeLists.txt").open("a") as f:
        f.write(
            "set(DART_USE_SYSTEM_IMGUI ON)\nset(INCLUDE_INSTALL_DIR include)\nset(DART_MAJOR_VERSION 7)\n"
            "include(CMakePackageConfigHelpers)\n"
            f'configure_package_config_file("{ROOT.as_posix()}/cmake/DARTConfig.cmake.in" '
            '"${CMAKE_BINARY_DIR}/DARTConfig.cmake" INSTALL_DESTINATION lib/cmake/DART '
            "PATH_VARS INCLUDE_INSTALL_DIR)\n"
            'install(FILES "${CMAKE_BINARY_DIR}/DARTConfig.cmake" DESTINATION lib/cmake/DART)\n'
        )
    cmake_fixture_run(
        shutil.which("cmake"),
        "-S",
        tmp_path / "dart/source",
        "-B",
        tmp_path / "dart/build",
    )
    cmake_fixture_run(shutil.which("cmake"), "--install", tmp_path / "dart/build")
    selected = other if abi == "opposite" else prefix
    math = other if abi != "same" else prefix
    source = tmp_path / "consumer"
    source.mkdir()
    (source / "main.c").write_text("int main(void) { return 0; }\n")
    (source / "FindBullet.cmake").write_text(
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        f'set(BULLET_INCLUDE_DIRS "{selected.as_posix()}/include")\n'
        f'set(BULLET_INCLUDE_DIR "{selected.as_posix()}/include")\n'
        'set(BULLET_COLLISION_LIBRARY "${collision_file}")\n'
        'set(BULLET_MATH_LIBRARY "${math_file}")\n'
    )
    discovery = (
        f'find_package(DART CONFIG {"REQUIRED" if abi == "same" else "QUIET"} PATHS "{stage.as_posix()}/lib/cmake/DART" NO_DEFAULT_PATH)\n'
        + f'if({"NOT " if abi == "same" else ""}DART_FOUND)\n'
        'message(FATAL_ERROR "Wrong C-only installed ABI acceptance")\nendif()\n'
    )
    if scope == "function":
        discovery = "function(discover)\n" + discovery + "endfunction()\ndiscover()\n"
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\nproject(Consumer LANGUAGES C)\n"
        f'set(CMAKE_MODULE_PATH "{source.as_posix()}")\n'
        + bullet_imports(selected, math_prefix=math)
        + "set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)\n"
        "set(CMAKE_REQUIRED_FLAGS caller-flags)\n"
        "set(CMAKE_REQUIRED_LIBRARIES caller-library)\n"
        'set(before_c "${CMAKE_C_COMPILER}")\n'
        'set(before_cxx "${CMAKE_CXX_COMPILER}")\n'
        'set(before_cxx_loaded "${CMAKE_CXX_COMPILER_LOADED}")\n'
        "get_property(before_languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        + discovery
        + "get_property(after_languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        'if(NOT after_languages STREQUAL before_languages OR "CXX" IN_LIST after_languages\n'
        ' OR NOT CMAKE_C_COMPILER STREQUAL before_c OR NOT "${CMAKE_CXX_COMPILER}" STREQUAL before_cxx\n'
        ' OR NOT "${CMAKE_CXX_COMPILER_LOADED}" STREQUAL before_cxx_loaded\n'
        ' OR NOT CMAKE_TRY_COMPILE_TARGET_TYPE STREQUAL "STATIC_LIBRARY"\n'
        ' OR NOT CMAKE_REQUIRED_FLAGS STREQUAL "caller-flags"\n'
        ' OR NOT CMAKE_REQUIRED_LIBRARIES STREQUAL "caller-library")\n'
        'message(FATAL_ERROR "C-only discovery leaked caller state: '
        "languages=${before_languages}->${after_languages}; "
        "C=${before_c}->${CMAKE_C_COMPILER}; CXX=${before_cxx}->${CMAKE_CXX_COMPILER}; "
        "CXX-loaded=${before_cxx_loaded}->${CMAKE_CXX_COMPILER_LOADED}; "
        "try-compile=${CMAKE_TRY_COMPILE_TARGET_TYPE}; "
        'required-flags=${CMAKE_REQUIRED_FLAGS}; required-libraries=${CMAKE_REQUIRED_LIBRARIES}")\nendif()\n'
        "add_executable(consumer main.c)\n",
    )
    cmake_fixture_run(
        shutil.which("cmake"), "-G", "Ninja", "-S", source, "-B", source / "build"
    )
    cmake_fixture_run(shutil.which("cmake"), "--build", source / "build")
    cmake_fixture_run(
        source / ("build/consumer.exe" if os.name == "nt" else "build/consumer")
    )


@pytest.mark.parametrize("configuration", ["Release", "Debug", "", "multi"])
@pytest.mark.parametrize("mapping", ["direct", "different", "empty", "mixed"])
def test_bullet_c_only_configuration_snapshot(tmp_path, configuration, mapping):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double", "STATIC")
    wrong = build_bullet_abi_fixture(tmp_path / "wrong", "single", "STATIC")
    bullet_module(tmp_path)
    graph = bullet_imports(prefix)
    wrong_collision = (
        wrong / "lib" / (wrong / "link-files.txt").read_text().splitlines()[0]
    ).as_posix()
    if mapping != "direct":
        graph += (
            'set_target_properties(collision PROPERTIES IMPORTED_CONFIGURATIONS "RELEASE;DEBUG" '
            'IMPORTED_LOCATION_RELEASE "${collision_file}" '
            f'IMPORTED_LOCATION_DEBUG "{wrong_collision}")\n'
            "set_target_properties(math PROPERTIES IMPORTED_CONFIGURATIONS DEBUG "
            'IMPORTED_LOCATION_DEBUG "${math_file}")\n'
        )
        for config in ("RELEASE", "DEBUG", ""):
            collision_mapping = "" if mapping == "empty" else "RELEASE"
            if mapping == "mixed" and config == "DEBUG":
                collision_mapping = "DEBUG"
            graph += (
                f'set_property(TARGET collision PROPERTY MAP_IMPORTED_CONFIG_{config} "{collision_mapping}")\n'
                f"set_property(TARGET math PROPERTY MAP_IMPORTED_CONFIG_{config} DEBUG)\n"
            )
    (tmp_path / "main.c").write_text("int main(void) { return 0; }\n")
    args = (
        ("-G", "Ninja Multi-Config", "-DCMAKE_CONFIGURATION_TYPES=Release;Debug")
        if configuration == "multi"
        else (f"-DCMAKE_BUILD_TYPE={configuration}",)
    )
    accepted = mapping != "mixed" or configuration not in ("Debug", "multi")
    result = configure(
        tmp_path,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(C)\n"
        + graph
        + "set(CMAKE_TRY_COMPILE_CONFIGURATION caller-config)\n"
        "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        f'if({"NOT " if accepted else ""}BULLET_FOUND OR NOT CMAKE_TRY_COMPILE_CONFIGURATION STREQUAL "caller-config" OR CMAKE_CXX_COMPILER_LOADED)\n'
        'message(FATAL_ERROR "C-only configuration snapshot failed")\nendif()\n'
        f'add_executable(consumer "{(tmp_path / "main.c").as_posix()}")\n',
        cmake_args=args,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(
        shutil.which("cmake"),
        "--build",
        tmp_path / "build",
        "--config",
        "Release" if configuration == "multi" else configuration,
    )
    children = list((tmp_path / "build/CMakeFiles").glob("DARTBulletABI-*"))
    assert len(children) == (2 if configuration == "multi" else 1)
    for child in children:
        assert (
            "project(DARTBulletABI LANGUAGES CXX)"
            in (child / "CMakeLists.txt").read_text()
        )
        assert (child / "single.log").is_file() and (child / "double.log").is_file()


@pytest.mark.parametrize("identity", ["imported", "built", "missing", "dynamic"])
def test_bullet_c_only_transitive_usage_and_identity(tmp_path, identity):
    fixture = tmp_path / "bullet with spaces"
    prefix = build_bullet_abi_fixture(fixture, "double", "STATIC")
    source = fixture / "source"
    collision = source / "collision.cpp"
    collision.write_text(
        'extern "C" int dart_first();\n'
        + collision.read_text().replace("{\n", "{ dart_first();\n", 1)
    )
    (source / "first.cpp").write_text(
        'extern "C" int dart_second();\nextern "C" int dart_first() { return dart_second(); }\n'
    )
    (source / "second.cpp").write_text(
        'extern "C" int dart_last();\nextern "C" int dart_second() { return dart_last(); }\n'
    )
    (source / "last.cpp").write_text(
        '#ifdef __linux__\nextern "C" int dart_link_required;\n'
        'extern "C" int dart_last() { return dart_link_required; }\n'
        '#else\nextern "C" int dart_last() { return 0; }\n#endif\n'
    )
    with (source / "CMakeLists.txt").open("a") as f:
        f.write(
            "add_library(DARTProbeAuxA STATIC first.cpp last.cpp)\nadd_library(DARTProbeAuxB STATIC second.cpp)\ninstall(TARGETS DARTProbeAuxA DARTProbeAuxB DESTINATION lib)\n"
        )
    cmake_fixture_run(shutil.which("cmake"), "--build", fixture / "build")
    # Install compares rounded timestamps, so a same-second rebuild can leave
    # the original archive behind. This prefix belongs only to this fixture.
    shutil.rmtree(prefix)
    cmake_fixture_run(shutil.which("cmake"), "--install", fixture / "build")
    for archive in (prefix / "lib").iterdir():
        assert archive.read_bytes() == (fixture / "build" / archive.name).read_bytes()
    header = prefix / "include/btBulletCollisionCommon.h"
    header.write_text(
        header.read_text()
        + (
            "#ifndef REQUIRED_CHILD_DEFINITION\n#error missing transitive compile definition\n#endif\n"
            "#if IDENTITY_PRESENT != EXPECTED_IDENTITY\n#error changed target identity\n#endif\n"
            "#ifdef WRONG_NORMAL_MAPPING\n#error changed normal wrapper mapping\n#endif\n"
        )
    )
    bullet_module(tmp_path)
    (tmp_path / "main.c").write_text("int main(void) { return 0; }\n")
    (tmp_path / "main.cpp").write_text(BULLET_CONSUMER)
    graph = (
        bullet_imports(prefix)
        .replace(
            "add_library(Bullet INTERFACE IMPORTED)",
            "add_library(actual_bullet INTERFACE)",
        )
        .replace(
            "target_link_libraries(Bullet ", "target_link_libraries(actual_bullet "
        )
        .replace(
            "target_include_directories(Bullet ",
            "target_include_directories(actual_bullet ",
        )
    )
    graph += "add_library(Bullet ALIAS actual_bullet)\nadd_library(usage INTERFACE IMPORTED)\n"
    if identity in ("imported", "dynamic"):
        graph += "add_library(identity_marker INTERFACE IMPORTED)\n"
    elif identity == "built":
        graph += f'add_library(identity_marker STATIC "{(tmp_path / "main.c").as_posix()}")\n'
    predicate = "$<TARGET_EXISTS:identity_marker>"
    if identity == "dynamic":
        predicate = "$<TARGET_EXISTS:$<IF:$<BOOL:1>,identity_marker,missing_marker>>"
    option_predicate = f"$<NOT:{predicate}>" if identity == "missing" else predicate
    graph += (
        'target_link_libraries(actual_bullet INTERFACE "$<TARGET_NAME_IF_EXISTS:usage>" "$<TARGET_NAME_IF_EXISTS:missing_usage>")\n'
        "target_link_libraries(usage INTERFACE DARTProbeAuxA DARTProbeAuxB DARTProbeAuxA)\n"
        f'target_link_directories(usage INTERFACE "{prefix.as_posix()}/lib")\n'
        f'target_compile_definitions(usage INTERFACE "$<$<COMPILE_LANGUAGE:CXX>:REQUIRED_CHILD_DEFINITION>" "IDENTITY_PRESENT={predicate}" EXPECTED_IDENTITY={0 if identity == "missing" else 1})\n'
        f'target_link_options(usage INTERFACE "$<$<AND:$<PLATFORM_ID:Linux>,{option_predicate}>:LINKER:--defsym,dart_link_required=0>")\n'
        "set_property(TARGET actual_bullet PROPERTY MAP_IMPORTED_CONFIG_RELEASE Debug)\n"
        'target_compile_definitions(actual_bullet INTERFACE "$<$<CONFIG:Debug>:WRONG_NORMAL_MAPPING>")\n'
    )
    # Ordinary CMake consumers establish the graph's actual usage and link order.
    control = tmp_path / "control"
    control.mkdir()
    result = configure(
        control,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(C CXX)\n"
        + graph
        + f'add_executable(consumer "{(tmp_path / "main.cpp").as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n",
        cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", control / "build")
    # The positive link must actually need the second A scan and, on ELF, the
    # required linker definition. Otherwise stale fixture archives hide losses.
    for missing in ("order", "option") if sys.platform == "linux" else ():
        negative = tmp_path / f"missing-{missing}"
        negative.mkdir()
        broken = (
            graph.replace(
                "DARTProbeAuxA DARTProbeAuxB DARTProbeAuxA",
                "DARTProbeAuxA DARTProbeAuxB",
            )
            if missing == "order"
            else graph + "set_property(TARGET usage PROPERTY INTERFACE_LINK_OPTIONS)\n"
        )
        result = configure(
            negative,
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(C CXX)\n"
            + broken
            + f'add_executable(consumer "{(tmp_path / "main.cpp").as_posix()}")\n'
            "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
            "target_link_libraries(consumer PRIVATE Bullet)\n",
            cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
        )
        assert result.returncode == 0, result.stdout + result.stderr
        built = subprocess.run(
            [shutil.which("cmake"), "--build", str(negative / "build")],
            capture_output=True,
            text=True,
        )
        output = built.stdout + built.stderr
        assert built.returncode != 0, output
        assert ("dart_last" if missing == "order" else "dart_link_required") in output
    result = configure(
        tmp_path,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(C)\n"
        + graph
        + "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        + f'if({"" if identity == "dynamic" else "NOT "}BULLET_FOUND OR CMAKE_CXX_COMPILER_LOADED)\n'
        'message(FATAL_ERROR "Wrong C-only usage acceptance")\nendif()\n',
        cmake_args=("-DCMAKE_BUILD_TYPE=Release",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if identity == "dynamic":
        assert "cannot snapshot target context" in result.stdout
    else:
        child = next((tmp_path / "build/CMakeFiles").glob("DARTBulletABI-*"))
        snapshot = (child / "CMakeLists.txt").read_text()
        assert "DARTProbeAuxA;DARTProbeAuxB;DARTProbeAuxA" in snapshot
        assert 'add_library("actual_bullet" INTERFACE)' in snapshot
        assert 'add_library("Bullet" ALIAS "actual_bullet")' in snapshot


@pytest.mark.parametrize("selected_compiler", [False, True])
def test_bullet_c_only_toolchain_and_stale_probe(tmp_path, selected_compiler):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double", "STATIC")
    bullet_module(tmp_path)
    header = prefix / "include/btBulletCollisionCommon.h"
    header.write_text(
        header.read_text()
        + "#if !defined(BULLET_TOOLCHAIN_FLAG) || !defined(BULLET_CONFIG_FLAG)\n#error lost toolchain flags\n#endif\n"
    )
    token = 'alpha;${DO_NOT_EXPAND};]=];"quoted";\\slash;\nlast'
    digest = hashlib.sha256(token.encode()).hexdigest()
    toolchain = tmp_path / "tool chain.cmake"
    toolchain.write_text(
        "set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES BULLET_PLATFORM_TOKEN)\n"
        'if(CMAKE_PROJECT_NAME STREQUAL "DARTBulletABI")\n'
        "cmake_policy(GET CMP0141 debug_policy)\n"
        'if(NOT debug_policy STREQUAL "OLD")\nmessage(FATAL_ERROR "Changed unset parent debug policy")\nendif()\nendif()\n'
        'string(SHA256 actual "${BULLET_PLATFORM_TOKEN}")\n'
        f'if(NOT actual STREQUAL "{digest}")\n'
        'message(FATAL_ERROR "Lost literal toolchain value")\nendif()\n'
    )
    compiler = re.search(
        r"^CMAKE_CXX_COMPILER:FILEPATH=(.+)$",
        (tmp_path / "bullet/build/CMakeCache.txt").read_text(),
        re.MULTILINE,
    ).group(1)
    compiler_args = (
        (
            f"-DCMAKE_CXX_COMPILER={compiler}",
            "-DCMAKE_CXX_COMPILER_TARGET=",
            "-DCMAKE_SYSROOT=",
        )
        if selected_compiler
        else ()
    )
    result = configure(
        tmp_path,
        "cmake_policy(VERSION 3.22)\n"
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(C)\n"
        + bullet_imports(prefix)
        + "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        'if(NOT BULLET_FOUND OR CMAKE_CROSSCOMPILING)\nmessage(FATAL_ERROR "Lost native toolchain context")\nendif()\n'
        f'set(CMAKE_CXX_COMPILER "{(tmp_path / "missing-cxx").as_posix()}")\n'
        "include(DARTFindBullet)\n"
        'if(BULLET_FOUND OR CMAKE_CXX_COMPILER_LOADED)\nmessage(FATAL_ERROR "Reused stale ABI or changed caller language")\nendif()\n'
        "set(CMAKE_CXX_COMPILER CMAKE_CXX_COMPILER-NOTFOUND)\ninclude(DARTFindBullet)\n"
        'if(BULLET_FOUND)\nmessage(FATAL_ERROR "Replaced an explicitly unavailable compiler")\nendif()\n',
        cmake_args=(
            f"-DCMAKE_TOOLCHAIN_FILE={toolchain}",
            f"-DBULLET_PLATFORM_TOKEN={token}",
            "-DCMAKE_BUILD_TYPE=Debug",
            "-DCMAKE_CXX_FLAGS=-DBULLET_TOOLCHAIN_FLAG",
            "-DCMAKE_CXX_FLAGS_DEBUG=-DBULLET_CONFIG_FLAG",
            *compiler_args,
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    children = list((tmp_path / "build/CMakeFiles").glob("DARTBulletABI-*"))
    assert len(children) == 2
    assert sum((child / "double.log").exists() for child in children) == 1
    for child in children:
        systems = list((child / "build/CMakeFiles").glob("*/CMakeSystem.cmake"))
        assert systems and 'set(CMAKE_CROSSCOMPILING "FALSE")' in systems[0].read_text()
    success = next(child for child in children if (child / "double.log").exists())
    cache = (success / "build/CMakeCache.txt").read_text()
    assert (
        f"CMAKE_CXX_COMPILER:FILEPATH={compiler}" in cache
        or f"CMAKE_CXX_COMPILER:STRING={compiler}" in cache
    )
    if selected_compiler:
        assert "CMAKE_SYSROOT:STRING=\n" in cache
        assert "CMAKE_CXX_COMPILER_TARGET:STRING=\n" in cache


@pytest.mark.parametrize("language", ["C", "CXX"])
def test_bullet_system_include_precedence(tmp_path, language):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double", "STATIC")
    wrong = tmp_path / "wrong headers"
    shutil.copytree(prefix / "include", wrong)
    (wrong / "btBulletCollisionCommon.h").write_text(
        "#error SYSTEM classification changed header lookup\n"
    )
    bullet_module(tmp_path)
    graph = bullet_imports(prefix).replace(
        f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")',
        f'target_include_directories(Bullet INTERFACE "{wrong.as_posix()}")',
    )
    graph += f'set_target_properties(collision PROPERTIES SYSTEM FALSE INTERFACE_INCLUDE_DIRECTORIES "{prefix.as_posix()}/include")\n'
    cpp_source = tmp_path / "consumer.cpp"
    cpp_source.write_text(BULLET_CONSUMER)
    cpp_consumer = (
        f'add_executable(consumer "{cpp_source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    # Ordinary CMake proves that the successful header selection depends on SYSTEM.
    for system in (False, True):
        control = tmp_path / f"control-{system}"
        control.mkdir()
        result = configure(
            control,
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
            + graph
            + f"set_property(TARGET collision PROPERTY SYSTEM {str(system).upper()})\n"
            + cpp_consumer,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        built = subprocess.run(
            [shutil.which("cmake"), "--build", str(control / "build")],
            capture_output=True,
            text=True,
        )
        assert (built.returncode == 0) == (not system), built.stdout + built.stderr
        if system:
            assert (
                "SYSTEM classification changed header lookup"
                in built.stdout + built.stderr
            )
        else:
            cmake_fixture_run(
                control
                / ("build/consumer.exe" if os.name == "nt" else "build/consumer")
            )
    c_source = tmp_path / "consumer.c"
    c_source.write_text("int main(void) { return 0; }\n")
    result = configure(
        tmp_path,
        f"set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language({language})\n"
        + graph
        + "get_property(before_languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
        "get_property(languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        "if(NOT BULLET_FOUND OR NOT languages STREQUAL before_languages)\n"
        'message(FATAL_ERROR "Lost SYSTEM include precedence or caller language")\nendif()\n'
        + (
            cpp_consumer
            if language == "CXX"
            else f'add_executable(consumer "{c_source.as_posix()}")\n'
        ),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", tmp_path / "build")
    cmake_fixture_run(
        tmp_path / ("build/consumer.exe" if os.name == "nt" else "build/consumer")
    )


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


@pytest.mark.parametrize(
    "binding", ["unscoped", "global", "local", "alias", "promoted-alias"]
)
def test_bullet_directory_scopes_remain_unverified(tmp_path, binding):
    double = build_bullet_abi_fixture(tmp_path / "double", "double", "STATIC")
    single = build_bullet_abi_fixture(tmp_path / "single", "single", "STATIC")
    bullet_module(tmp_path)
    (tmp_path / "consumer.cpp").write_text(BULLET_CONSUMER)
    child = tmp_path / "source/child"
    child.mkdir(parents=True)

    def imports(prefix, names, global_import=False):
        files = (prefix / "link-files.txt").read_text().splitlines()
        return "".join(
            f"add_library({name} UNKNOWN IMPORTED{' GLOBAL' if global_import else ''})\n"
            f'set_target_properties({name} PROPERTIES IMPORTED_LOCATION "{(prefix / "lib" / file).as_posix()}")\n'
            for name, file in zip(names, files)
        )

    graph = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "add_library(Bullet INTERFACE IMPORTED GLOBAL)\n"
        f'target_include_directories(Bullet INTERFACE "{double.as_posix()}/include")\n'
    )
    links = "target_link_libraries(Bullet INTERFACE collision math)\n"
    if binding in ("unscoped", "global"):
        graph += imports(double, ("collision", "math"), True)
        child_content = links if binding == "global" else ""
    elif binding == "local":
        child_content = imports(double, ("collision", "math")) + links
    else:
        graph += imports(double, ("real_collision", "real_math"))
        child_content = (
            "add_library(collision ALIAS real_collision)\n"
            "add_library(math ALIAS real_math)\n" + links
        )
    (child / "CMakeLists.txt").write_text(child_content)
    graph += "add_subdirectory(child)\n"
    if binding == "promoted-alias":
        # Local aliases keep their lookup scope after the targets become global.
        graph += "set_property(TARGET real_collision real_math PROPERTY IMPORTED_GLOBAL TRUE)\n"
    if binding not in ("unscoped", "global"):
        graph += imports(single, ("collision", "math"), True)
    if binding == "unscoped":
        graph += links
    graph += (
        f'add_executable(consumer "{(tmp_path / "consumer.cpp").as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
        "get_property(before_links TARGET Bullet PROPERTY INTERFACE_LINK_LIBRARIES)\n"
        'set(CMAKE_REQUIRED_LIBRARIES "caller-library")\n'
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        "set(DART_BULLET_REQUIRE_PRECISION double)\ninclude(DARTFindBullet)\n"
    )
    if binding == "unscoped":
        graph += (
            'if(NOT BULLET_FOUND OR NOT _dart_bullet_precision STREQUAL "double")\n'
            'message(FATAL_ERROR "Unscoped double ABI was not verified")\nendif()\n'
        )
    else:
        graph += (
            "if(BULLET_FOUND OR NOT _DART_BULLET_GRAPH_UNKNOWN OR NOT DART_BULLET_PC_ERROR)\n"
            'message(FATAL_ERROR "Scoped ABI or incomplete metadata was accepted")\nendif()\n'
            'if(DART_BULLET_PC_LIBS MATCHES "::@")\n'
            'message(FATAL_ERROR "Directory markers escaped into pkg-config")\nendif()\n'
        )
    graph += (
        "get_property(after_links TARGET Bullet PROPERTY INTERFACE_LINK_LIBRARIES)\n"
        'if(NOT "${before_links}" STREQUAL "${after_links}" OR NOT CMAKE_REQUIRED_LIBRARIES STREQUAL "caller-library")\n'
        'message(FATAL_ERROR "Finder changed caller graph or check state")\nendif()\n'
    )
    result = configure(tmp_path, graph, cmake_args=("-DCMAKE_BUILD_TYPE=Release",))
    assert result.returncode == 0, result.stdout + result.stderr
    # The original scoped graph still links double despite competing parent names.
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")
    cmake_fixture_run(tmp_path / "build/consumer")


@pytest.mark.parametrize("language", ["C", "CXX"])
@pytest.mark.parametrize(
    "selection", ["default", "noconfig", "mapped", "mapped-empty", "legacy"]
)
def test_bullet_empty_configuration_pkgconfig_consumer(tmp_path, language, selection):
    prefix = build_bullet_abi_fixture(
        tmp_path / "bullet", "double", "STATIC", "-float64"
    )
    wrong = build_bullet_abi_fixture(tmp_path / "wrong", "single", "STATIC")
    bullet_module(tmp_path)
    source = tmp_path / "consumer.cpp"
    source.write_text(BULLET_CONSUMER)
    graph = bullet_imports(prefix).replace(
        f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")',
        f'target_include_directories(Bullet INTERFACE "$<$<CONFIG:>:{prefix.as_posix()}/include>" '
        f'"$<$<CONFIG:Release>:{wrong.as_posix()}/include>")',
    )
    wrong_files = [
        wrong / "lib" / name
        for name in (wrong / "link-files.txt").read_text().splitlines()
    ]
    for target, good, bad in (
        ("collision", "${collision_file}", wrong_files[0]),
        ("math", "${math_file}", wrong_files[1]),
    ):
        generic = good if selection in ("default", "mapped-empty") else bad.as_posix()
        release = good if selection == "mapped" else bad.as_posix()
        graph += (
            f'set_target_properties({target} PROPERTIES IMPORTED_LOCATION "{generic}" '
            f'IMPORTED_CONFIGURATIONS RELEASE IMPORTED_LOCATION_RELEASE "{release}")\n'
        )
        if selection in ("noconfig", "legacy", "mapped-empty"):
            noconfig = bad.as_posix() if selection == "mapped-empty" else good
            graph += f'set_property(TARGET {target} PROPERTY IMPORTED_LOCATION_NOCONFIG "{noconfig}")\n'
        if selection in ("mapped", "mapped-empty"):
            mapping = "Release" if selection == "mapped" else ""
            graph += f'set_property(TARGET {target} PROPERTY MAP_IMPORTED_CONFIG_NOCONFIG "{mapping}")\n'
    if selection == "legacy":
        graph = graph.replace(
            "target_link_libraries(Bullet INTERFACE collision_wrapper math)",
            "target_link_libraries(Bullet INTERFACE collision_wrapper)",
        )
        graph += (
            "add_library(wrong_math UNKNOWN IMPORTED)\n"
            f'set_property(TARGET wrong_math PROPERTY IMPORTED_LOCATION "{wrong_files[1].as_posix()}")\n'
            "set_target_properties(collision PROPERTIES "
            "IMPORTED_LINK_INTERFACE_LIBRARIES wrong_math IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG math)\n"
        )
    # Prove the provider's native empty configuration before testing the finder.
    ordinary = tmp_path / "ordinary"
    ordinary.mkdir()
    result = configure(
        ordinary,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        + graph
        + f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n",
        cmake_args=("-DCMAKE_BUILD_TYPE=",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    executable = "consumer.exe" if os.name == "nt" else "consumer"
    cmake_fixture_run(shutil.which("cmake"), "--build", ordinary / "build")
    cmake_fixture_run(ordinary / "build" / executable)
    result = configure(
        tmp_path,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\n"
        f"enable_language({language})\n"
        + graph
        + "get_property(before_languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        'set(CMAKE_REQUIRED_FLAGS "caller-flags")\n'
        'set(CMAKE_REQUIRED_LIBRARIES "caller-library")\n'
        'set(CMAKE_TRY_COMPILE_CONFIGURATION "caller-config")\n'
        "include(DARTFindBullet)\n_dart_bullet_probe_precision(precision)\n"
        'if(NOT BULLET_FOUND OR NOT precision STREQUAL "double" OR DART_BULLET_PC_ERROR)\n'
        'message(FATAL_ERROR "Empty configuration failed double ABI validation")\nendif()\n'
        "get_property(after_languages GLOBAL PROPERTY ENABLED_LANGUAGES)\n"
        'if(NOT "${before_languages}" STREQUAL "${after_languages}" '
        'OR NOT CMAKE_REQUIRED_FLAGS STREQUAL "caller-flags" '
        'OR NOT CMAKE_REQUIRED_LIBRARIES STREQUAL "caller-library" '
        'OR NOT CMAKE_TRY_COMPILE_CONFIGURATION STREQUAL "caller-config")\n'
        'message(FATAL_ERROR "Empty configuration probe changed caller state")\nendif()\n'
        '_dart_bullet_config_location(selected collision "")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/selected.txt" "${selected}")\n'
        'file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/actual.txt" CONTENT "$<TARGET_LINKER_FILE:collision>")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/bullet-selection.pc" "Name: bullet-selection\\nDescription: Empty configuration fixture\\nVersion: 1\\nCflags: -DBT_USE_DOUBLE_PRECISION${DART_BULLET_PC_CFLAGS}\\nLibs:${DART_BULLET_PC_LIBS}\\n")\n'
        "set(DART_BULLET_REQUIRE_PRECISION single)\ninclude(DARTFindBullet)\n"
        'if(BULLET_FOUND OR Bullet_FOUND)\nmessage(FATAL_ERROR "Opposite scalar ABI accepted")\nendif()\n',
        cmake_args=("-DCMAKE_BUILD_TYPE=",),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (tmp_path / "build/selected.txt").read_text() == (
        tmp_path / "build/actual.txt"
    ).read_text()
    env = {
        **os.environ,
        "PKG_CONFIG_PATH": str(tmp_path / "build"),
        "PKG_CONFIG_LIBDIR": str(tmp_path / "build"),
    }
    env.pop("PKG_CONFIG_SYSROOT_DIR", None)
    absent = subprocess.run(
        [pkgconfig_fixture_executable(), "--exists", "bullet-float64"],
        capture_output=True,
        text=True,
        env=env,
    )
    assert absent.returncode != 0, absent.stdout + absent.stderr
    options, flags = pkgconfig_consumer_options(
        "bullet-selection", static=True, env=env
    )
    consumer = tmp_path / "pc-consumer"
    consumer.mkdir()
    result = configure(
        consumer,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        f'add_executable(consumer "{source.as_posix()}")\n' + options,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    cmake_fixture_run(shutil.which("cmake"), "--build", consumer / "build")
    cmake_fixture_run(consumer / "build" / executable)
    assert all(str(wrong) not in flag for flag in flags)
