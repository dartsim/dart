"""Exercise DART's installed dependency finders with isolated CMake packages."""

import hashlib
import os
import re
import shutil
import subprocess
import tarfile
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def configure(tmp_path, content, *, cmake_args=()):
    source = tmp_path / "source"
    source.mkdir(exist_ok=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(DependencyRequirements NONE)\n"
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
            shutil.which("cmake"),
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
        "add_component_dependency_packages(dart fixture Eigen3 ODE)\n"
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
    for name in ("DARTFindODE.cmake", "FindODE.cmake"):
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
        config_package(tmp_path, "tinyxml2", "9.0.0", "tinyxml2::tinyxml2")
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


@pytest.mark.parametrize(
    "package,minimum,target,required",
    [
        ("fmt", "8.1.1", "fmt::fmt", False),
        ("Eigen3", "3.4.0", "Eigen3::Eigen", True),
        ("fcl", "0.7.0", "fcl", True),
        ("octomap", "1.9.7", "octomap", False),
        ("spdlog", "1.9.2", "spdlog::spdlog", False),
        ("tinyxml2", "9.0.0", "tinyxml2::tinyxml2", False),
        ("urdfdom", "3.0.0", "urdfdom", False),
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
        versions["older"] = "8.0.0"
    if package == "urdfdom" and version_kind == "older":
        versions["older"] = "2.3.5"
    if package in ("tinyxml2", "urdfdom") and version_kind == "newer":
        versions["newer"] = f"{major + 1}.0.0"
    extra = ""
    if package == "ODE":
        extra = "set_target_properties(ODE::ODE PROPERTIES INTERFACE_COMPILE_DEFINITIONS dLIBCCD_BOX_CYL)\n"
    config_package(tmp_path, package, versions[version_kind], target, extra)
    accepted = version_kind in ("minimum", "newer")
    result = configure(
        tmp_path,
        f"set(DART_USE_SYSTEM_FMT ON)\nset(DART_USE_SYSTEM_ODE ON)\n"
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


@pytest.mark.parametrize(
    "libdir,pcdir",
    [
        ("lib", "lib/pkgconfig"),
        ("lib", "libdata/pkgconfig"),
        ("lib/x86_64-linux-gnu", "lib/x86_64-linux-gnu/pkgconfig"),
    ],
)
@pytest.mark.parametrize(
    "metadata", ["3.0.2", "3.0.0", "2.3.5", "invalid", "unrelated", None]
)
def test_urdfdom_without_cmake_version(tmp_path, monkeypatch, libdir, pcdir, metadata):
    config_package(tmp_path, "urdfdom", None)
    prefix = tmp_path / "prefix"
    config_dir = prefix / libdir / "urdfdom/cmake"
    config_dir.parent.mkdir(parents=True, exist_ok=True)
    (prefix / "lib/cmake/urdfdom").rename(config_dir)
    if metadata is not None:
        pc_prefix = tmp_path / "prefix-other" if metadata == "unrelated" else prefix
        pc_file = pc_prefix / pcdir / "urdfdom.pc"
        pc_file.parent.mkdir(parents=True)
        pc_file.write_text(
            f"prefix={pc_prefix.as_posix()}\n"
            "Name: urdfdom\nDescription: URDF parser\n"
            f"Version: {'3.0.2' if metadata == 'unrelated' else metadata}\n"
        )
        monkeypatch.setenv("PKG_CONFIG_PATH", str(pc_file.parent))
    accepted = metadata in ("3.0.2", "3.0.0")
    result = configure(
        tmp_path,
        f'set(CMAKE_INCLUDE_PATH "{(tmp_path / "prefix-other" / pcdir).as_posix()}")\n'
        "include(DARTFindurdfdom)\n"
        f"if({'NOT urdfdom_FOUND' if accepted else 'urdfdom_FOUND OR URDFDOM_FOUND'})\n"
        'message(FATAL_ERROR "Incorrect urdfdom version acceptance")\nendif()\n'
        + (
            f'if(NOT TARGET urdfdom OR NOT urdfdom_VERSION STREQUAL "{metadata}")\n'
            'message(FATAL_ERROR "Missing urdfdom version or target")\nendif()\n'
            if accepted
            else ""
        ),
        cmake_args=(f"-Durdfdom_DIR={config_dir}/../cmake",),
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    if not accepted:
        assert "urdfdom >= 3.0.0 is required" in output


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
        f'set(BULLET_INCLUDE_DIRS "{include.as_posix()}")\n'
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


@pytest.mark.parametrize(
    "package,minimum,header,defines,target",
    [
        (
            "tinyxml2",
            "9.0.0",
            "tinyxml2.h",
            "#define TINYXML2_MAJOR_VERSION 9\n#define TINYXML2_MINOR_VERSION 0\n#define TINYXML2_PATCH_VERSION 0\n",
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
        extra += "set_target_properties(ODE::ODE PROPERTIES INTERFACE_COMPILE_DEFINITIONS dLIBCCD_BOX_CYL)\n"
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
        ("tinyxml2", "9.0.0", "TINYXML2", "tinyxml2.h", "TINYXML2"),
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
        + f"find_package({package} {minimum} QUIET MODULE)\n"
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


@pytest.mark.parametrize("version", ["7.1.3", "8.1.1", None])
@pytest.mark.parametrize(
    "version_source", ["metadata", "core.h", "base.h", "build-interface"]
)
def test_parent_fmt_target_version(tmp_path, version, version_source):
    include = tmp_path / "fmt-parent" / "include"
    header = (
        include
        / "fmt"
        / ("base.h" if version_source == "build-interface" else version_source)
    )
    header.parent.mkdir(parents=True)
    if version and version_source != "metadata":
        major, minor, patch = map(int, version.split("."))
        header.write_text(
            f"#define FMT_VERSION {major * 10000 + minor * 100 + patch}\n"
        )
    include_path = (
        f"$<BUILD_INTERFACE:{include.as_posix()}>"
        if version_source == "build-interface"
        else include.as_posix()
    )
    content = (
        "set(DART_USE_SYSTEM_FMT OFF)\n"
        "add_library(fmt::fmt INTERFACE IMPORTED)\n"
        f'set_target_properties(fmt::fmt PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "{include_path}")\n'
    )
    if version and version_source == "metadata":
        content += f"set(fmt_VERSION {version})\n"
    result = configure(tmp_path, content + "include(DARTFindfmt)\n")
    if version == "8.1.1":
        assert result.returncode == 0, result.stdout + result.stderr
    else:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "fmt >= 8.1.1" in result.stdout + result.stderr


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
