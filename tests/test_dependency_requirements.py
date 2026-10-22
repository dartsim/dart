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


@pytest.mark.parametrize("missing", ["tinyxml2", "urdfdom", "sdformat"])
def test_io_requires_all_parser_dependencies(tmp_path, missing):
    if missing != "tinyxml2":
        config_package(tmp_path, "tinyxml2", "9.0.0", "tinyxml2::tinyxml2")
    if missing == "sdformat":
        config_package(tmp_path, "urdfdom", "3.0.0", "urdfdom")
    result = configure(
        tmp_path,
        "include(DARTMacros)\n"
        f"set(CMAKE_DISABLE_FIND_PACKAGE_{missing} TRUE)\n"
        f'include("{(ROOT / "dart/utils/CMakeLists.txt").as_posix()}")\n',
    )
    assert result.returncode != 0
    assert f"dart-io requires {missing}" in result.stdout + result.stderr
    assert "DART_BUILD_IO=OFF" in result.stdout + result.stderr


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
        # Bullet libraries that this project builds cannot be linked by the
        # precision probe; their directories show the precision instead.
        (
            325,
            'file(WRITE "${CMAKE_BINARY_DIR}/bt/dummy.c" "int bt_dummy;\\n")\n'
            'file(WRITE "${CMAKE_BINARY_DIR}/bt/CMakeLists.txt"\n'
            '  "add_definitions(-DBT_USE_DOUBLE_PRECISION)\\n'
            "add_library(BulletCollision STATIC dummy.c)\\n"
            'add_library(helper STATIC dummy.c)\\n")\n'
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\n"
            "enable_language(C)\n"
            "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)\n"
            'add_subdirectory("${CMAKE_BINARY_DIR}/bt" bt-build)\n'
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
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "FindBullet.cmake").write_text(
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        f'set(BULLET_INCLUDE_DIRS "{include.as_posix()}")\n'
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


def bullet_module(tmp_path, version=325):
    include = tmp_path / "discovered"
    (include / "LinearMath").mkdir(parents=True)
    (include / "LinearMath/btScalar.h").write_text(
        f"#define BT_BULLET_VERSION {version}\n"
    )
    modules = tmp_path / "modules"
    modules.mkdir(exist_ok=True)
    (modules / "FindBullet.cmake").write_text(
        "set(BULLET_FOUND TRUE)\nset(Bullet_FOUND TRUE)\n"
        f'set(BULLET_INCLUDE_DIRS "{include.as_posix()}")\n'
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


def pkgconfig_consumer_options(package, *, static=False, env=None):
    flags = []
    content = ""
    for kind in ("--cflags", "--libs"):
        result = cmake_fixture_run(
            shutil.which("pkg-config"),
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
    include = bullet_module(tmp_path)
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
        include = bullet_module(consumer, 325 if selected_version == 305 else 305)
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
        "if(MSVC)\nset(strict_flags /WX /W4 /we4244)\n"
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
    pc_library = (
        "debug_math"
        if edge == "mapped-target" or configuration == "Debug"
        else "release_math"
    )
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


@pytest.mark.parametrize("configuration", ["Release", "Debug"])
@pytest.mark.parametrize("layout", ["plain", "spaces"])
def test_bullet_named_libraries_pkgconfig_consumer(tmp_path, configuration, layout):
    folder = "bullet with spaces" if layout == "spaces" else "bullet"
    prefix = build_bullet_abi_fixture(tmp_path / folder, "double", "STATIC", "-float64")
    bullet_module(tmp_path)
    source = tmp_path / "consumer.cpp"
    source.write_text(BULLET_CONSUMER)
    graph = (
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\nenable_language(CXX)\n"
        "add_library(Bullet INTERFACE IMPORTED)\n"
        f'target_include_directories(Bullet INTERFACE "{prefix.as_posix()}/include")\n'
        f'target_link_directories(Bullet INTERFACE "$<$<CONFIG:{configuration}>:{prefix.as_posix()}/lib>" '
        '"$<$<CONFIG:Unused>:/missing/bullet>")\n'
        "target_link_libraries(Bullet INTERFACE BulletCollision-float64 LinearMath-float64)\n"
        f'add_executable(consumer "{source.as_posix()}")\n'
        "target_compile_definitions(consumer PRIVATE BT_USE_DOUBLE_PRECISION)\n"
        "target_link_libraries(consumer PRIVATE Bullet)\n"
    )
    result = configure(
        tmp_path,
        graph + "include(DARTFindBullet)\n_dart_bullet_probe_precision(precision)\n"
        'if(NOT BULLET_FOUND OR NOT precision STREQUAL "double")\n'
        'message(FATAL_ERROR "Named libraries failed ABI validation")\nendif()\n'
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
    assert "-L" + str(prefix / "lib") in flags
    assert all("/missing/bullet" not in flag for flag in flags)
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
def test_bullet_transitive_built_precision(tmp_path, mode):
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
        'if(NOT after_languages STREQUAL before_languages OR NOT after_languages STREQUAL "C"\n'
        ' OR NOT CMAKE_C_COMPILER STREQUAL before_c OR NOT "${CMAKE_CXX_COMPILER}" STREQUAL before_cxx\n'
        ' OR NOT "${CMAKE_CXX_COMPILER_LOADED}" STREQUAL before_cxx_loaded\n'
        ' OR NOT CMAKE_TRY_COMPILE_TARGET_TYPE STREQUAL "STATIC_LIBRARY"\n'
        ' OR NOT CMAKE_REQUIRED_FLAGS STREQUAL "caller-flags"\n'
        ' OR NOT CMAKE_REQUIRED_LIBRARIES STREQUAL "caller-library")\n'
        'message(FATAL_ERROR "C-only discovery leaked caller state")\nendif()\n'
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
    cmake_fixture_run(shutil.which("cmake"), "--install", fixture / "build")
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
