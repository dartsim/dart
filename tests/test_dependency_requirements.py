"""Exercise DART's installed dependency finders with isolated CMake packages."""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def configure(tmp_path, content, *, cmake_args=()):
    source = tmp_path / "source"
    source.mkdir(exist_ok=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\n"
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


def test_component_exports_only_install_owned_dependency_finders(tmp_path):
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
    config_package(consumer, "Eigen3", "3.4.0")
    result = configure(
        consumer,
        f'set(CMAKE_MODULE_PATH "{installed.as_posix()}")\n'
        "find_package(Eigen3 REQUIRED)\n"
        "include(DARTFindEigen3)\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert not (installed / "FindEigen3.cmake").exists()
    for name in ("DARTFindEigen3.cmake", "DARTFindODE.cmake", "FindODE.cmake"):
        assert (installed / name).read_text() == (ROOT / "cmake" / name).read_text()


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


@pytest.mark.parametrize("version", ["3.4.0", "3.4.1", "5.0.1"])
def test_eigen_accepts_minor_scoped_version_files(tmp_path, version):
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
    assert result.returncode == 0, result.stdout + result.stderr


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
@pytest.mark.parametrize("version", [305, 306, 325, None])
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
    if version and version >= 306:
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
        ("assimp", "5.2.2", "ASSIMP", "assimp/scene.h", None),
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
