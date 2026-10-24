"""Exercise DART's installed dependency finders with isolated CMake packages."""

import hashlib
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CMAKE = os.environ.get("DART_TEST_CMAKE") or shutil.which("cmake")


def configure(tmp_path, content, *, cmake_args=(), language="NONE"):
    source = tmp_path / "source"
    source.mkdir(exist_ok=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\n"
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
        ("imgui", "1.91.9", "imgui::imgui", True),
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
        )
        + "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\n"
        "enable_language(CXX)\n"
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)\n"
        "include(DARTMacros)\ninclude(Components)\n"
        "initialize_component_helpers(${PROJECT_NAME})\n"
        "set(CONFIG_INSTALL_DIR cmake)\nset(LIBRARY_INSTALL_DIR lib)\n"
        "set(DART_MAJOR_VERSION 6)\nset(DART_MINOR_VERSION 20)\n"
        "set(DART_VERSION 6.20.1)\n"
        'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/utils.cpp" "")\n'
        'add_library(dart-utils STATIC "${CMAKE_CURRENT_BINARY_DIR}/utils.cpp")\n'
        "add_component(${PROJECT_NAME} utils)\n"
        "add_component_targets(${PROJECT_NAME} utils dart-utils)\n"
        f'add_subdirectory("{(ROOT / "dart/utils/urdf").as_posix()}" urdf)\n'
        + (
            "if(NOT TARGET ${PROJECT_NAME}-utils-urdf OR NOT TARGET ${PROJECT_NAME}_component_utils-urdf)\n"
            if accepted
            else "if(TARGET ${PROJECT_NAME}-utils-urdf OR TARGET ${PROJECT_NAME}_component_utils-urdf)\n"
        )
        + 'message(FATAL_ERROR "Incorrect URDF component acceptance")\nendif()\n',
        cmake_args=(f"-Durdfdom_DIR={config_dir}/../cmake",),
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    if not accepted:
        assert "urdfdom >= 3.0.0 is required" in output
        assert "Looking for urdfdom >= 3.0.0 - NOT found" in output


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


@pytest.mark.parametrize("version,accepted", [("1.91.9b", True), ("1.90.9b", False)])
def test_imgui_release_suffix_floor(tmp_path, version, accepted):
    config_package(tmp_path, "imgui", version, "imgui::imgui")
    result = configure(tmp_path, "include(DARTFindimgui)\n")
    assert (result.returncode == 0) == accepted, result.stdout + result.stderr
    if not accepted:
        assert "ImGui >= 1.91.9" in result.stderr


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
            "1.91.9",
            "imgui.h",
            '#define IMGUI_VERSION "1.91.9"\n',
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
        ("assimp", "5.2.0", "ASSIMP", "assimp/scene.h", None),
        ("fcl", "0.7.0", "FCL", "fcl/config.h", "FCL_VERSION"),
        ("tinyxml2", "9.0.0", "TINYXML2", "tinyxml2.h", "TINYXML2"),
        ("ODE", "0.16.2", "ODE", "ode/version.h", "dODE_VERSION"),
        ("imgui", "1.91.9", "imgui", "imgui.h", "IMGUI_VERSION"),
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
        variables += f'set(imgui_INCLUDE_DIR "{include.as_posix()}")\nset(imgui_backends_INCLUDE_DIR "{include.as_posix()}")\n'
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
    "version,accepted", [("6.0.5", True), ("5.2.0", True), ("5.1.0", False)]
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
        tmp_path, "assimp", "5.2.0", None, 'message(FATAL_ERROR "Loaded config")\n'
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
        lookup + "set(assimp_VERSION 5.2.0)\n"
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


@pytest.mark.parametrize("languages", ["C", "CXX", "C;CXX"])
@pytest.mark.parametrize("has_macro", [0, 1])
def test_ode_libccd_probe_uses_an_enabled_language(tmp_path, languages, has_macro):
    # The probe runs during find_package(DART COMPONENTS collision-ode), so it
    # must work in consumer projects that enable only C++. DART reads the cached
    # result outside the finder's scope, where its local default is not visible.
    include = tmp_path / "ode-config" / "include"
    header = include / "ode" / "odeconfig.h"
    header.parent.mkdir(parents=True)
    header.write_text(
        "#define dLIBCCD_BOX_CYL\n" if has_macro else "/* libccd disabled */\n"
    )
    config_package(
        tmp_path,
        "ODE",
        "0.16.2",
        "ODE::ODE",
        extra="set_target_properties(ODE::ODE PROPERTIES "
        f'INTERFACE_INCLUDE_DIRECTORIES "{include.as_posix()}")\n',
    )
    result = configure(
        tmp_path,
        "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH TRUE)\n"
        + "".join(f"enable_language({language})\n" for language in languages.split(";"))
        + "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)\n"
        "include(DARTFindODE)\n"
        "if(NOT ODE_FOUND OR NOT "
        f'"$CACHE{{DART_ODE_HAS_LIBCCD_BOX_CYL}}" EQUAL {has_macro})\n'
        'message(FATAL_ERROR "Unexpected ODE libccd box-cylinder probe: '
        '$CACHE{DART_ODE_HAS_LIBCCD_BOX_CYL}")\nendif()\n',
    )
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
        cmake_fixture_run(CMAKE, "--build", tmp_path / "build")


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
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")


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
        CMAKE,
        "-G",
        "Ninja",
        "-S",
        source,
        "-B",
        tmp_path / "build",
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
    )
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")
    cmake_fixture_run(CMAKE, "--install", tmp_path / "build")
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


def install_bullet_dart_fixture(
    tmp_path, prefix, linkage="SHARED", suffix="", pc_settings="", discovery_version=305
):
    """Install the production component and probe output with a public dependency."""
    source = tmp_path / "source"
    source.mkdir(parents=True)
    stage = tmp_path / "install"
    (source / "empty_core.cpp").write_text("int dart_core_fixture() { return 0; }\n")
    (source / "core.cpp").write_text(
        BULLET_CONSUMER.replace("int main()", "int dart_fixture()")
    )
    backend = (
        (ROOT / "dart/collision/bullet/CMakeLists.txt")
        .read_text()
        .split("# Search all header and source files")[0]
    )
    (source / "backend.cmake").write_text(backend)
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
        f"add_library(dart-collision-bullet {linkage} core.cpp)\n"
        f"add_library(dart {linkage} empty_core.cpp)\n"
        'target_link_libraries(dart-collision-bullet PUBLIC "$<BUILD_INTERFACE:Bullet>" "$<INSTALL_INTERFACE:$<1:Bullet>>")\n'
        "if(BT_USE_DOUBLE_PRECISION)\ntarget_compile_definitions(dart-collision-bullet PUBLIC BT_USE_DOUBLE_PRECISION)\nendif()\n"
        "set(PROJECT_NAME_UPPERCASE DART)\n"
        f'set(DART_SOURCE_DIR "{ROOT.as_posix()}")\n'
        "set(CONFIG_INSTALL_DIR lib/cmake/DART)\nset(LIBRARY_INSTALL_DIR lib)\n"
        "include(Components)\ninitialize_component_helpers(dart)\n"
        "add_component(dart dart)\nadd_component_targets(dart dart dart)\n"
        "add_component(dart collision-bullet)\nadd_component_targets(dart collision-bullet dart-collision-bullet)\nadd_component_dependency_packages(dart collision-bullet Bullet)\ninstall_component_exports(dart)\n"
        f'install(FILES "{ROOT.as_posix()}/cmake/DARTFindPackageVersion.cmake" DESTINATION lib/cmake/DART)\n'
    )
    cmake_fixture_run(
        CMAKE,
        "-G",
        "Ninja",
        "-S",
        source,
        "-B",
        tmp_path / "build",
        f"-DCMAKE_INSTALL_PREFIX={stage}",
    )
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")
    cmake_fixture_run(CMAKE, "--install", tmp_path / "build")
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
    component = stage / "lib/cmake/DART/dart_collision-bulletComponent.cmake"
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
            "get_property(found DIRECTORY PROPERTY DART_collision-bullet_FOUND)\n"
            + (
                'if(NOT found)\nmessage(FATAL_ERROR "Matching ABI rejected")\nendif()\n'
                f'include("{stage.as_posix()}/lib/cmake/DART/dart_collision-bulletTargets.cmake")\n'
                "add_executable(consumer main.cpp)\ntarget_link_libraries(consumer PRIVATE dart-collision-bullet)\n"
                if accepted
                else 'if(found OR BULLET_FOUND OR Bullet_FOUND)\nmessage(FATAL_ERROR "Unverified or mismatching ABI accepted")\nendif()\n'
            )
        )
        cmake_fixture_run(CMAKE, "-G", "Ninja", "-S", source, "-B", source / "build")
        if accepted:
            cmake_fixture_run(CMAKE, "--build", source / "build")
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


@pytest.mark.parametrize("version,floor", [(305, "3.05"), (325, "3.17")])
def test_bullet_installed_selected_version(tmp_path, version, floor):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "single", version=version)
    # Unrelated discovery deliberately advertises the opposite header version.
    stage = install_bullet_dart_fixture(
        tmp_path / "dart", prefix, discovery_version=325 if version == 305 else 305
    )
    component = stage / "lib/cmake/DART/dart_collision-bulletComponent.cmake"
    assert f'set(DART_BULLET_REQUIRE_VERSION "{floor}")' in component.read_text()
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
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")


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
        cmake_fixture_run(CMAKE, "--build", case / "build")


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
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")
    component = (tmp_path / "build/component.cmake").read_text()
    assert 'set(DART_BULLET_REQUIRE_PRECISION "double")' in component


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
            cmake_fixture_run(CMAKE, "--build", tmp_path / "build", "--config", config)


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
            cmake_fixture_run(CMAKE, "--build", tmp_path / "build", "--config", config)
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
        cmake_fixture_run(CMAKE, "--build", control / "build")
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
            cmake_fixture_run(CMAKE, "--build", tmp_path / "build", "--config", config)
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
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")


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
        [CMAKE, "--build", str(control / "build")],
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
    cmake_fixture_run(CMAKE, "--build", pc_consumer / "build")
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
        "link-only-new",
        "interface-link-only",
        "interface-link-only-new",
        "mixed",
        "definition-active",
        "definition-inactive",
    ],
)
@pytest.mark.parametrize("carrier", ["definition", "option"])
def test_bullet_transitive_built_precision(tmp_path, mode, carrier):
    legacy_link_only = tuple(
        map(
            int,
            re.search(
                r"cmake version (\d+)\.(\d+)",
                cmake_fixture_run(CMAKE, "--version").stdout,
            ).groups(),
        )
    ) < (3, 24)
    direct_link_only = mode in ("link-only", "link-only-new")
    interface_link_only = mode in ("interface-link-only", "interface-link-only-new")
    if mode.endswith("link-only-new") and legacy_link_only:
        pytest.skip(
            "CMP0131 NEW requires CMake 3.24; legacy behavior is covered separately"
        )
    unverified = direct_link_only and not legacy_link_only
    configuration = (
        "Debug" if mode in ("config-active", "definition-active") else "Release"
    )
    double = mode not in (
        "config-inactive",
        "definition-inactive",
        "mapped-empty",
        "link-only-new",
        "interface-link-only",
        "interface-link-only-new",
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
    if (direct_link_only or interface_link_only) and not legacy_link_only:
        owner_policy = "NEW" if mode.endswith("-new") else "OLD"
        graph += f"cmake_policy(SET CMP0131 {owner_policy})\n"
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
    elif direct_link_only:
        edge = "$<LINK_ONLY:precision_options>"
    elif interface_link_only:
        graph += (
            "add_library(interface_options INTERFACE)\n"
            'target_link_libraries(interface_options INTERFACE "$<LINK_ONLY:precision_options>")\n'
        )
        edge = "interface_options"
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
    cmake_fixture_run(CMAKE, "--build", control / "build")
    (source / "backend.cmake").write_text(
        (ROOT / "dart/collision/bullet/CMakeLists.txt")
        .read_text()
        .split("# Search all header and source files")[0]
    )
    tail = "include(DARTFindBullet)\n_dart_bullet_probe_precision(precision)\n"
    if (direct_link_only or interface_link_only) and not legacy_link_only:
        # The finder must not infer the owner policy from its own include scope.
        tail = (
            f"cmake_policy(SET CMP0131 {'OLD' if owner_policy == 'NEW' else 'NEW'})\n"
            + tail
        )
    if mode == "mixed" or unverified:
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
    if mode != "mixed" and not unverified:
        cmake_fixture_run(CMAKE, "--build", tmp_path / "build")
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
        CMAKE,
        "--build",
        control / "build",
        "--target",
        "BulletCollision",
        "LinearMath",
    )
    result = subprocess.run(
        [
            CMAKE,
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
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")


@pytest.mark.parametrize("scope", ["direct", "function"])
@pytest.mark.parametrize("abi", ["same", "opposite", "mixed"])
def test_bullet_c_only_installed_discovery(tmp_path, scope, abi):
    prefix = build_bullet_abi_fixture(tmp_path / "bullet", "double", "STATIC")
    other = build_bullet_abi_fixture(tmp_path / "other", "single", "STATIC")
    stage = install_bullet_dart_fixture(tmp_path / "dart", prefix, "STATIC")
    (stage / "include").mkdir()
    with (tmp_path / "dart/source/CMakeLists.txt").open("a") as f:
        f.write(
            "set(DART_USE_SYSTEM_IMGUI ON)\nset(INCLUDE_INSTALL_DIR include)\nset(DART_MAJOR_VERSION 6)\n"
            "include(CMakePackageConfigHelpers)\n"
            f'configure_package_config_file("{ROOT.as_posix()}/cmake/DARTConfig.cmake.in" '
            '"${CMAKE_BINARY_DIR}/DARTConfig.cmake" INSTALL_DESTINATION lib/cmake/DART '
            "PATH_VARS INCLUDE_INSTALL_DIR)\n"
            'install(FILES "${CMAKE_BINARY_DIR}/DARTConfig.cmake" DESTINATION lib/cmake/DART)\n'
        )
    cmake_fixture_run(
        CMAKE,
        "-S",
        tmp_path / "dart/source",
        "-B",
        tmp_path / "dart/build",
    )
    cmake_fixture_run(CMAKE, "--install", tmp_path / "dart/build")
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
        f'find_package(DART CONFIG {"REQUIRED" if abi == "same" else "QUIET"} COMPONENTS collision-bullet PATHS "{stage.as_posix()}/lib/cmake/DART" NO_DEFAULT_PATH)\n'
        + f'if({"NOT " if abi == "same" else ""}DART_FOUND)\n'
        'message(FATAL_ERROR "Wrong C-only installed ABI acceptance")\nendif()\n'
    )
    if scope == "function":
        discovery = "function(discover)\n" + discovery + "endfunction()\ndiscover()\n"
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\nproject(Consumer LANGUAGES C)\n"
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
    cmake_fixture_run(CMAKE, "-G", "Ninja", "-S", source, "-B", source / "build")
    cmake_fixture_run(CMAKE, "--build", source / "build")
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
        CMAKE,
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
    cmake_fixture_run(CMAKE, "--build", fixture / "build")
    # Install compares rounded timestamps, so a same-second rebuild can leave
    # the original archive behind. This prefix belongs only to this fixture.
    shutil.rmtree(prefix)
    cmake_fixture_run(CMAKE, "--install", fixture / "build")
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
    cmake_fixture_run(CMAKE, "--build", control / "build")
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
            [CMAKE, "--build", str(negative / "build")],
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
        'if(CMAKE_PROJECT_NAME STREQUAL "DARTBulletABI" AND POLICY CMP0141)\n'
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
    version = cmake_fixture_run(CMAKE, "--version").stdout
    if tuple(map(int, re.search(r"cmake version (\d+)\.(\d+)", version).groups())) < (
        3,
        25,
    ):
        pytest.skip("The per-target SYSTEM property requires CMake 3.25")
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
            [CMAKE, "--build", str(control / "build")],
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
    cmake_fixture_run(CMAKE, "--build", tmp_path / "build")
    cmake_fixture_run(
        tmp_path / ("build/consumer.exe" if os.name == "nt" else "build/consumer")
    )


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
