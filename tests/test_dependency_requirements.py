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
