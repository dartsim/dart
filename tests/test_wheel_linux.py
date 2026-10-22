"""Guard the Linux wheel ABI policy and the platform workflow routing."""

import importlib.util
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_wheel_matrix_and_platform_routing():
    workflow = yaml.load(
        (ROOT / ".github/workflows/publish_dartpy.yml").read_text(),
        Loader=yaml.BaseLoader,
    )
    job = workflow["jobs"]["build_wheels"]
    rows = job["strategy"]["matrix"]["include"]
    linux = [row for row in rows if row["os"] == "linux"]
    # One cp312-abi3 wheel per platform serves Python 3.12 and newer.
    assert {row["build"] for row in linux} == {
        f"cp312-abi3-manylinux_{arch}" for arch in ("x86_64", "aarch64")
    }
    for row in rows:
        assert "-cp312-abi3-" in f"-{row['build']}"
        assert row["abi3"] == "true"
        assert row["release_only"] == "false"
    for row in linux:
        assert row["runner"] == (
            "ubuntu-24.04-arm" if "aarch64" in row["build"] else "ubuntu-24.04"
        )
    assert [(row["runner"], row["build"]) for row in rows if row["os"] != "linux"] == [
        ("macos-15", "cp312-abi3-macosx_arm64"),
        ("windows-2025-vs2026", "cp312-abi3-win_amd64"),
    ]
    steps = {step.get("name"): step for step in job["steps"]}
    assert "matrix.os == 'linux'" in steps["Build and test manylinux wheel"]["if"]
    assert "scripts/wheel_linux.py" in steps["Build and test manylinux wheel"]["run"]
    for name in (
        "Install Pixi wheel environment",
        "Build wheel",
        "Repair wheel",
        "Verify wheel",
        "Test wheel",
        "Audit and test abi3 wheel on Python 3.13",
    ):
        assert "matrix.os != 'linux'" in steps[name]["if"]


def test_linux_driver_repairs_and_tests_in_clean_base(monkeypatch, tmp_path):
    python_tag = "cp312"
    spec = importlib.util.spec_from_file_location(
        "wheel_linux", ROOT / "scripts/wheel_linux.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    monkeypatch.setattr(module, "__file__", str(tmp_path / "scripts/wheel_linux.py"))
    monkeypatch.setattr(module.platform, "machine", lambda: "aarch64")
    monkeypatch.setattr("sys.argv", ["wheel_linux.py", python_tag])
    monkeypatch.setattr(
        module.subprocess, "run", lambda cmd, **kwargs: calls.append((cmd, kwargs))
    )
    module.main()
    build, repair, *smokes = [cmd for cmd, _ in calls]
    assert "BASE_IMAGE=quay.io/pypa/manylinux_2_28_aarch64" in build
    assert "--plat manylinux_2_28_aarch64 --only-plat" in repair[-1]
    assert f"/opt/python/{python_tag}-{python_tag}/bin/python" in repair[-1]
    assert "--exclude=build" in repair[-1]
    assert "abi3audit --strict" in repair[-1]
    assert "scikit-build-core" in repair[-1]
    assert "setuptools" not in repair[-1]
    # The abi3 wheel is also tested on a newer interpreter.
    smoke_tags = [python_tag, "cp313"]
    assert len(smokes) == len(smoke_tags)
    for smoke, tag in zip(smokes, smoke_tags):
        assert "quay.io/pypa/manylinux_2_28_aarch64" in smoke
        assert "dartpy-manylinux-2-28-aarch64" not in smoke
        assert "LD_LIBRARY_PATH" in smoke
        assert f"/opt/python/{tag}-{tag}/bin/python" in smoke
        assert smoke[-1].endswith(f"{python_tag}-*-manylinux_2_28_aarch64.whl")
    assert all(kwargs == {"check": True} for _, kwargs in calls)


@pytest.mark.parametrize("python_tag", ["cp311", "cp313", "cp314"])
def test_linux_driver_rejects_unsupported_python(monkeypatch, python_tag):
    spec = importlib.util.spec_from_file_location(
        "wheel_linux", ROOT / "scripts/wheel_linux.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr("sys.argv", ["wheel_linux.py", python_tag])
    with pytest.raises(SystemExit, match="2"):
        module.main()


def test_native_dependency_image_preserves_collision_backends():
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    assert "quay.io/pypa/manylinux_2_28_x86_64" in dockerfile
    assert "-DUSE_DOUBLE_PRECISION=ON" in dockerfile
    assert "-DENABLE_DOUBLE_PRECISION=ON" in dockerfile
    for setting in (
        "-DODE_DOUBLE_PRECISION=ON",
        "-DODE_WITH_LIBCCD=ON",
        "-DODE_WITH_LIBCCD_SYSTEM=ON",
        "-DODE_WITH_LIBCCD_BOX_CYL=OFF",
    ):
        assert setting in dockerfile
    assert "sha256sum -c -" in dockerfile
    assert "v6.0.5" in dockerfile
    # nanobind is fetched by CMake; the image no longer needs pybind11.
    assert "pybind11" not in dockerfile


def test_dependency_image_fetches_archives_without_git_clones():
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    assert "git clone" not in dockerfile


def test_dependency_image_builds_sdformat_against_urdfdom_6():
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    headers = "ros/urdfdom_headers/archive/refs/tags/3.0.1.tar.gz"
    parser = "ros/urdfdom/archive/refs/tags/6.0.1.tar.gz"
    assert headers in dockerfile
    assert parser in dockerfile
    # The trailing newline skips the "# sdformat embeds ..." comment above gz-cmake.
    sdformat = dockerfile.index("# sdformat\n")
    assert (
        dockerfile.index("# tinyxml2")
        < dockerfile.index("# console_bridge")
        < dockerfile.index(headers)
        < dockerfile.index(parser)
        < sdformat
    )
    assert (
        "-DUSE_INTERNAL_URDF=OFF" in dockerfile[sdformat : dockerfile.index("# Bullet")]
    )


def write_nanobind_package(package):
    """Write a complete fake nanobind 3.1.x package at `package`."""
    (package / "nanobindConfig.cmake").write_text(
        f'set(NB_DIR "{package.as_posix()}")\n'
    )
    (package / "include/nanobind").mkdir(parents=True)
    (package / "include/nanobind/nanobind.h").write_text(
        "#define NB_VERSION_MAJOR 3\n"
        "#define NB_VERSION_MINOR 1\n"
        "#define NB_VERSION_PATCH 0\n"
    )
    (package / "src").mkdir()
    (package / "src/nb_internals.h").write_text("#  define NB_INTERNALS_VERSION 22\n")
    (package / "nanobindConfigVersion.cmake").write_text(
        'set(PACKAGE_VERSION "3.1.0")\n' "set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
    )


@pytest.mark.parametrize("wheels", [True, False])
@pytest.mark.parametrize("stable_abi", [True, False])
def test_python_discovery_needs_no_embedding_library(tmp_path, wheels, stable_abi):
    selected_python = (tmp_path / "selected-python").as_posix()
    (tmp_path / "FindPython.cmake").write_text(
        "if(Python_FOUND)\n"
        '  message(FATAL_ERROR "Python discovered twice")\n'
        "endif()\n"
        'if(NOT Python_FIND_VERSION STREQUAL "3.12")\n'
        '  message(FATAL_ERROR "Python floor was not requested")\n'
        "endif()\n"
        'if(NOT "Interpreter" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Interpreter was not requested")\n'
        "endif()\n"
        'if("Development" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Embedding library unavailable")\n'
        "endif()\n"
        'if(NOT "Development.Module" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Extension headers were not requested")\n'
        "endif()\n"
        'if(DART_DARTPY_STABLE_ABI AND NOT "Development.SABIModule" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Stable ABI headers were not requested")\n'
        "endif()\n"
        "set(Python_FOUND TRUE)\n"
        'if(NOT Python_EXECUTABLE OR Python_EXECUTABLE STREQUAL "legacy-python")\n'
        f'  set(Python_EXECUTABLE "{selected_python}")\n'
        "endif()\n"
    )
    (tmp_path / "FindPython3.cmake").write_text(
        'message(FATAL_ERROR "Duplicate Python3 discovery")\n'
    )
    write_nanobind_package(tmp_path)
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(PythonComponents NONE)\n"
        "macro(dart_find_package)\nendmacro()\n"
        "macro(dart_check_required_package)\nendmacro()\n"
        "macro(dart_check_optional_package)\nendmacro()\n"
        "function(add_subdirectory)\nendfunction()\n"
        f'set(CMAKE_MODULE_PATH "{tmp_path.as_posix()}")\n'
        "set(DART_BUILD_DARTPY ON)\n"
        f'set(DART_BUILD_WHEELS {"ON" if wheels else "OFF"})\n'
        f'set(DART_DARTPY_STABLE_ABI {"ON" if stable_abi else "OFF"})\n'
        "set(DART_USE_SYSTEM_ODE ON)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        f'include("{ROOT.as_posix()}/cmake/DARTFindDependencies.cmake")\n'
        "if(NOT DEFINED Python3_EXECUTABLE)\n"
        '  set(Python3_EXECUTABLE "legacy-python")\n'
        "endif()\n"
        "set(DART_USE_SYSTEM_NANOBIND ON)\n"
        f'set(nanobind_DIR "{tmp_path.as_posix()}")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n'
        "if(NOT DEFINED expected_python)\n"
        f'  set(expected_python "{selected_python}")\n'
        "endif()\n"
        "get_property(selected CACHE Python3_EXECUTABLE PROPERTY VALUE)\n"
        'if(NOT selected STREQUAL "${expected_python}")\n'
        '  message(FATAL_ERROR "Selected interpreter was not cached")\n'
        "endif()\n"
        'if(NOT Python3_EXECUTABLE STREQUAL "${expected_python}")\n'
        '  message(FATAL_ERROR "Legacy interpreter shadows selected Python")\n'
        "endif()\n"
        "get_property(last_forced CACHE _DART_PYTHON3_EXECUTABLE PROPERTY VALUE)\n"
        'if(NOT last_forced STREQUAL "${expected_python}")\n'
        '  message(FATAL_ERROR "Last forced interpreter was not cached")\n'
        "endif()\n"
    )
    result = subprocess.run(
        ["cmake", "-S", str(tmp_path), "-B", str(tmp_path / "build")],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "will be ignored" not in result.stderr

    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(tmp_path),
            "-B",
            str(tmp_path / "build"),
            "-DPython3_EXECUTABLE=different-python",
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Python3_EXECUTABLE differs from Python_EXECUTABLE" in result.stderr
    assert "Set Python_EXECUTABLE to select the Python interpreter" in " ".join(
        result.stderr.split()
    )

    new_python = (tmp_path / "new-python").as_posix()
    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(tmp_path),
            "-B",
            str(tmp_path / "build"),
            f"-DPython_EXECUTABLE={new_python}",
            f"-Dexpected_python={new_python}",
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "will be ignored" not in result.stderr


def test_builtin_findpython_primary_persists_across_configures(tmp_path):
    write_nanobind_package(tmp_path)
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(BuiltinFindPython NONE)\n"
        "function(add_subdirectory)\nendfunction()\n"
        "set(DART_BUILD_DARTPY ON)\n"
        "set(DART_DARTPY_STABLE_ABI OFF)\n"
        "set(DART_USE_SYSTEM_NANOBIND ON)\n"
        f'set(nanobind_DIR "{tmp_path.as_posix()}")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/selected.txt" "${Python_EXECUTABLE}")\n'
    )
    build = tmp_path / "build"

    def configure(*args):
        return subprocess.run(
            ["cmake", "-S", str(tmp_path), "-B", str(build), *args],
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )

    def cached(name):
        for line in (build / "CMakeCache.txt").read_text().splitlines():
            if line.startswith(f"{name}:"):
                return line.split("=", 1)[1]
        return None

    result = configure()
    assert result.returncode == 0, result.stdout + result.stderr
    selected = (build / "selected.txt").read_text()
    assert Path(selected).is_file()
    for name in ("Python_EXECUTABLE", "Python3_EXECUTABLE", "_DART_PYTHON3_EXECUTABLE"):
        assert cached(name) == selected

    # A later alias must not seed an empty primary: the selection persists.
    result = configure("-DPython3_EXECUTABLE=/nonexistent/python")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Python3_EXECUTABLE differs from Python_EXECUTABLE" in result.stderr
    assert (build / "selected.txt").read_text() == selected
    for name in ("Python_EXECUTABLE", "Python3_EXECUTABLE", "_DART_PYTHON3_EXECUTABLE"):
        assert cached(name) == selected


def test_dartpy_disabled_does_not_discover_python(tmp_path):
    for name in ("Python", "Python3"):
        (tmp_path / f"Find{name}.cmake").write_text(
            f'message(FATAL_ERROR "Unexpected {name} discovery")\n'
        )
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(PythonDisabled NONE)\n"
        "macro(dart_find_package)\nendmacro()\n"
        "macro(dart_check_required_package)\nendmacro()\n"
        "macro(dart_check_optional_package)\nendmacro()\n"
        f'set(CMAKE_MODULE_PATH "{tmp_path.as_posix()}")\n'
        "set(DART_BUILD_DARTPY OFF)\n"
        "set(DART_USE_SYSTEM_ODE ON)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        f'include("{ROOT.as_posix()}/cmake/DARTFindDependencies.cmake")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n'
    )
    result = subprocess.run(
        ["cmake", "-S", str(tmp_path), "-B", str(tmp_path / "build")],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_smoke_helper_does_not_require_python_on_path(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location(
        "test_wheel", ROOT / "scripts/test_wheel.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    wheel = tmp_path / "dartpy.whl"
    wheel.touch()
    tested = []
    monkeypatch.setenv("PATH", "")
    monkeypatch.setattr(module, "test_wheel", tested.append)
    assert module.main(["test_wheel.py", str(wheel)]) == 0
    assert tested == [wheel]
