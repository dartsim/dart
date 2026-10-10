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
    assert {row["build"] for row in linux} == {
        f"cp{version}-manylinux_{arch}"
        for version in (310, 311, 312, 313)
        for arch in ("x86_64", "aarch64")
    }
    for row in linux:
        assert row["runner"] == (
            "ubuntu-24.04-arm" if "aarch64" in row["build"] else "ubuntu-24.04"
        )
        assert row["release_only"] == (
            "false" if row["build"].startswith("cp313-") else "true"
        )
    assert [(row["runner"], row["build"]) for row in rows if row["os"] != "linux"] == [
        ("macos-15", "cp313-macosx_arm64"),
        ("macos-15", "cp312-macosx_arm64"),
        ("windows-2025-vs2026", "cp313-win_amd64"),
        ("windows-2025-vs2026", "cp312-win_amd64"),
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
    ):
        assert "matrix.os != 'linux'" in steps[name]["if"]


def test_linux_driver_repairs_and_tests_in_clean_base(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location(
        "wheel_linux", ROOT / "scripts/wheel_linux.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    monkeypatch.setattr(module, "__file__", str(tmp_path / "scripts/wheel_linux.py"))
    monkeypatch.setattr(module.platform, "machine", lambda: "aarch64")
    monkeypatch.setattr("sys.argv", ["wheel_linux.py", "cp312"])
    monkeypatch.setattr(
        module.subprocess, "run", lambda cmd, **kwargs: calls.append((cmd, kwargs))
    )
    module.main()
    assert len(calls) == 3
    build, repair, smoke = [cmd for cmd, _ in calls]
    assert "BASE_IMAGE=quay.io/pypa/manylinux_2_28_aarch64" in build
    assert "--plat manylinux_2_28_aarch64 --only-plat" in repair[-1]
    assert "/opt/python/cp312-cp312/bin/python" in repair[-1]
    assert "--exclude=build" in repair[-1]
    assert "quay.io/pypa/manylinux_2_28_aarch64" in smoke
    assert "dartpy-manylinux-2-28-aarch64" not in smoke
    assert "LD_LIBRARY_PATH" in smoke
    assert smoke[-1].endswith("cp312-*-manylinux_2_28_aarch64.whl")
    assert all(kwargs == {"check": True} for _, kwargs in calls)


def test_linux_driver_rejects_unsupported_python(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "wheel_linux", ROOT / "scripts/wheel_linux.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr("sys.argv", ["wheel_linux.py", "cp314"])
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
    assert "v3.0.3" in dockerfile


@pytest.mark.parametrize("wheels, succeeds", [(True, True), (False, False)])
def test_wheels_do_not_require_python_embedding_library(tmp_path, wheels, succeeds):
    (tmp_path / "FindPython3.cmake").write_text(
        'if("Development" IN_LIST Python3_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Embedding library unavailable")\n'
        "endif()\n"
        'if(NOT "Development.Module" IN_LIST Python3_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Extension headers were not requested")\n'
        "endif()\n"
        "set(Python3_FOUND TRUE)\n"
    )
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\n"
        "project(PythonComponents NONE)\n"
        "macro(dart_find_package)\nendmacro()\n"
        "macro(dart_check_required_package)\nendmacro()\n"
        "macro(dart_check_optional_package)\nendmacro()\n"
        f'set(CMAKE_MODULE_PATH "{tmp_path.as_posix()}")\n'
        "set(DART_BUILD_DARTPY ON)\n"
        f'set(DART_BUILD_WHEELS {"ON" if wheels else "OFF"})\n'
        "set(DART_USE_SYSTEM_ODE ON)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        f'include("{ROOT.as_posix()}/cmake/DARTFindDependencies.cmake")\n'
    )
    result = subprocess.run(
        ["cmake", "-S", str(tmp_path), "-B", str(tmp_path / "build")],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert (result.returncode == 0) == succeeds, result.stdout + result.stderr
    if not succeeds:
        assert "Embedding library unavailable" in result.stderr


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
