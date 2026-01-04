import importlib.util
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def run_cpp_example():
    repo_root = Path(__file__).resolve().parents[3]
    script_path = repo_root / "scripts" / "run_cpp_example.py"

    spec = importlib.util.spec_from_file_location("run_cpp_example", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("old_name", "new_name"),
    [
        ("raylib_gui", "raylib"),
        ("atlas_simbicon", "control_walking_humanoid"),
        ("biped_stand", "control_balance_biped"),
        ("box_stacking", "collision_box_stacking"),
        ("capsule_ground_contact", "collision_capsule_ground_contact"),
        ("heightmap", "collision_heightmap"),
        ("operational_space_control", "control_operational_space"),
        ("rigid_shapes", "collision_rigid_shapes"),
        ("vehicle", "control_vehicle"),
        ("wam_ikfast", "ik_analytic_wam"),
    ],
)
def test_normalize_target_deprecates_renames(
    run_cpp_example, capsys, old_name, new_name
):
    assert run_cpp_example._normalize_target(old_name) == new_name
    captured = capsys.readouterr()
    assert old_name in captured.err
    assert new_name in captured.err


def test_normalize_target_passthrough(run_cpp_example, capsys):
    assert (
        run_cpp_example._normalize_target("control_balance_biped")
        == "control_balance_biped"
    )
    assert capsys.readouterr().err == ""


def test_list_examples_filters_cmake_lists(run_cpp_example, tmp_path):
    (tmp_path / "alpha").mkdir()
    (tmp_path / "alpha" / "CMakeLists.txt").write_text("cmake_minimum_required")
    (tmp_path / "beta").mkdir()
    (tmp_path / "beta" / "README.md").write_text("not a cmake example")
    (tmp_path / "gamma").mkdir()
    (tmp_path / "gamma" / "CMakeLists.txt").write_text("cmake_minimum_required")

    assert run_cpp_example._list_examples(tmp_path) == ["alpha", "gamma"]


@pytest.mark.parametrize(
    ("target", "build_target", "binary_name"),
    [
        ("raylib", "dart_raylib", "raylib"),
        ("dart_raylib", "dart_raylib", "raylib"),
        (
            "control_walking_humanoid",
            "control_walking_humanoid",
            "control_walking_humanoid",
        ),
    ],
)
def test_resolve_build_and_binary(target, build_target, binary_name, run_cpp_example):
    resolved_build_target, resolved_binary = run_cpp_example._resolve_build_and_binary(
        target
    )
    assert resolved_build_target == build_target
    assert resolved_binary == binary_name


def test_cmake_cache_bool(run_cpp_example, tmp_path):
    cache_path = tmp_path / "CMakeCache.txt"
    cache_path.write_text("DART_BUILD_GUI_RAYLIB:BOOL=ON\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI_RAYLIB") is True

    cache_path.write_text("DART_BUILD_GUI_RAYLIB:BOOL=OFF\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI_RAYLIB") is False

    cache_path.write_text("DART_BUILD_GUI_RAYLIB:BOOL=maybe\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI_RAYLIB") is None

    cache_path.write_text("UNRELATED:BOOL=ON\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI_RAYLIB") is None


def test_ensure_target_requirements_enables_raylib(run_cpp_example, tmp_path, monkeypatch):
    (tmp_path / "CMakeCache.txt").write_text(
        "DART_BUILD_GUI_RAYLIB:BOOL=OFF\n", encoding="utf-8"
    )

    calls = []

    def fake_run(cmd, *args, **kwargs):
        calls.append((cmd, args, kwargs))
        return None

    monkeypatch.setattr(run_cpp_example.subprocess, "run", fake_run)

    env = {"EXAMPLE": "1"}
    run_cpp_example._ensure_target_requirements(tmp_path, "raylib", env)

    assert len(calls) == 1
    cmd, args, kwargs = calls[0]
    assert cmd[0] == "cmake"
    assert "-DDART_BUILD_GUI_RAYLIB=ON" in cmd
    assert kwargs.get("env") == env
    assert kwargs.get("check") is True


def test_ensure_target_requirements_noop_when_enabled(run_cpp_example, tmp_path, monkeypatch):
    (tmp_path / "CMakeCache.txt").write_text(
        "DART_BUILD_GUI_RAYLIB:BOOL=ON\n", encoding="utf-8"
    )

    def fail_run(*_args, **_kwargs):
        raise AssertionError("subprocess.run should not be called")

    monkeypatch.setattr(run_cpp_example.subprocess, "run", fail_run)

    run_cpp_example._ensure_target_requirements(tmp_path, "raylib", {"EXAMPLE": "1"})
