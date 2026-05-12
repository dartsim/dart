import importlib.util
import sys
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
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_normalize_target_deprecates_raylib_gui(run_cpp_example, capsys):
    assert run_cpp_example._normalize_target("raylib_gui") == "raylib"
    captured = capsys.readouterr()
    assert "raylib_gui" in captured.err
    assert "renamed to `raylib`" in captured.err


def test_normalize_target_passthrough(run_cpp_example, capsys):
    assert run_cpp_example._normalize_target("atlas_simbicon") == "atlas_simbicon"
    assert capsys.readouterr().err == ""


@pytest.mark.parametrize(
    ("target", "build_target", "binary_name", "requirements"),
    [
        ("raylib", "dart_raylib", "raylib", ("raylib",)),
        ("dart_raylib", "dart_raylib", "raylib", ("raylib",)),
        ("filament_gui", "dart_filament_gui", "filament_gui", ("filament",)),
        ("atlas_simbicon", "atlas_simbicon", "atlas_simbicon", ()),
    ],
)
def test_resolve_example(
    target, build_target, binary_name, requirements, run_cpp_example
):
    spec = run_cpp_example._resolve_example(target)
    assert spec.build_target == build_target
    assert spec.binary_name == binary_name
    assert spec.requirements == requirements


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
    spec = run_cpp_example._resolve_example("raylib")
    run_cpp_example._ensure_target_requirements(tmp_path, spec, env, smoke=False)

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

    spec = run_cpp_example._resolve_example("raylib")
    run_cpp_example._ensure_target_requirements(
        tmp_path, spec, {"EXAMPLE": "1"}, smoke=False
    )


def test_run_filament_smoke_fails_when_no_tests_discovered(
    run_cpp_example, tmp_path, monkeypatch
):
    calls = []

    def fake_run_with_optional_xvfb(command, env, use_xvfb):
        calls.append((command, env, use_xvfb))

    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setattr(
        run_cpp_example, "_run_with_optional_xvfb", fake_run_with_optional_xvfb
    )

    env = {"EXAMPLE": "1"}
    run_cpp_example._run_filament_smoke(tmp_path, env)

    assert len(calls) == 1
    command, runtime_env, use_xvfb = calls[0]
    assert command[:3] == ["ctest", "--test-dir", str(tmp_path)]
    assert "--no-tests=error" in command
    assert runtime_env["EXAMPLE"] == "1"
    assert use_xvfb is False
