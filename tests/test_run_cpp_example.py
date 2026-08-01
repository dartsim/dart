import importlib.util
import sys
from pathlib import Path


def _load_run_cpp_example():
    script = Path(__file__).resolve().parents[1] / "scripts" / "run_cpp_example.py"
    spec = importlib.util.spec_from_file_location("run_cpp_example", script)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_interactive_filament_run_args_do_not_force_dimensions(monkeypatch):
    module = _load_run_cpp_example()
    monkeypatch.setenv("DISPLAY", ":1")
    monkeypatch.delenv("DART_GUI_FILAMENT_WIDTH", raising=False)
    monkeypatch.delenv("DART_GUI_FILAMENT_HEIGHT", raising=False)

    args = module._prepare_filament_run_args(
        [], "mvp", scene_option_explicit=False, multiple_scenes=False
    )

    assert "--width" not in args
    assert "--height" not in args


def test_interactive_filament_run_args_keep_dimension_env_overrides(monkeypatch):
    module = _load_run_cpp_example()
    monkeypatch.setenv("DISPLAY", ":1")
    monkeypatch.setenv("DART_GUI_FILAMENT_WIDTH", "1440")
    monkeypatch.setenv("DART_GUI_FILAMENT_HEIGHT", "900")

    args = module._prepare_filament_run_args(
        [], "mvp", scene_option_explicit=False, multiple_scenes=False
    )

    assert args[args.index("--width") + 1] == "1440"
    assert args[args.index("--height") + 1] == "900"


def test_headless_filament_run_args_keep_stable_capture_dimensions(monkeypatch):
    module = _load_run_cpp_example()
    monkeypatch.delenv("DART_GUI_FILAMENT_WIDTH", raising=False)
    monkeypatch.delenv("DART_GUI_FILAMENT_HEIGHT", raising=False)

    args = module._prepare_filament_run_args(
        ["--headless"], "mvp", scene_option_explicit=False, multiple_scenes=False
    )

    assert args[args.index("--width") + 1] == "640"
    assert args[args.index("--height") + 1] == "480"


def test_filament_config_forwards_memory_diagnostics_build_override(
    monkeypatch, tmp_path
):
    module = _load_run_cpp_example()
    configured = {}

    def capture_configure(build_dir, definitions, env):
        configured.update(definitions)

    monkeypatch.setattr(module, "_configure", capture_configure)
    env = {
        "DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS_OVERRIDE": "ON",
        "DART_FETCH_FILAMENT_OVERRIDE": "ON",
        "DART_USE_SYSTEM_FILAMENT_OVERRIDE": "OFF",
    }

    module._ensure_filament(tmp_path, env, smoke=False)

    assert configured["DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS"] == "ON"
