import importlib.util
import sys
from pathlib import Path


def _load_runner():
    repo_root = Path(__file__).resolve().parents[3]
    script_path = repo_root / "python" / "tests" / "run_pytest_with_optional_xvfb.py"
    spec = importlib.util.spec_from_file_location(
        "run_pytest_with_optional_xvfb", script_path
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_linux_pytest_env_defaults_to_software_gl(monkeypatch):
    runner = _load_runner()
    monkeypatch.setattr(runner.sys, "platform", "linux")
    monkeypatch.delenv("LIBGL_ALWAYS_SOFTWARE", raising=False)
    monkeypatch.delenv("MESA_LOADER_DRIVER_OVERRIDE", raising=False)

    env = runner._pytest_env()

    assert env["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert env["MESA_LOADER_DRIVER_OVERRIDE"] == "llvmpipe"


def test_linux_pytest_env_preserves_gl_overrides(monkeypatch):
    runner = _load_runner()
    monkeypatch.setattr(runner.sys, "platform", "linux")
    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "0")
    monkeypatch.setenv("MESA_LOADER_DRIVER_OVERRIDE", "zinc")

    env = runner._pytest_env()

    assert env["LIBGL_ALWAYS_SOFTWARE"] == "0"
    assert env["MESA_LOADER_DRIVER_OVERRIDE"] == "zinc"


def test_main_uses_xvfb_without_linux_display(monkeypatch):
    runner = _load_runner()
    calls = []

    def fake_call(command, env):
        calls.append((command, env))
        return 0

    monkeypatch.setattr(runner.sys, "platform", "linux")
    monkeypatch.setattr(runner.sys, "argv", ["runner", "test_file.py"])
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    monkeypatch.setattr(runner.shutil, "which", lambda name: "/usr/bin/xvfb-run")
    monkeypatch.setattr(runner.subprocess, "call", fake_call)

    assert runner.main() == 0

    command, env = calls[0]
    assert command[:4] == [
        "/usr/bin/xvfb-run",
        "--auto-servernum",
        "--server-args=-screen 0 1024x768x24",
        sys.executable,
    ]
    assert command[4:] == ["-m", "pytest", "test_file.py"]
    assert env["LIBGL_ALWAYS_SOFTWARE"] == "1"
