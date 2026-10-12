"""Wheel import checks must not borrow libraries or graphics setup from Pixi."""

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest


def test_smoke_environment_drops_conda_libraries_and_graphics_overrides(
    tmp_path, monkeypatch
):
    script = Path(__file__).resolve().parents[1] / "scripts" / "test_wheel.py"
    spec = importlib.util.spec_from_file_location("dart_wheel_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    prefix = tmp_path / "conda"
    system_bin = tmp_path / "system" / "bin"
    sibling_bin = tmp_path / "conda-other" / "bin"
    monkeypatch.setenv("CONDA_PREFIX", str(prefix))
    monkeypatch.setenv(
        "PATH", os.pathsep.join(map(str, (prefix / "bin", system_bin, sibling_bin)))
    )
    for name in ("DISPLAY", "LD_LIBRARY_PATH", "PYTHONPATH", "VK_DRIVER_FILES"):
        monkeypatch.setenv(name, "must-not-leak")

    env = module._smoke_environment()
    assert env["PATH"] == os.pathsep.join(map(str, (system_bin, sibling_bin)))
    for name in (
        "CONDA_PREFIX",
        "DISPLAY",
        "LD_LIBRARY_PATH",
        "PYTHONPATH",
        "VK_DRIVER_FILES",
    ):
        assert name not in env
        assert os.environ[name] == (
            str(prefix) if name == "CONDA_PREFIX" else "must-not-leak"
        )


def test_command_timeout_reports_partial_output(capsys):
    script = Path(__file__).resolve().parents[1] / "scripts" / "test_wheel.py"
    spec = importlib.util.spec_from_file_location("dart_wheel_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    cmd = [
        sys.executable,
        "-c",
        "import sys, time; print('partial stdout', flush=True); "
        "print('partial stderr', file=sys.stderr, flush=True); time.sleep(10)",
    ]
    with pytest.raises(subprocess.TimeoutExpired):
        module.run_command(cmd, timeout=1)

    captured = capsys.readouterr()
    assert "partial stdout" in captured.out.splitlines()
    assert "partial stderr" in captured.err.splitlines()
    assert "Timed out after 1s" in captured.err
    assert " ".join(cmd) in captured.err
