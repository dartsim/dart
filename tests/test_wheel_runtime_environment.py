"""Wheel import checks must not borrow libraries or graphics setup from Pixi."""

import importlib.util
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "test_wheel.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("dart_wheel_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_smoke_environment_drops_conda_libraries_and_graphics_overrides(
    tmp_path, monkeypatch
):
    module = _load_runner()

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


def test_command_streams_output_before_child_finishes(tmp_path):
    ready = tmp_path / "ready"
    release = tmp_path / "release"
    output = tmp_path / "output"
    child = (
        "import sys, time\nfrom pathlib import Path\n"
        'print("child stdout", flush=True)\n'
        'print("child stderr", file=sys.stderr, flush=True)\n'
        f"Path({str(ready)!r}).touch()\n"
        f"while not Path({str(release)!r}).exists(): time.sleep(0.01)\n"
    )
    parent = (
        "import runpy\n"
        f"runpy.run_path({str(SCRIPT)!r})['run_command']("
        f"{[sys.executable, '-u', '-c', child]!r})\n"
    )
    with output.open("w") as stream:
        process = subprocess.Popen(
            [sys.executable, "-c", parent], stdout=stream, stderr=stream
        )
        try:
            deadline = time.monotonic() + 10
            while not ready.exists():
                assert process.poll() is None, output.read_text()
                assert time.monotonic() < deadline, output.read_text()
                time.sleep(0.01)
            text = output.read_text()
            assert "Running" in text
            assert "child stdout" in text
            assert "child stderr" in text
            assert process.poll() is None
        finally:
            release.touch()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    assert process.returncode == 0, output.read_text()


def test_command_timeout_reports_stage_and_stops_child(capfd):
    module = _load_runner()
    started = time.monotonic()
    with pytest.raises(subprocess.TimeoutExpired):
        module.run_command(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            stage="native smoke",
            timeout=0.5,
        )
    assert time.monotonic() - started < 10
    out, err = capfd.readouterr()
    assert "native smoke" in out
    assert "native smoke timed out after 0.5 seconds" in err


def test_command_nonzero_exit_keeps_failure_and_stage(capfd):
    module = _load_runner()
    with pytest.raises(subprocess.CalledProcessError) as error:
        module.run_command(
            [sys.executable, "-c", "raise SystemExit(7)"], stage="wheel installation"
        )
    assert error.value.returncode == 7
    assert "wheel installation failed with exit code 7" in capfd.readouterr().err
