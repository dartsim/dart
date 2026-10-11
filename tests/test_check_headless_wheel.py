"""Check the graphics dependency gate, including auditwheel-renamed libraries."""

import importlib.util
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

SPEC = importlib.util.spec_from_file_location(
    "check_headless_wheel",
    Path(__file__).resolve().parents[1] / "scripts/check_headless_wheel.py",
)
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


def test_needed_libraries_ignores_other_dynamic_tags():
    assert CHECK.needed_libraries(
        "0x1 (NEEDED) Shared library: [libGL-12345678.so.1]\n"
        "0xe (SONAME) Library soname: [libdart-gui.so]\n"
        "0x1 (NEEDED) Shared library: [libstdc++.so.6]\n"
    ) == ["libGL-12345678.so.1", "libstdc++.so.6"]


@pytest.mark.parametrize(
    "library",
    [
        "libGL",
        "libGLX",
        "libGLESv2",
        "libEGL",
        "libOpenGL",
        "libX11",
        "libxcb",
        "libvulkan",
        "libwayland-client",
        "libwayland-server",
        "libwayland-egl",
    ],
)
@pytest.mark.parametrize("suffix", [".so.1", "-abcdef12.so.1"])
def test_forbidden_graphics_dependencies(library, suffix):
    assert CHECK.forbidden_library(library + suffix)


@pytest.mark.parametrize(
    "library", ["libglib-2.0.so.0", "libstdc++.so.6", "libjpeg.so.62", "libdl.so.2"]
)
def test_non_graphics_dependencies_are_allowed(library):
    assert not CHECK.forbidden_library(library)


@pytest.mark.parametrize(
    "exit_code, marker, passes", [(0, True, True), (0, False, False), (-6, True, False)]
)
def test_runtime_gate_requires_typed_exception_output(
    monkeypatch, exit_code, marker, passes
):
    def missing_library(name):
        raise OSError(name)

    monkeypatch.setattr("ctypes.CDLL", missing_library)
    world = SimpleNamespace(
        step=lambda: None, getTime=lambda: 0.01, getTimeStep=lambda: 0.001
    )
    monkeypatch.setitem(
        sys.modules,
        "dartpy",
        SimpleNamespace(simulation=SimpleNamespace(World=lambda: world)),
    )
    monkeypatch.setattr(sys, "argv", ["check", "wheel.whl"])
    modes = iter(["capture", "window", "window-default"])

    def run(command, **kwargs):
        if "-m" in command:
            return subprocess.CompletedProcess(command, 0)
        mode = next(modes)
        output = f"{mode} PASS: RenderingUnavailableError: packages\n" if marker else ""
        return subprocess.CompletedProcess(command, exit_code, output, "")

    monkeypatch.setattr(subprocess, "run", run)
    if passes:
        exec(CHECK.RUNTIME_CHECK, {})
    else:
        with pytest.raises(SystemExit, match="1"):
            exec(CHECK.RUNTIME_CHECK, {})
