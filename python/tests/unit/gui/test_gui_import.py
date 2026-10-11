"""Import and lazy viewer construction do not require a graphics display."""

import os
import subprocess
import sys
from pathlib import Path

import dartpy as dart
import pytest


def _run_isolated(code, extra_environment=None):
    environment = os.environ.copy()
    environment.pop("DISPLAY", None)
    environment.pop("WAYLAND_DISPLAY", None)
    environment.update(extra_environment or {})
    result = subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            "import sys; sys.path.insert(0, sys.argv[1]); import dartpy as dart\n"
            + code,
            str(Path(dart.__file__).resolve().parent),
        ],
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    return result


def test_import_without_display_does_not_initialize_graphics():
    result = _run_isolated("viewer = dart.gui.Viewer()\n" "print('ready')")
    assert result.stdout == "ready\n"
    assert result.stderr == ""


def test_host_exception_translation_without_graphics():
    result = _run_isolated(
        "try:\n"
        "    dart.gui.Viewer().setGuiScale(-1)\n"
        "except ValueError as error:\n"
        "    print(str(error))\n"
        "else:\n"
        "    raise AssertionError('invalid GUI scale was accepted')\n"
    )
    assert result.stdout == "GUI scale must be finite and positive\n"
    assert result.stderr == ""


def test_gui_uses_a_single_public_namespace():
    assert not hasattr(dart.gui, "osg")
    assert not hasattr(dart.gui, "filament")


@pytest.mark.skipif(sys.platform != "linux", reason="Linux Vulkan ICD probe")
def test_capture_without_vulkan_icd_raises_typed_exception():
    result = _run_isolated(
        "try:\n"
        "    viewer = dart.gui.Viewer()\n"
        "    viewer.setUpViewForCapture(32, 24)\n"
        "    viewer.frame()\n"
        "except dart.gui.RenderingUnavailableError as error:\n"
        "    assert isinstance(error, RuntimeError)\n"
        "    assert 'libvulkan1' in str(error)\n"
        "    assert 'mesa-vulkan-drivers' in str(error)\n"
        "    print('capture unavailable')\n"
        "else:\n"
        "    raise AssertionError('capture accepted a missing ICD')\n",
        {
            "VK_DRIVER_FILES": "missing-dart-test-icd.json",
            "VK_ICD_FILENAMES": "missing-dart-test-icd.json",
        },
    )
    assert result.stdout == "capture unavailable\n"


@pytest.mark.skipif(sys.platform != "linux", reason="Linux X11 window backend")
@pytest.mark.parametrize("arguments", ["0, 0, 32, 24", ""])
def test_window_without_display_raises_typed_exception(arguments):
    result = _run_isolated(
        "try:\n"
        "    viewer = dart.gui.Viewer()\n"
        f"    viewer.setUpViewInWindow({arguments})\n"
        "    viewer.frame()\n"
        "except dart.gui.RenderingUnavailableError as error:\n"
        "    assert isinstance(error, RuntimeError)\n"
        "    assert 'libx11-6' in str(error)\n"
        "    assert 'libgl1' in str(error)\n"
        "    print('window unavailable')\n"
        "else:\n"
        "    raise AssertionError('window accepted a missing display')\n"
    )
    assert result.stdout == "window unavailable\n"
