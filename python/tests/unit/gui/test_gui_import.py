"""Import and lazy viewer construction do not require a graphics display."""

import os
import subprocess
import sys
from pathlib import Path

import dartpy as dart


def _run_isolated(code):
    environment = os.environ.copy()
    environment.pop("DISPLAY", None)
    environment.pop("WAYLAND_DISPLAY", None)
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
