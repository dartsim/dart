from __future__ import annotations

import argparse
import importlib.util
import pathlib

import pytest


_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "capture_py_demo", _ROOT / "scripts" / "capture_py_demo.py"
)
assert _SPEC is not None
assert _SPEC.loader is not None
capture_py_demo = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(capture_py_demo)


def _write_ppm(path: pathlib.Path, pixels: bytes) -> None:
    path.write_bytes(b"P6\n2 1\n255\n" + pixels)


def test_ppm_to_png_conversion_and_blank_guard(tmp_path: pathlib.Path) -> None:
    ppm = tmp_path / "capture.ppm"
    png = tmp_path / "capture.png"
    _write_ppm(ppm, bytes([0, 0, 0, 255, 0, 0]))

    assert capture_py_demo.ppm_has_nonzero_pixels(ppm)
    capture_py_demo.convert_ppm_to_png(ppm, png)
    assert png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")

    blank = tmp_path / "blank.ppm"
    _write_ppm(blank, bytes(6))
    assert not capture_py_demo.ppm_has_nonzero_pixels(blank)


def test_visual_capture_rejects_noop_backend(tmp_path: pathlib.Path) -> None:
    args = argparse.Namespace(
        backend="noop",
        allow_noop=False,
        scene="hello_world",
        frames=1,
        width=2,
        height=1,
        show_ui=False,
    )

    with pytest.raises(ValueError, match="noop"):
        capture_py_demo.build_demo_args(
            args, tmp_path / "capture.ppm", tmp_path / "frames"
        )
