"""Unit coverage for the image golden workflow wrapper."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_golden
import image_verdict
from _image_tools import read_image, write_png


def _contrast_pixels(width: int = 100, height: int = 100) -> bytes:
    pixels = bytearray(bytes((92, 92, 92)) * (width * height))
    x0, y0, x1, y1 = image_verdict._scene_region(width, height)
    span = max(1, x1 - x0)
    for y in range(y0, y1):
        for x in range(x0, x1):
            rel = (x - x0) / span
            if rel < 1 / 3:
                color = (20, 20, 20)
            elif rel < 2 / 3:
                color = (120, 120, 120)
            else:
                color = (240, 240, 240)
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = bytes(color)
    return bytes(pixels)


def _write_contrast_png(path: Path, width: int = 100, height: int = 100) -> bytes:
    pixels = _contrast_pixels(width, height)
    write_png(path, width, height, pixels)
    return pixels


def test_update_writes_golden_and_metadata_sidecar(tmp_path: Path) -> None:
    capture = tmp_path / "capture.png"
    golden = tmp_path / "golden.png"
    _write_contrast_png(capture, 80, 90)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPTS / "image_golden.py"),
            str(capture),
            str(golden),
            "--update",
            "--backend",
            "llvmpipe",
            "--fidelity",
            "headless",
            "--meta",
            "view=canonical",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert read_image(golden).width == 80
    sidecar = json.loads(image_golden.golden_sidecar_path(golden).read_text())
    assert sidecar["metadata"] == {
        "backend": "llvmpipe",
        "fidelity": "headless",
        "view": "canonical",
    }
    assert sidecar["golden"]["height"] == 90
    assert sidecar["tolerance"]["fail"] == 0.016


def test_compare_catches_seeded_render_regression(tmp_path: Path) -> None:
    golden = tmp_path / "golden.png"
    capture = tmp_path / "capture.png"
    pixels = bytearray(_write_contrast_png(golden))
    width = height = 100
    for y in range(0, 20):
        for x in range(0, 20):
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = b"\x00\xff\x00"
    write_png(capture, width, height, bytes(pixels))

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPTS / "image_golden.py"),
            str(capture),
            str(golden),
            "--failpercent",
            "0.5",
            "--retries",
            "3",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 1, result.stdout
    verdict = json.loads(result.stdout)
    assert verdict["pass"] is False
    assert verdict["golden_workflow"]["attempt"] == 3
    assert verdict["checks"]["diff"]["pct_pixels_over_threshold"] > 0.5
