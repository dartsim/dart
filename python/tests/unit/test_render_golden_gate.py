"""Unit coverage for the opt-in render golden gate wrapper."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import render_golden_gate
from _image_tools import write_png


def _png_bytes(path: Path, *, changed: bool = False) -> bytes:
    width = height = 16
    pixels = bytearray()
    for y in range(height):
        for x in range(width):
            if x < width // 2:
                pixels.extend((32, 32, 32))
            else:
                pixels.extend((220, 220, 220))
    if changed:
        pixels[0:3] = b"\x00\xff\x00"
    write_png(path, width, height, bytes(pixels))
    return path.read_bytes()


def test_render_golden_gate_rerenders_on_retry(tmp_path: Path, monkeypatch) -> None:
    golden = tmp_path / "golden.png"
    output = tmp_path / "verdict.json"
    golden_bytes = _png_bytes(golden)
    changed_bytes = _png_bytes(tmp_path / "changed.png", changed=True)
    renders = [changed_bytes, golden_bytes]
    sizes = []

    def render_scene(size):
        sizes.append(size)
        return renders.pop(0)

    monkeypatch.setitem(render_golden_gate.SCENES, "unit_retry", render_scene)

    exit_code = render_golden_gate.main(
        [
            "--scene",
            "unit_retry",
            "--golden",
            str(golden),
            "--retries",
            "2",
            "--fail",
            "0.0",
            "--failpercent",
            "0.0",
            "--out",
            str(output),
        ]
    )

    verdict = json.loads(output.read_text())
    assert exit_code == 0
    assert sizes == [(320, 240), (320, 240)]
    assert verdict["golden_workflow"]["attempt"] == 2
    assert verdict["golden_workflow"]["retries"] == 2
