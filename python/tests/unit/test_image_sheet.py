"""Unit coverage for contact-sheet image assembly."""

from __future__ import annotations

import builtins
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_sheet
from _image_tools import read_image, write_png


def _solid_png(
    path: Path, color: tuple[int, int, int], width: int = 210, height: int = 220
) -> None:
    write_png(path, width, height, bytes(color) * (width * height))


def test_contact_sheet_geometry_and_label_smoke(tmp_path: Path) -> None:
    inputs = []
    for index, color in enumerate(
        [(60, 90, 130), (120, 70, 30), (30, 150, 90), (150, 30, 110)]
    ):
        path = tmp_path / f"frame_{index:06d}.png"
        _solid_png(path, color)
        inputs.append(path)
    output = tmp_path / "sheet.png"

    result = image_sheet.assemble_sheet(inputs, output)

    sheet = read_image(output)
    assert result["width"] == 600
    assert result["height"] == 600
    assert sheet.width == 600
    assert sheet.height == 600
    assert len(result["frames"]) == 4

    white_label_pixels = 0
    for y in range(0, 24):
        for x in range(0, 24):
            offset = (y * sheet.width + x) * 3
            if sheet.pixels[offset : offset + 3] == b"\xff\xff\xff":
                white_label_pixels += 1
    assert white_label_pixels > 0


def test_contact_sheet_does_not_require_strict_zip_keyword(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    first = tmp_path / "first.png"
    second = tmp_path / "second.png"
    _solid_png(first, (60, 90, 130))
    _solid_png(second, (120, 70, 30))
    output = tmp_path / "sheet.png"
    real_zip = builtins.zip

    def python39_zip(*iterables, **kwargs):
        if kwargs:
            raise TypeError("zip() takes no keyword arguments")
        return real_zip(*iterables)

    monkeypatch.setattr(builtins, "zip", python39_zip)

    result = image_sheet.assemble_sheet([first, second], output)

    assert result["frames"][0]["path"] == str(first)
    assert result["frames"][1]["path"] == str(second)


def test_contact_sheet_rejects_tiles_below_floor(tmp_path: Path) -> None:
    frame = tmp_path / "frame.png"
    _solid_png(frame, (30, 60, 90))

    with pytest.raises(ValueError, match="at least 200 px"):
        image_sheet.assemble_sheet([frame], tmp_path / "sheet.png", tile_size=199)


def test_parse_grid_rejects_invalid_shape() -> None:
    with pytest.raises(ValueError, match="COLSxROWS"):
        image_sheet.parse_grid("3-by-3")
