"""Unit coverage for evidence image composition (side-by-side, blend, diff)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_compose
from _image_tools import diff_heatmap, overlay_blend, read_image, side_by_side, write_png


def _solid_png(path: Path, color: tuple[int, int, int], size: int = 32) -> None:
    write_png(path, size, size, bytes(color) * (size * size))


def test_side_by_side_dimensions_and_labels(tmp_path: Path) -> None:
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    _solid_png(a, (200, 40, 40))
    _solid_png(b, (40, 200, 40))
    result = side_by_side(
        [read_image(a), read_image(b)], labels=["BEFORE", "AFTER"], gap=4
    )
    assert result.width == 32 * 2 + 4 * 3
    assert result.height > 32 + 8  # label band adds height
    # Label text renders as white pixels in the band above the tiles.
    band = result.pixels[: result.width * 12 * 3]
    assert b"\xff\xff\xff" in band


def test_overlay_blend_midpoint(tmp_path: Path) -> None:
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    _solid_png(a, (0, 0, 0))
    _solid_png(b, (200, 100, 50))
    blended = overlay_blend(read_image(a), read_image(b), alpha=0.5)
    assert blended.pixels[0:3] == bytes((100, 50, 25))


def test_diff_heatmap_statistics(tmp_path: Path) -> None:
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    _solid_png(a, (10, 10, 10))
    _solid_png(b, (10, 10, 10))
    identical, stats = diff_heatmap(read_image(a), read_image(b))
    assert stats == {
        "changed_pixel_fraction": 0.0,
        "mean_abs": 0.0,
        "max_abs": 0,
    }
    assert set(identical.pixels) == {0}

    c = tmp_path / "c.png"
    _solid_png(c, (30, 10, 10))
    _, changed_stats = diff_heatmap(read_image(a), read_image(c))
    assert changed_stats["changed_pixel_fraction"] == 1.0
    assert changed_stats["max_abs"] == 20


def test_size_mismatch_is_rejected(tmp_path: Path) -> None:
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    _solid_png(a, (0, 0, 0), size=32)
    write_png(b, 16, 16, bytes((0, 0, 0)) * 256)
    with pytest.raises(ValueError, match="equal image sizes"):
        diff_heatmap(read_image(a), read_image(b))


def test_ppm_reader_skips_header_whitespace_without_eating_pixel_whitespace(
    tmp_path: Path,
) -> None:
    path = tmp_path / "whitespace.ppm"
    path.write_bytes(b"P6\n1 1\n255\n\n" + bytes([10, 20, 30]))
    assert read_image(path).pixels == bytes([10, 20, 30])


def test_cli_diff_manifest(tmp_path: Path, capsys: pytest.CaptureFixture) -> None:
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    out = tmp_path / "diff.png"
    manifest_path = tmp_path / "diff.json"
    _solid_png(a, (10, 10, 10))
    _solid_png(b, (12, 10, 10))
    code = image_compose.main(
        [
            "diff",
            str(a),
            str(b),
            "--out",
            str(out),
            "--manifest",
            str(manifest_path),
        ]
    )
    assert code == 0
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "dart.image_compose/v1"
    assert manifest["mode"] == "diff"
    assert manifest["stats"]["max_abs"] == 2
    assert out.exists()


def test_cli_label_count_mismatch(tmp_path: Path) -> None:
    a = tmp_path / "a.png"
    _solid_png(a, (1, 2, 3))
    code = image_compose.main(
        [
            "side-by-side",
            str(a),
            "--labels",
            "ONE",
            "TWO",
            "--out",
            str(tmp_path / "x.png"),
        ]
    )
    assert code == 2
