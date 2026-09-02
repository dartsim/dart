"""Unit coverage for agent-time image verdict tooling."""

from __future__ import annotations

import json
import sys
import zlib
from hashlib import sha256
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_verdict
from _image_tools import ImageData, read_image, write_png, write_ppm


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


def _write_low_contrast_png(path: Path, width: int = 100, height: int = 100) -> None:
    # Two close mid-gray values: non-blank and non-uniform, but no
    # shadow/lighting contrast. A legitimate minimal render, not a defect.
    pixels = bytearray(bytes((92, 92, 92)) * (width * height))
    for y in range(height // 3, 2 * height // 3):
        for x in range(width // 3, 2 * width // 3):
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = b"\x60\x60\x60"
    write_png(path, width, height, bytes(pixels))


def _corrupt_idat(path: Path) -> None:
    """Replace the IDAT payload with undecompressable bytes, CRC kept valid."""
    data = bytearray(path.read_bytes())
    offset = 8
    while offset < len(data):
        length = int.from_bytes(data[offset : offset + 4], "big")
        kind = bytes(data[offset + 4 : offset + 8])
        if kind == b"IDAT":
            start = offset + 8
            data[start : start + length] = b"\x00" * length
            payload = bytes(data[start : start + length])
            checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
            data[start + length : start + length + 4] = checksum.to_bytes(4, "big")
            path.write_bytes(bytes(data))
            return
        offset += 12 + length
    raise AssertionError("test PNG did not contain an IDAT chunk")


def _corrupt_chunk_crc(path: Path, target: bytes) -> None:
    """Flip one bit of the stored CRC of *target* so only the CRC is wrong."""
    data = bytearray(path.read_bytes())
    offset = 8
    while offset < len(data):
        length = int.from_bytes(data[offset : offset + 4], "big")
        kind = bytes(data[offset + 4 : offset + 8])
        crc_start = offset + 8 + length
        if kind == target:
            data[crc_start] ^= 0x01
            path.write_bytes(bytes(data))
            return
        offset = crc_start + 4
    raise AssertionError(f"test PNG did not contain a {target!r} chunk")


def test_low_contrast_non_blank_image_passes_unless_contrast_required(
    tmp_path: Path,
) -> None:
    image = tmp_path / "low_contrast.png"
    _write_low_contrast_png(image)

    default_verdict = image_verdict.build_verdict(image)
    assert default_verdict["checks"]["non_blank"]["pass"] is True
    assert default_verdict["checks"]["non_blank"]["unique_colors_seen"] == 2
    assert default_verdict["checks"]["contrast"]["pass"] is False
    # Contrast is report-only by default: a low-contrast render still passes.
    assert default_verdict["pass"] is True
    assert default_verdict["reasons"] == []
    assert default_verdict["thresholds_used"]["require_contrast"] is False

    gated_verdict = image_verdict.build_verdict(image, require_contrast=True)
    assert gated_verdict["pass"] is False
    assert gated_verdict["thresholds_used"]["require_contrast"] is True
    # With no reference and a passing non-blank check, the only failure reasons
    # come from the now-gating contrast check.
    assert gated_verdict["reasons"]
    assert gated_verdict["reasons"] == gated_verdict["checks"]["contrast"]["reasons"]


def test_verdict_schema_contrast_and_metadata_round_trip(tmp_path: Path) -> None:
    image = tmp_path / "capture.png"
    _write_contrast_png(image)

    verdict = image_verdict.build_verdict(
        image,
        metadata={
            "backend": "llvmpipe",
            "fidelity": "test",
            "view": "front",
        },
    )

    assert verdict["pass"] is True
    assert verdict["checks"]["non_blank"]["pass"] is True
    assert verdict["checks"]["contrast"]["pass"] is True
    assert verdict["metadata"]["backend"] == "llvmpipe"
    assert verdict["machine_scope"] == "pixel-integrity"
    assert verdict["semantic_review"]["required"] is True
    assert verdict["semantic_review"]["performed_by_this_tool"] is False
    assert "text correctness oracle" in verdict["semantic_review"]["instruction"]
    assert verdict["thresholds_used"]["diff"]["fail"] == 0.016

    loaded = json.loads(json.dumps(verdict))
    assert loaded["schema_version"] == "dart.image_verdict/v1"
    assert loaded["image"]["width"] == 100
    assert loaded["image"]["sha256"] == sha256(image.read_bytes()).hexdigest()
    assert loaded["reasons"] == []


def test_ppm_verdict_hashes_the_analyzed_source_bytes(tmp_path: Path) -> None:
    image = tmp_path / "capture.ppm"
    write_ppm(image, 100, 100, _contrast_pixels())

    verdict = image_verdict.build_verdict(image)

    assert verdict["pass"] is True
    assert verdict["image"]["sha256"] == sha256(image.read_bytes()).hexdigest()


def test_blank_image_fails_non_blank_and_contrast(tmp_path: Path) -> None:
    image = tmp_path / "blank.png"
    write_png(image, 64, 64, bytes(64 * 64 * 3))

    verdict = image_verdict.build_verdict(image)

    assert verdict["pass"] is False
    assert verdict["checks"]["non_blank"]["pass"] is False
    assert "image contains only zero-valued pixels" in verdict["reasons"]
    assert verdict["checks"]["contrast"]["pass"] is False


def test_uniform_clear_color_image_fails_non_blank(tmp_path: Path) -> None:
    image = tmp_path / "clear_color.png"
    write_png(image, 64, 64, bytes((192, 192, 192)) * (64 * 64))

    verdict = image_verdict.build_verdict(image)

    assert verdict["pass"] is False
    assert verdict["checks"]["non_blank"]["pass"] is False
    assert verdict["checks"]["non_blank"]["unique_colors_seen"] == 1
    assert "image contains a single uniform color" in verdict["reasons"]


def test_corrupt_png_data_uses_controlled_cli_error(
    tmp_path: Path, capsys
) -> None:
    image = tmp_path / "corrupt.png"
    write_png(image, 16, 16, bytes((64, 64, 64)) * (16 * 16))
    _corrupt_idat(image)

    try:
        read_image(image)
    except ValueError as exc:
        assert "invalid PNG image data" in str(exc)
    else:
        raise AssertionError("corrupt PNG unexpectedly decoded")

    assert image_verdict.main([str(image)]) == 2
    output = capsys.readouterr()
    assert "invalid PNG image data" in output.err
    assert "Traceback" not in output.err


def test_png_with_corrupt_chunk_crc_gets_no_verdict(tmp_path: Path, capsys) -> None:
    # Every chunk carries a CRC; a capture whose bytes failed the format's own
    # integrity guard must never reach a verdict that a packet can bind.
    image = tmp_path / "bad_crc.png"
    _write_contrast_png(image)
    assert image_verdict.build_verdict(image)["pass"] is True
    _corrupt_chunk_crc(image, b"IDAT")

    with pytest.raises(ValueError, match="PNG chunk CRC mismatch"):
        read_image(image)
    with pytest.raises(ValueError, match="PNG chunk CRC mismatch"):
        image_verdict.build_verdict(image)

    assert image_verdict.main([str(image)]) == 2
    output = capsys.readouterr()
    assert "PNG chunk CRC mismatch" in output.err
    assert "Traceback" not in output.err


def test_png_with_corrupt_header_crc_gets_no_verdict(tmp_path: Path) -> None:
    image = tmp_path / "bad_header_crc.png"
    _write_contrast_png(image)
    _corrupt_chunk_crc(image, b"IHDR")

    with pytest.raises(ValueError, match="PNG chunk CRC mismatch"):
        image_verdict.build_verdict(image)


def test_png_with_trailing_bytes_after_iend_gets_no_verdict(
    tmp_path: Path, capsys
) -> None:
    # Junk appended after IEND leaves the decodable prefix intact, so a lenient
    # decoder would still hand back a passing verdict for tampered bytes.
    image = tmp_path / "trailing.png"
    _write_contrast_png(image)
    assert image_verdict.build_verdict(image)["pass"] is True
    image.write_bytes(image.read_bytes() + b"appended junk")

    with pytest.raises(ValueError, match="trailing bytes after PNG IEND chunk"):
        read_image(image)
    with pytest.raises(ValueError, match="trailing bytes after PNG IEND chunk"):
        image_verdict.build_verdict(image)

    assert image_verdict.main([str(image)]) == 2
    output = capsys.readouterr()
    assert "trailing bytes after PNG IEND chunk" in output.err
    assert "Traceback" not in output.err


def test_valid_png_still_produces_a_passing_verdict(tmp_path: Path, capsys) -> None:
    # Happy-path pin for the strict decoder: an untampered capture keeps its
    # passing verdict and a zero CLI exit status.
    image = tmp_path / "valid.png"
    pixels = _write_contrast_png(image)

    verdict = image_verdict.build_verdict(image)

    assert verdict["pass"] is True
    assert verdict["reasons"] == []
    assert verdict["checks"]["non_blank"]["pass"] is True
    assert verdict["image"]["width"] == 100
    assert verdict["image"]["height"] == 100
    assert verdict["image"]["sha256"] == sha256(image.read_bytes()).hexdigest()
    assert read_image(image).pixels == pixels

    assert image_verdict.main([str(image)]) == 0
    output = capsys.readouterr()
    assert output.err == ""
    assert json.loads(output.out)["pass"] is True


def test_unique_colors_seen_reports_the_true_distinct_color_count(
    tmp_path: Path,
) -> None:
    # The blank check only needs "at least two colors", but the reported number
    # must be the real distinct-color count, not a counter capped at 2.
    width = height = 12
    palette = [
        (10, 20, 30),
        (200, 10, 10),
        (10, 200, 10),
        (10, 10, 200),
        (250, 250, 250),
    ]
    pixels = bytearray()
    for index in range(width * height):
        pixels += bytes(palette[index % len(palette)])
    image = tmp_path / "palette.png"
    write_png(image, width, height, bytes(pixels))

    verdict = image_verdict.build_verdict(image)
    non_blank = verdict["checks"]["non_blank"]

    assert non_blank["unique_colors_seen"] == len(palette)
    assert non_blank["nonzero_pixels"] == width * height
    assert non_blank["total_pixels"] == width * height
    assert non_blank["pass"] is True

    # Same count when the pixels arrive through the PPM path.
    ppm = tmp_path / "palette.ppm"
    write_ppm(ppm, width, height, bytes(pixels))
    ppm_non_blank = image_verdict.build_verdict(ppm)["checks"]["non_blank"]
    assert ppm_non_blank["unique_colors_seen"] == len(palette)

    # A pixel that repeats a palette entry does not inflate the count, and a
    # genuinely new color does.
    grown = bytearray(pixels)
    grown[0:3] = bytes(palette[1])
    grown[3:6] = bytes((7, 8, 9))
    grown_image = tmp_path / "palette_grown.png"
    write_png(grown_image, width, height, bytes(grown))
    grown_non_blank = image_verdict.build_verdict(grown_image)["checks"]["non_blank"]
    assert grown_non_blank["unique_colors_seen"] == len(palette) + 1


def test_diff_catches_seeded_render_regression(tmp_path: Path) -> None:
    golden = tmp_path / "golden.png"
    capture = tmp_path / "capture.png"
    pixels = bytearray(_write_contrast_png(golden))
    width = height = 100
    for y in range(0, 20):
        for x in range(0, 20):
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = b"\xff\x00\x00"
    write_png(capture, width, height, bytes(pixels))

    verdict = image_verdict.build_verdict(capture, golden)

    assert verdict["pass"] is False
    assert verdict["machine_scope"] == "pixel-integrity-and-reference-diff"
    assert verdict["checks"]["diff"]["pass"] is False
    assert verdict["checks"]["diff"]["pct_pixels_over_threshold"] > 1.0
    assert verdict["image"]["sha256"] == sha256(capture.read_bytes()).hexdigest()
    assert verdict["reference"]["sha256"] == sha256(
        golden.read_bytes()
    ).hexdigest()
    assert any("diff pixels over threshold" in reason for reason in verdict["reasons"])


def test_diff_aa_ignore_suppresses_edge_fringe() -> None:
    width = height = 210
    reference_pixels = bytearray(_contrast_pixels(width, height))
    edge_x = 100
    for y in range(height):
        for x, value in ((edge_x - 1, 20), (edge_x, 128), (edge_x + 1, 240)):
            offset = (y * width + x) * 3
            reference_pixels[offset : offset + 3] = bytes((value, value, value))
    image_pixels = bytearray(reference_pixels)
    for y in range(height):
        offset = (y * width + edge_x) * 3
        image_pixels[offset : offset + 3] = bytes((240, 240, 240))

    image = ImageData(None, width, height, bytes(image_pixels))
    reference = ImageData(None, width, height, bytes(reference_pixels))

    strict = image_verdict.analyze_diff(
        image, reference, fail=0.016, failpercent=0.0, ignore_aa=False
    )
    tolerant = image_verdict.analyze_diff(
        image, reference, fail=0.016, failpercent=0.0, ignore_aa=True
    )

    assert strict["pass"] is False
    assert strict["pixels_over_threshold"] == height
    assert tolerant["pass"] is True
    assert tolerant["pixels_over_threshold"] == 0
    assert tolerant["ignored_aa_pixels"] == height


def test_ssim_reports_null_reason_when_numpy_unavailable(
    monkeypatch, tmp_path: Path
) -> None:
    golden = tmp_path / "golden.png"
    capture = tmp_path / "capture.png"
    pixels = _write_contrast_png(golden)
    write_png(capture, 100, 100, pixels)
    monkeypatch.setattr(
        image_verdict,
        "_load_numpy",
        lambda: (None, "numpy unavailable for test"),
    )

    verdict = image_verdict.build_verdict(capture, golden)

    assert verdict["checks"]["ssim"] == {
        "method": "global_luminance",
        "value": None,
        "reason": "numpy unavailable for test",
    }
