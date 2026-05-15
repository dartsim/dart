import importlib.util
import sys
from pathlib import Path

import pytest


def _write_ppm(path: Path, width: int, height: int, pixels: bytes) -> None:
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)


def _contrast_pixels(width: int, height: int) -> bytes:
    pixels = bytearray([1, 1, 1] * width * height)
    x0 = max(0, int(width * 0.40))
    y0 = max(0, int(height * 0.45))
    x1 = min(width, int(width * 0.97))
    y1 = min(height, int(height * 0.90))

    index = 0
    for y in range(y0, y1):
        for x in range(x0, x1):
            value = (1, 1, 1)
            if index < 33:
                value = (100, 100, 100)
            elif index < 66:
                value = (255, 255, 255)
            offset = (y * width + x) * 3
            pixels[offset : offset + 3] = value
            index += 1

    return bytes(pixels)


@pytest.fixture(scope="module")
def analyzer():
    repo_root = Path(__file__).resolve().parents[4]
    script_path = (
        repo_root
        / "dart"
        / "gui"
        / "experimental"
        / "detail"
        / "filament"
        / "testing"
        / "analyze_headless_smoke.py"
    )

    spec = importlib.util.spec_from_file_location(
        "analyze_headless_smoke", script_path
    )
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_basic_mode_accepts_nonzero_pixels(analyzer, tmp_path, capsys):
    ppm_path = tmp_path / "nonzero.ppm"
    _write_ppm(ppm_path, 2, 2, bytes([0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0]))

    analyzer.analyze_basic(ppm_path, 2, 2)

    assert "image nonzero pixels: 1/4" in capsys.readouterr().out


def test_basic_mode_rejects_zero_pixels(analyzer, tmp_path):
    ppm_path = tmp_path / "zero.ppm"
    _write_ppm(ppm_path, 2, 2, bytes(2 * 2 * 3))

    with pytest.raises(RuntimeError, match="only zero-valued pixels"):
        analyzer.analyze_basic(ppm_path, 2, 2)


def test_contrast_mode_rejects_flat_nonzero_image(analyzer, tmp_path):
    width = 20
    height = 20
    ppm_path = tmp_path / "flat.ppm"
    _write_ppm(ppm_path, width, height, bytes([80, 80, 80] * width * height))

    with pytest.raises(RuntimeError, match="lacks expected shadow/lighting contrast"):
        analyzer.analyze_contrast(ppm_path, width, height)


def test_contrast_mode_accepts_shadow_fixture_distribution(
    analyzer, tmp_path, capsys
):
    width = 20
    height = 20
    ppm_path = tmp_path / "contrast.ppm"
    _write_ppm(ppm_path, width, height, _contrast_pixels(width, height))

    analyzer.analyze_contrast(ppm_path, width, height)

    output = capsys.readouterr().out
    assert "image nonzero pixels: 400/400" in output
    assert "scene luminance contrast:" in output


def test_cli_defaults_to_contrast_mode(analyzer, tmp_path, monkeypatch, capsys):
    width = 20
    height = 20
    ppm_path = tmp_path / "cli.ppm"
    _write_ppm(ppm_path, width, height, _contrast_pixels(width, height))
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "analyze_headless_smoke.py",
            str(ppm_path),
            "--width",
            str(width),
            "--height",
            str(height),
        ],
    )

    analyzer.main()

    assert "scene luminance contrast:" in capsys.readouterr().out
