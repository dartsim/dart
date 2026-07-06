#!/usr/bin/env python3

import argparse
from pathlib import Path


def _read_ppm(path: Path, expected_width: int, expected_height: int) -> bytes:
    data = path.read_bytes()
    header = f"P6\n{expected_width} {expected_height}\n255\n".encode()
    if not data.startswith(header):
        raise RuntimeError(
            f"{path} does not start with the expected "
            f"{expected_width}x{expected_height} binary PPM header"
        )
    pixels = data[len(header) :]
    expected_size = expected_width * expected_height * 3
    if len(pixels) < expected_size:
        raise RuntimeError(
            f"{path} contains {len(pixels)} bytes of pixel data, "
            f"expected at least {expected_size}"
        )
    return pixels[:expected_size]


def _luminance(pixel: bytes) -> float:
    red, green, blue = pixel
    return 0.2126 * red + 0.7152 * green + 0.0722 * blue


def _scene_region(width: int, height: int) -> tuple[int, int, int, int]:
    return (
        max(0, int(width * 0.40)),
        max(0, int(height * 0.45)),
        min(width, int(width * 0.97)),
        min(height, int(height * 0.90)),
    )


def _require_nonzero_pixels(pixels: bytes) -> None:
    total_pixels = len(pixels) // 3
    nonzero_pixels = 0
    for offset in range(0, len(pixels), 3):
        if any(pixels[offset : offset + 3]):
            nonzero_pixels += 1

    if nonzero_pixels == 0:
        raise RuntimeError("image contains only zero-valued pixels")

    print(f"image nonzero pixels: {nonzero_pixels}/{total_pixels}")


def analyze_basic(path: Path, width: int, height: int) -> None:
    pixels = _read_ppm(path, width, height)
    _require_nonzero_pixels(pixels)


def analyze_contrast(path: Path, width: int, height: int) -> None:
    pixels = _read_ppm(path, width, height)
    _require_nonzero_pixels(pixels)
    x0, y0, x1, y1 = _scene_region(width, height)
    values = []
    for y in range(y0, y1):
        row_offset = y * width * 3
        for x in range(x0, x1):
            offset = row_offset + x * 3
            values.append(_luminance(pixels[offset : offset + 3]))

    if not values:
        raise RuntimeError("scene luminance probe region is empty")

    values.sort()
    dark_count = sum(value < 55.0 for value in values)
    mid_count = sum(70.0 <= value <= 170.0 for value in values)
    bright_count = sum(value > 190.0 for value in values)
    p05 = values[int(len(values) * 0.05)]
    p95 = values[int(len(values) * 0.95)]
    spread = p95 - p05

    min_dark = max(32, len(values) // 200)
    min_mid = max(32, len(values) // 400)
    min_bright = max(32, len(values) // 200)
    min_spread = 90.0

    failures = []
    if dark_count < min_dark:
        failures.append(f"dark pixels {dark_count} < {min_dark}")
    if mid_count < min_mid:
        failures.append(f"mid-tone pixels {mid_count} < {min_mid}")
    if bright_count < min_bright:
        failures.append(f"bright pixels {bright_count} < {min_bright}")
    if spread < min_spread:
        failures.append(f"luminance spread {spread:.1f} < {min_spread:.1f}")
    if failures:
        raise RuntimeError(
            "scene region lacks expected shadow/lighting contrast: "
            + ", ".join(failures)
        )

    print(
        "scene luminance contrast: "
        f"dark={dark_count}, mid={mid_count}, bright={bright_count}, "
        f"p05={p05:.1f}, p95={p95:.1f}, spread={spread:.1f}"
    )


def _scene_region_mean_luminance(pixels: bytes, width: int, height: int) -> float:
    x0, y0, x1, y1 = _scene_region(width, height)
    total = 0.0
    count = 0
    for y in range(y0, y1):
        row_offset = y * width * 3
        for x in range(x0, x1):
            offset = row_offset + x * 3
            total += _luminance(pixels[offset : offset + 3])
            count += 1
    if count == 0:
        raise RuntimeError("scene luminance probe region is empty")
    return total / count


def _mean_abs_luminance_diff(
    pixels_a: bytes, pixels_b: bytes, width: int, height: int
) -> float:
    total = 0.0
    for offset in range(0, width * height * 3, 3):
        lum_a = _luminance(pixels_a[offset : offset + 3])
        lum_b = _luminance(pixels_b[offset : offset + 3])
        total += abs(lum_a - lum_b)
    return total / max(1, width * height)


def analyze_compare(
    path_a: Path,
    path_b: Path,
    width: int,
    height: int,
    min_divergence: float,
) -> None:
    """Assert two captures are non-blank and framed distinctly.

    Two different camera angles of the same scene must not collapse to the same
    (or a duplicated) image, so their image-evidence stats have to diverge
    beyond ``min_divergence``. The check combines the whole-image mean absolute
    per-pixel luminance difference with the scene-region mean-luminance delta and
    requires the larger of the two to clear the threshold.
    """

    pixels_a = _read_ppm(path_a, width, height)
    pixels_b = _read_ppm(path_b, width, height)
    _require_nonzero_pixels(pixels_a)
    _require_nonzero_pixels(pixels_b)

    mean_abs_diff = _mean_abs_luminance_diff(pixels_a, pixels_b, width, height)
    region_delta = abs(
        _scene_region_mean_luminance(pixels_a, width, height)
        - _scene_region_mean_luminance(pixels_b, width, height)
    )
    divergence = max(mean_abs_diff, region_delta)

    print(
        "two-view divergence: "
        f"mean_abs_diff={mean_abs_diff:.3f}, region_delta={region_delta:.3f}, "
        f"divergence={divergence:.3f} (min {min_divergence:.3f})"
    )
    if divergence < min_divergence:
        raise RuntimeError(
            f"{path_a} and {path_b} are not framed distinctly: divergence "
            f"{divergence:.3f} < {min_divergence:.3f} (duplicated frame?)"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument(
        "--mode", choices=("basic", "contrast", "compare"), default="contrast"
    )
    parser.add_argument("--compare", type=Path, default=None)
    parser.add_argument("--min-divergence", type=float, default=2.0)
    args = parser.parse_args()
    if args.mode == "basic":
        analyze_basic(args.path, args.width, args.height)
    elif args.mode == "compare":
        if args.compare is None:
            parser.error("--mode compare requires --compare PATH")
        analyze_compare(
            args.path, args.compare, args.width, args.height, args.min_divergence
        )
    else:
        analyze_contrast(args.path, args.width, args.height)


if __name__ == "__main__":
    main()
