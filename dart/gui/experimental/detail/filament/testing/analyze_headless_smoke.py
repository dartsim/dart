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


def analyze(path: Path, width: int, height: int) -> None:
    pixels = _read_ppm(path, width, height)
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    args = parser.parse_args()
    analyze(args.path, args.width, args.height)


if __name__ == "__main__":
    main()
