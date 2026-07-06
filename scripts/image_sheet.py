#!/usr/bin/env python3
"""Assemble captured frames into an agent-readable contact sheet."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from _image_tools import fit_to_tile, paste_rgb, read_image, write_png

SCHEMA_VERSION = "dart.image_sheet/v1"
MIN_TILE_SIZE = 200
DEFAULT_BACKGROUND = (24, 28, 32)
DEFAULT_TILE_BACKGROUND = (16, 18, 20)
LABEL_BACKGROUND = (0, 0, 0)
LABEL_COLOR = (255, 255, 255)


FONT_3X5 = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "010", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
    "-": ("000", "000", "111", "000", "000"),
}


def assemble_sheet(
    inputs: list[Path],
    output: Path,
    *,
    cols: int = 3,
    rows: int = 3,
    tile_size: int = MIN_TILE_SIZE,
    labels: bool = True,
    label_start: int = 0,
) -> dict[str, object]:
    if not inputs:
        raise ValueError("at least one input frame is required")
    if cols <= 0 or rows <= 0:
        raise ValueError("grid dimensions must be positive")
    if tile_size < MIN_TILE_SIZE:
        raise ValueError(f"tile size must be at least {MIN_TILE_SIZE} px per edge")
    capacity = cols * rows
    if len(inputs) > capacity:
        raise ValueError(
            f"{len(inputs)} input frames exceed {cols}x{rows} grid capacity"
        )

    images = [read_image(path) for path in inputs]
    sheet_width = cols * tile_size
    sheet_height = rows * tile_size
    canvas = bytearray(DEFAULT_BACKGROUND * (sheet_width * sheet_height))

    for index, image in enumerate(images):
        col = index % cols
        row = index // cols
        x0 = col * tile_size
        y0 = row * tile_size
        tile = fit_to_tile(image, tile_size, tile_size, DEFAULT_TILE_BACKGROUND)
        paste_rgb(canvas, sheet_width, x0, y0, tile_size, tile_size, tile)
        if labels:
            _draw_label(
                canvas,
                sheet_width,
                sheet_height,
                x0 + 6,
                y0 + 6,
                str(label_start + index),
            )

    write_png(output, sheet_width, sheet_height, bytes(canvas))
    return {
        "schema_version": SCHEMA_VERSION,
        "path": str(output),
        "width": sheet_width,
        "height": sheet_height,
        "grid": {"cols": cols, "rows": rows},
        "tile_size": tile_size,
        "frames": [
            {
                "path": str(path),
                "width": image.width,
                "height": image.height,
                "label": str(label_start + index) if labels else None,
            }
            for index, (path, image) in enumerate(zip(inputs, images))
        ],
    }


def parse_grid(value: str) -> tuple[int, int]:
    if "x" not in value.lower():
        raise ValueError("--grid must be formatted as COLSxROWS, for example 3x3")
    cols_text, rows_text = value.lower().split("x", 1)
    try:
        cols = int(cols_text)
        rows = int(rows_text)
    except ValueError as exc:
        raise ValueError("--grid dimensions must be integers") from exc
    if cols <= 0 or rows <= 0:
        raise ValueError("--grid dimensions must be positive")
    return cols, rows


def _draw_label(
    canvas: bytearray,
    canvas_width: int,
    canvas_height: int,
    x: int,
    y: int,
    text: str,
) -> None:
    scale = 2
    spacing = 1
    glyph_width = 3 * scale
    glyph_height = 5 * scale
    label_width = len(text) * glyph_width + max(0, len(text) - 1) * spacing * scale
    label_height = glyph_height
    _fill_rect(
        canvas,
        canvas_width,
        canvas_height,
        x - 3,
        y - 3,
        label_width + 6,
        label_height + 6,
        LABEL_BACKGROUND,
    )
    cursor = x
    for char in text:
        glyph = FONT_3X5.get(char)
        if glyph is not None:
            _draw_glyph(canvas, canvas_width, canvas_height, cursor, y, glyph, scale)
        cursor += glyph_width + spacing * scale


def _draw_glyph(
    canvas: bytearray,
    canvas_width: int,
    canvas_height: int,
    x: int,
    y: int,
    glyph: tuple[str, ...],
    scale: int,
) -> None:
    for row, bits in enumerate(glyph):
        for col, bit in enumerate(bits):
            if bit == "1":
                _fill_rect(
                    canvas,
                    canvas_width,
                    canvas_height,
                    x + col * scale,
                    y + row * scale,
                    scale,
                    scale,
                    LABEL_COLOR,
                )


def _fill_rect(
    canvas: bytearray,
    canvas_width: int,
    canvas_height: int,
    x: int,
    y: int,
    width: int,
    height: int,
    color: tuple[int, int, int],
) -> None:
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(canvas_width, x + width)
    y1 = min(canvas_height, y + height)
    if x0 >= x1 or y0 >= y1:
        return
    row = bytes(color) * (x1 - x0)
    for yy in range(y0, y1):
        start = (yy * canvas_width + x0) * 3
        canvas[start : start + len(row)] = row


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build a labeled contact-sheet PNG.")
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--grid", default="3x3", help="grid as COLSxROWS (default: 3x3)"
    )
    parser.add_argument("--tile-size", type=int, default=MIN_TILE_SIZE)
    parser.add_argument("--no-labels", action="store_true")
    parser.add_argument("--label-start", type=int, default=0)
    args = parser.parse_args(argv)

    try:
        cols, rows = parse_grid(args.grid)
        result = assemble_sheet(
            args.inputs,
            args.out,
            cols=cols,
            rows=rows,
            tile_size=args.tile_size,
            labels=not args.no_labels,
            label_start=args.label_start,
        )
    except (OSError, ValueError) as exc:
        print(f"image_sheet.py: {exc}", file=sys.stderr)
        return 2

    sys.stdout.write(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
