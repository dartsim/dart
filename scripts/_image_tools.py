#!/usr/bin/env python3
"""Dependency-free RGB image helpers for agent-time visual verification."""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


@dataclass(frozen=True)
class ImageData:
    path: Path | None
    width: int
    height: int
    pixels: bytes

    @property
    def pixel_count(self) -> int:
        return self.width * self.height


def read_image(path: Path) -> ImageData:
    data = path.read_bytes()
    if data.startswith(b"P6"):
        width, height, pixels = read_ppm_bytes(data, path)
    elif data.startswith(PNG_SIGNATURE):
        width, height, pixels = read_png_bytes(data, path)
    else:
        raise ValueError(f"{path}: expected a binary PPM or PNG image")
    return ImageData(path=path, width=width, height=height, pixels=pixels)


def read_ppm_bytes(data: bytes, path: Path | None = None) -> tuple[int, int, bytes]:
    label = str(path) if path is not None else "<ppm>"
    token, index = _next_ppm_token(data, 0, label)
    if token != b"P6":
        raise ValueError(f"{label}: expected a binary PPM (P6)")
    width_token, index = _next_ppm_token(data, index, label)
    height_token, index = _next_ppm_token(data, index, label)
    maxval_token, index = _next_ppm_token(data, index, label)
    try:
        width = int(width_token)
        height = int(height_token)
        maxval = int(maxval_token)
    except ValueError as exc:
        raise ValueError(f"{label}: invalid PPM header") from exc
    if width <= 0 or height <= 0:
        raise ValueError(f"{label}: invalid PPM dimensions {width}x{height}")
    if maxval != 255:
        raise ValueError(f"{label}: expected an 8-bit PPM maxval of 255")

    expected = width * height * 3
    while (
        index < len(data)
        and data[index] in b" \t\r\n"
        and len(data) - (index + 1) >= expected
    ):
        index += 1
    pixels = data[index : index + expected]
    if len(pixels) < expected:
        raise ValueError(f"{label}: truncated PPM pixel payload")
    return width, height, pixels


def write_ppm(path: Path, width: int, height: int, pixels: bytes) -> None:
    _require_rgb_size(width, height, pixels)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)


def read_png_bytes(data: bytes, path: Path | None = None) -> tuple[int, int, bytes]:
    label = str(path) if path is not None else "<png>"
    if not data.startswith(PNG_SIGNATURE):
        raise ValueError(f"{label}: expected a PNG signature")

    offset = len(PNG_SIGNATURE)
    width = height = bit_depth = color_type = interlace = None
    idat = bytearray()
    saw_iend = False
    while offset < len(data):
        if offset + 12 > len(data):
            raise ValueError(f"{label}: truncated PNG chunk header")
        length = struct.unpack(">I", data[offset : offset + 4])[0]
        kind = data[offset + 4 : offset + 8]
        payload_start = offset + 8
        payload_end = payload_start + length
        if payload_end + 4 > len(data):
            raise ValueError(f"{label}: truncated PNG chunk payload")
        payload = data[payload_start:payload_end]
        offset = payload_end + 4

        if kind == b"IHDR":
            if length != 13:
                raise ValueError(f"{label}: invalid IHDR length")
            (
                width,
                height,
                bit_depth,
                color_type,
                compression,
                filter_method,
                interlace,
            ) = struct.unpack(">IIBBBBB", payload)
            if compression != 0 or filter_method != 0:
                raise ValueError(f"{label}: unsupported PNG compression/filter method")
        elif kind == b"IDAT":
            idat.extend(payload)
        elif kind == b"IEND":
            saw_iend = True
            break

    if not saw_iend:
        raise ValueError(f"{label}: missing PNG IEND chunk")
    if width is None or height is None:
        raise ValueError(f"{label}: missing PNG IHDR chunk")
    if width <= 0 or height <= 0:
        raise ValueError(f"{label}: invalid PNG dimensions {width}x{height}")
    if bit_depth != 8:
        raise ValueError(f"{label}: only 8-bit PNG images are supported")
    if color_type not in (0, 2, 4, 6):
        raise ValueError(f"{label}: unsupported PNG color type {color_type}")
    if interlace != 0:
        raise ValueError(f"{label}: interlaced PNG images are not supported")

    channels = {0: 1, 2: 3, 4: 2, 6: 4}[color_type]
    stride = width * channels
    try:
        raw = zlib.decompress(bytes(idat))
    except zlib.error as exc:
        raise ValueError(f"{label}: invalid PNG image data") from exc
    expected = height * (1 + stride)
    if len(raw) < expected:
        raise ValueError(f"{label}: truncated PNG image data")

    rows: list[bytes] = []
    previous = bytearray(stride)
    cursor = 0
    for _y in range(height):
        filter_type = raw[cursor]
        cursor += 1
        scanline = bytearray(raw[cursor : cursor + stride])
        cursor += stride
        _unfilter_png_scanline(scanline, previous, channels, filter_type, label)
        rows.append(bytes(scanline))
        previous = scanline

    rgb = bytearray(width * height * 3)
    out = 0
    for row in rows:
        if color_type == 0:
            for gray in row:
                rgb[out : out + 3] = bytes((gray, gray, gray))
                out += 3
        elif color_type == 2:
            rgb[out : out + len(row)] = row
            out += len(row)
        elif color_type == 4:
            for offset in range(0, len(row), 2):
                gray = row[offset]
                rgb[out : out + 3] = bytes((gray, gray, gray))
                out += 3
        else:
            for offset in range(0, len(row), 4):
                rgb[out : out + 3] = row[offset : offset + 3]
                out += 3
    return width, height, bytes(rgb)


def write_png(path: Path, width: int, height: int, pixels: bytes) -> None:
    _require_rgb_size(width, height, pixels)
    rows = [
        b"\x00" + pixels[y * width * 3 : (y + 1) * width * 3] for y in range(height)
    ]
    png = (
        PNG_SIGNATURE
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(b"".join(rows), 6))
        + _png_chunk(b"IEND", b"")
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def write_image(path: Path, image: ImageData) -> None:
    suffix = path.suffix.lower()
    if suffix == ".ppm":
        write_ppm(path, image.width, image.height, image.pixels)
    elif suffix == ".png":
        write_png(path, image.width, image.height, image.pixels)
    else:
        raise ValueError(f"{path}: output extension must be .ppm or .png")


def resize_nearest(image: ImageData, width: int, height: int) -> bytes:
    if width <= 0 or height <= 0:
        raise ValueError("resize dimensions must be positive")
    out = bytearray(width * height * 3)
    for y in range(height):
        src_y = min(image.height - 1, y * image.height // height)
        for x in range(width):
            src_x = min(image.width - 1, x * image.width // width)
            src = (src_y * image.width + src_x) * 3
            dst = (y * width + x) * 3
            out[dst : dst + 3] = image.pixels[src : src + 3]
    return bytes(out)


def fit_to_tile(
    image: ImageData,
    tile_width: int,
    tile_height: int,
    background: tuple[int, int, int],
) -> bytes:
    if tile_width <= 0 or tile_height <= 0:
        raise ValueError("tile dimensions must be positive")
    scale = min(tile_width / image.width, tile_height / image.height)
    scaled_width = max(1, min(tile_width, round(image.width * scale)))
    scaled_height = max(1, min(tile_height, round(image.height * scale)))
    scaled = resize_nearest(image, scaled_width, scaled_height)
    tile = bytearray(background * (tile_width * tile_height))
    x0 = (tile_width - scaled_width) // 2
    y0 = (tile_height - scaled_height) // 2
    paste_rgb(tile, tile_width, x0, y0, scaled_width, scaled_height, scaled)
    return bytes(tile)


def paste_rgb(
    canvas: bytearray,
    canvas_width: int,
    x0: int,
    y0: int,
    width: int,
    height: int,
    pixels: bytes,
) -> None:
    _require_rgb_size(width, height, pixels)
    for y in range(height):
        dst = ((y0 + y) * canvas_width + x0) * 3
        src = y * width * 3
        canvas[dst : dst + width * 3] = pixels[src : src + width * 3]


def _next_ppm_token(data: bytes, index: int, label: str) -> tuple[bytes, int]:
    while index < len(data):
        byte = data[index]
        if byte in b" \t\r\n":
            index += 1
            continue
        if byte == ord("#"):
            newline = data.find(b"\n", index)
            if newline == -1:
                raise ValueError(f"{label}: unterminated PPM comment")
            index = newline + 1
            continue
        break
    if index >= len(data):
        raise ValueError(f"{label}: incomplete PPM header")
    start = index
    while index < len(data) and data[index] not in b" \t\r\n":
        index += 1
    return data[start:index], index


def _unfilter_png_scanline(
    scanline: bytearray,
    previous: bytearray,
    bytes_per_pixel: int,
    filter_type: int,
    label: str,
) -> None:
    if filter_type == 0:
        return
    if filter_type == 1:
        for index in range(len(scanline)):
            left = scanline[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
            scanline[index] = (scanline[index] + left) & 0xFF
        return
    if filter_type == 2:
        for index, up in enumerate(previous):
            scanline[index] = (scanline[index] + up) & 0xFF
        return
    if filter_type == 3:
        for index, up in enumerate(previous):
            left = scanline[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
            scanline[index] = (scanline[index] + ((left + up) // 2)) & 0xFF
        return
    if filter_type == 4:
        for index, up in enumerate(previous):
            left = scanline[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
            up_left = (
                previous[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
            )
            scanline[index] = (scanline[index] + _paeth(left, up, up_left)) & 0xFF
        return
    raise ValueError(f"{label}: unsupported PNG filter type {filter_type}")


def _paeth(left: int, up: int, up_left: int) -> int:
    estimate = left + up - up_left
    distance_left = abs(estimate - left)
    distance_up = abs(estimate - up)
    distance_up_left = abs(estimate - up_left)
    if distance_left <= distance_up and distance_left <= distance_up_left:
        return left
    if distance_up <= distance_up_left:
        return up
    return up_left


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)
    )


def _require_rgb_size(width: int, height: int, pixels: bytes) -> None:
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image dimensions {width}x{height}")
    expected = width * height * 3
    if len(pixels) != expected:
        raise ValueError(f"RGB payload has {len(pixels)} bytes, expected {expected}")


# --- Composite helpers (side-by-side, blend, diff heatmap, text) -----------

# Compact 3x5 uppercase bitmap font shared by composite labels; mirrors the
# glyph set used by dartpy's debug-label compositing.
FONT_3X5_TEXT = {
    "A": ("010", "101", "111", "101", "101"),
    "B": ("110", "101", "110", "101", "110"),
    "C": ("011", "100", "100", "100", "011"),
    "D": ("110", "101", "101", "101", "110"),
    "E": ("111", "100", "110", "100", "111"),
    "F": ("111", "100", "110", "100", "100"),
    "G": ("011", "100", "101", "101", "011"),
    "H": ("101", "101", "111", "101", "101"),
    "I": ("111", "010", "010", "010", "111"),
    "J": ("001", "001", "001", "101", "010"),
    "K": ("101", "110", "100", "110", "101"),
    "L": ("100", "100", "100", "100", "111"),
    "M": ("101", "111", "111", "101", "101"),
    "N": ("101", "111", "111", "111", "101"),
    "O": ("010", "101", "101", "101", "010"),
    "P": ("110", "101", "110", "100", "100"),
    "Q": ("010", "101", "101", "011", "001"),
    "R": ("110", "101", "110", "110", "101"),
    "S": ("011", "100", "010", "001", "110"),
    "T": ("111", "010", "010", "010", "010"),
    "U": ("101", "101", "101", "101", "111"),
    "V": ("101", "101", "101", "010", "010"),
    "W": ("101", "101", "111", "111", "101"),
    "X": ("101", "010", "010", "010", "101"),
    "Y": ("101", "101", "010", "010", "010"),
    "Z": ("111", "001", "010", "100", "111"),
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "011", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "001", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
    "-": ("000", "000", "111", "000", "000"),
    "_": ("000", "000", "000", "000", "111"),
    ".": ("000", "000", "000", "000", "010"),
    ":": ("000", "010", "000", "010", "000"),
    "/": ("001", "001", "010", "100", "100"),
    "+": ("000", "010", "111", "010", "000"),
    "(": ("010", "100", "100", "100", "010"),
    ")": ("010", "001", "001", "001", "010"),
    " ": ("000", "000", "000", "000", "000"),
}


def draw_text_rgb(
    pixels: bytearray,
    width: int,
    height: int,
    text: str,
    origin: tuple[int, int],
    color: tuple[int, int, int] = (255, 255, 255),
    scale: int = 2,
) -> None:
    """Draw uppercase bitmap text into a raw RGB byte buffer."""
    x_cursor, y_origin = int(origin[0]), int(origin[1])
    for character in str(text).upper():
        glyph = FONT_3X5_TEXT.get(character, FONT_3X5_TEXT[" "])
        for row_index, row in enumerate(glyph):
            for column_index, bit in enumerate(row):
                if bit != "1":
                    continue
                for dy in range(scale):
                    y = y_origin + row_index * scale + dy
                    if y < 0 or y >= height:
                        continue
                    for dx in range(scale):
                        x = x_cursor + column_index * scale + dx
                        if x < 0 or x >= width:
                            continue
                        offset = (y * width + x) * 3
                        pixels[offset : offset + 3] = bytes(color)
        x_cursor += (3 + 1) * scale


def side_by_side(
    images: list[ImageData],
    labels: list[str] | None = None,
    gap: int = 8,
    background: tuple[int, int, int] = (24, 28, 32),
    label_scale: int = 2,
) -> ImageData:
    """Compose images horizontally with optional per-panel labels."""
    if not images:
        raise ValueError("side_by_side requires at least one image")
    if labels is not None and len(labels) != len(images):
        raise ValueError("labels must match images one-to-one")
    tile_height = max(image.height for image in images)
    label_band = (5 * label_scale + 6) if labels else 0
    total_width = sum(image.width for image in images) + gap * (len(images) + 1)
    total_height = tile_height + 2 * gap + label_band
    canvas = bytearray(bytes(background) * (total_width * total_height))
    x_offset = gap
    for index, image in enumerate(images):
        paste_rgb(
            canvas,
            total_width,
            x_offset,
            gap + label_band,
            image.width,
            image.height,
            image.pixels,
        )
        if labels:
            draw_text_rgb(
                canvas,
                total_width,
                total_height,
                labels[index],
                (x_offset, 3),
                scale=label_scale,
            )
        x_offset += image.width + gap
    return ImageData(
        path=Path("side_by_side"),
        width=total_width,
        height=total_height,
        pixels=bytes(canvas),
    )


def overlay_blend(a: ImageData, b: ImageData, alpha: float = 0.5) -> ImageData:
    """Blend two equal-size images: result = (1-alpha)*a + alpha*b."""
    if (a.width, a.height) != (b.width, b.height):
        raise ValueError("overlay_blend requires equal image sizes")
    alpha = min(max(float(alpha), 0.0), 1.0)
    blended = bytearray(len(a.pixels))
    inverse = 1.0 - alpha
    for index in range(len(a.pixels)):
        blended[index] = int(a.pixels[index] * inverse + b.pixels[index] * alpha)
    return ImageData(
        path=Path("overlay_blend"),
        width=a.width,
        height=a.height,
        pixels=bytes(blended),
    )


def diff_heatmap(
    a: ImageData, b: ImageData, amplify: float = 4.0
) -> tuple[ImageData, dict]:
    """Colorized absolute difference plus summary statistics.

    Unchanged pixels stay dark; differences map through a blue->yellow->red
    ramp scaled by ``amplify`` so subtle changes stay visible.
    """
    if (a.width, a.height) != (b.width, b.height):
        raise ValueError("diff_heatmap requires equal image sizes")
    heat = bytearray(len(a.pixels))
    total = a.width * a.height
    changed = 0
    accumulated = 0
    peak = 0
    for pixel in range(total):
        offset = pixel * 3
        delta = max(
            abs(a.pixels[offset] - b.pixels[offset]),
            abs(a.pixels[offset + 1] - b.pixels[offset + 1]),
            abs(a.pixels[offset + 2] - b.pixels[offset + 2]),
        )
        accumulated += delta
        peak = max(peak, delta)
        if delta > 0:
            changed += 1
        magnitude = min(int(delta * amplify), 255)
        if magnitude <= 0:
            continue
        # Continuous blue -> yellow -> red ramp over the amplified magnitude.
        position = magnitude / 255.0
        if position < 0.5:
            blend = position * 2.0
            heat[offset] = int(255 * blend)
            heat[offset + 1] = int(255 * blend)
            heat[offset + 2] = int(255 * (1.0 - blend))
        else:
            blend = (position - 0.5) * 2.0
            heat[offset] = 255
            heat[offset + 1] = int(255 * (1.0 - blend))
            heat[offset + 2] = 0
    stats = {
        "changed_pixel_fraction": changed / total if total else 0.0,
        "mean_abs": accumulated / total if total else 0.0,
        "max_abs": peak,
    }
    image = ImageData(
        path=Path("diff_heatmap"), width=a.width, height=a.height, pixels=bytes(heat)
    )
    return image, stats
