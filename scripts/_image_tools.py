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

    if data[index : index + 2] == b"\r\n":
        index += 2
    elif index < len(data) and data[index] in b" \t\r\n":
        index += 1
    expected = width * height * 3
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
