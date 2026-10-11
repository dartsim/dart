"""Display-free capture writes a correctly sized, nonblank Filament image.

PNG decoding uses the standard library so the gate needs no image dependency.
"""

import struct
import zlib

import dartpy as dart
import pytest


def _read_png_luminance(path):
    """Minimal stdlib PNG reader for 8-bit gray/RGB/RGBA. Returns
    (width, height, luminance_variance). A blank (uniform) frame has variance 0.
    """
    with open(path, "rb") as f:
        data = f.read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
    pos = 8
    width = height = bitdepth = colortype = None
    idat = bytearray()
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        ctype = data[pos + 4 : pos + 8]
        chunk = data[pos + 8 : pos + 8 + length]
        if ctype == b"IHDR":
            width, height, bitdepth, colortype = struct.unpack(">IIBB", chunk[:10])
        elif ctype == b"IDAT":
            idat += chunk
        elif ctype == b"IEND":
            break
        pos += 12 + length
    assert bitdepth == 8, f"expected 8-bit PNG, got {bitdepth}"
    channels = {0: 1, 2: 3, 4: 2, 6: 4}[colortype]
    raw = zlib.decompress(bytes(idat))
    stride = width * channels
    out = bytearray()
    prev = bytearray(stride)
    p = 0
    for _ in range(height):
        ftype = raw[p]
        p += 1
        line = bytearray(raw[p : p + stride])
        p += stride
        for i in range(stride):
            a = line[i - channels] if i >= channels else 0
            b = prev[i]
            c = prev[i - channels] if i >= channels else 0
            x = line[i]
            if ftype == 1:
                x += a
            elif ftype == 2:
                x += b
            elif ftype == 3:
                x += (a + b) >> 1
            elif ftype == 4:
                pp = a + b - c
                pa, pb, pc = abs(pp - a), abs(pp - b), abs(pp - c)
                x += a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
            line[i] = x & 0xFF
        out += line
        prev = line

    n = width * height
    total = 0.0
    total_sq = 0.0
    for i in range(0, len(out), channels):
        if channels >= 3:
            lum = 0.299 * out[i] + 0.587 * out[i + 1] + 0.114 * out[i + 2]
        else:
            lum = out[i]
        total += lum
        total_sq += lum * lum
    mean = total / n
    return width, height, total_sq / n - mean * mean


def _build_box_world():
    """A trivial one-body scene: a single colored box at the origin, enough
    (with the viewer's default lighting) that a correct render is non-blank."""
    world = dart.simulation.World()
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, 0.0])
    box = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "box", tf)
    box.setShape(dart.dynamics.BoxShape([1.0, 1.0, 1.0]))
    box.getVisualAspect(True).setColor([0.8, 0.2, 0.2])
    world.addSimpleFrame(box)
    return world


def test_capture_screen_non_blank(tmp_path):
    world = _build_box_world()
    viewer = dart.gui.Viewer()
    viewer.addWorldNode(dart.gui.WorldNode(world))
    viewer.setUpViewForCapture(320, 240)
    viewer.setCameraHome([2.0, -3.0, 2.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    for _ in range(5):
        viewer.frame()
    out = tmp_path / "capture.png"
    viewer.captureScreen(str(out))
    viewer.frame()

    width, height, variance = _read_png_luminance(out)
    assert (width, height) == (320, 240)
    assert variance > 1e-4, f"image looks blank (variance {variance})"
