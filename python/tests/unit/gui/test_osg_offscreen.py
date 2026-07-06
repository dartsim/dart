"""Headless off-screen capture tests for the release-6.20 dartpy OSG bindings
(WP-ASV.10). They exercise the setUpOffscreen / captureOffscreen /
defaultAgentCamera bindings added on top of the WP-ASV.9 C++ helper.

The capture tests need a GLX pbuffer, i.e. a real X server. They skip cleanly
when no DISPLAY is available; on a headless host run pytest under
`xvfb-run -a -s '-screen 0 1280x1024x24'`. PNG decoding is stdlib-only (no
Pillow) so the gate stays dependency-free.
"""

import os
import struct
import zlib

import pytest

import dartpy as dart

# The off-screen pbuffer path requires an X server.
pytestmark = pytest.mark.skipif(
    not os.environ.get("DISPLAY"),
    reason="off-screen GLX capture needs a DISPLAY (run under xvfb-run on "
    "headless hosts)",
)

# Skip the whole module if dartpy was built without the OSG GUI.
try:
    _osg = dart.gui.osg
except AttributeError:  # pragma: no cover - depends on build config
    pytest.skip("dartpy built without gui.osg", allow_module_level=True)


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


def test_capture_offscreen_non_blank(tmp_path):
    """captureOffscreen sets up the pbuffer, frames the scene, and writes a
    non-blank PNG in one call."""
    world = _build_box_world()
    viewer = dart.gui.osg.ImGuiViewer()
    node = dart.gui.osg.WorldNode(world)
    viewer.addWorldNode(node)

    eye, center, up = dart.gui.osg.defaultAgentCamera([0.0, 0.0, 0.0], 0.87)
    out = str(tmp_path / "capture_offscreen.png")
    if not viewer.captureOffscreen(out, eye, center, up, width=320, height=240):
        pytest.skip("no off-screen GL context (no usable DISPLAY)")

    width, height, variance = _read_png_luminance(out)
    assert (width, height) == (320, 240)
    assert variance > 1e-4, f"image looks blank (variance {variance})"


def test_set_up_offscreen_then_capture_screen(tmp_path):
    """setUpOffscreen configures an existing viewer for headless rendering so
    the already-bound captureScreen writes a valid PNG without a window."""
    world = _build_box_world()
    viewer = dart.gui.osg.ImGuiViewer()
    node = dart.gui.osg.WorldNode(world)
    viewer.addWorldNode(node)

    if not viewer.setUpOffscreen(width=320, height=240):
        pytest.skip("no off-screen GL context (no usable DISPLAY)")

    for _ in range(5):
        viewer.frame()
    out = str(tmp_path / "set_up_offscreen.png")
    viewer.captureScreen(out)
    viewer.frame()  # SaveScreen writes the PNG during this frame.

    width, height, _ = _read_png_luminance(out)
    assert (width, height) == (320, 240)


def test_default_agent_camera_geometry():
    """defaultAgentCamera fits the bounding sphere to the vertical FOV with a
    z-up axis; no DISPLAY needed for the pure geometry."""
    import numpy as np

    eye, center, up = dart.gui.osg.defaultAgentCamera(
        [0.0, 0.0, 0.0], 1.0, fovYDeg=30.0
    )
    distance = np.linalg.norm(np.asarray(eye) - np.asarray(center))
    expected = 1.0 / np.sin(np.radians(30.0) / 2.0)
    assert distance == pytest.approx(expected, rel=1e-4)
    assert np.allclose(np.asarray(center), [0.0, 0.0, 0.0])
    assert np.allclose(np.asarray(up), [0.0, 0.0, 1.0])
