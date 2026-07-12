from __future__ import annotations

import ctypes
import ctypes.util
import os
import struct
import sys
import zlib

import numpy as np
import pytest

import dartpy as dart

os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")


def _can_open_linux_display() -> bool:
    if not sys.platform.startswith("linux"):
        return True
    display = os.environ.get("DISPLAY")
    if not display:
        return False
    lib_name = ctypes.util.find_library("X11")
    if not lib_name:
        return False
    try:
        lib_x11 = ctypes.CDLL(lib_name)
        lib_x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        lib_x11.XOpenDisplay.restype = ctypes.c_void_p
        lib_x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        handle = lib_x11.XOpenDisplay(display.encode())
        if not handle:
            return False
        lib_x11.XCloseDisplay(handle)
        return True
    except Exception:  # noqa: BLE001
        return False


pytestmark = pytest.mark.skipif(
    not hasattr(dart, "gui") or not hasattr(dart.gui, "OffscreenRenderer"),
    reason="dartpy GUI offscreen renderer is not available",
)
pytestmark = [
    pytestmark,
    pytest.mark.skipif(
        not _can_open_linux_display(),
        reason="Filament OpenGL headless rendering requires a usable DISPLAY/Xvfb",
    ),
]


def _box_descriptor(size: tuple[float, float, float] = (0.8, 0.8, 0.8)):
    descriptor = dart.gui.RenderableDescriptor()
    descriptor.id = 0xA50001
    descriptor.shape_frame_name = "python_offscreen_box"
    descriptor.shape_node_name = descriptor.shape_frame_name
    descriptor.geometry.kind = dart.gui.ShapeKind.Box
    descriptor.geometry.size = np.asarray(size, dtype=float)
    half = 0.5 * np.asarray(size, dtype=float)
    descriptor.geometry.local_bounds_min = -half
    descriptor.geometry.local_bounds_max = half
    descriptor.geometry.has_local_bounds = True
    descriptor.material.rgba = np.array([0.88, 0.18, 0.10, 1.0])
    descriptor.world_transform = np.eye(4)
    descriptor.render_resource_version = 1
    return descriptor


def _nonblank(image: object) -> bool:
    pixels = np.frombuffer(memoryview(image), dtype=np.uint8)
    rgb = pixels.reshape(-1, 4)[:, :3]
    return int(rgb.max()) - int(rgb.min()) > 12


def _decode_png_size(data: bytes) -> tuple[int, int]:
    assert data.startswith(b"\x89PNG\r\n\x1a\n")
    offset = 8
    width = height = None
    idat = bytearray()
    while offset < len(data):
        length = struct.unpack(">I", data[offset : offset + 4])[0]
        chunk_type = data[offset + 4 : offset + 8]
        payload = data[offset + 8 : offset + 8 + length]
        offset += 12 + length
        if chunk_type == b"IHDR":
            width, height = struct.unpack(">II", payload[:8])
        elif chunk_type == b"IDAT":
            idat.extend(payload)
        elif chunk_type == b"IEND":
            break
    assert width is not None and height is not None
    raw = zlib.decompress(bytes(idat))
    assert len(raw) == height * (1 + width * 4)
    return int(width), int(height)


def test_descriptor_offscreen_render_buffer_and_png():
    renderer = dart.gui.OffscreenRenderer(width=96, height=72)
    camera = dart.gui.orbit_camera(distance=3.0, target=(0.0, 0.0, 0.0))

    image = renderer.render([_box_descriptor()], camera)

    assert image.width == 96
    assert image.height == 72
    assert image.channels == 4
    view = memoryview(image)
    assert view.shape == (72, 96, 4)
    assert _nonblank(image)
    assert _decode_png_size(image.png_bytes()) == (96, 72)


def _settled_debug_world():
    world = dart.World()
    ground = world.add_rigid_body("debug_ground", position=(0.0, 0.0, -0.05))
    ground.is_static = True
    ground.set_collision_shape(dart.CollisionShape.box((1.4, 1.4, 0.1)))
    box = world.add_rigid_body("debug_box", position=(0.0, 0.0, 0.12))
    box.set_collision_shape(dart.CollisionShape.box((0.22, 0.22, 0.22)))
    for _ in range(50):
        world.step()
    return world


def _bytes_over_tolerance(a, b, tolerance: int = 4) -> int:
    pa = np.asarray(memoryview(a), dtype=np.uint8).astype(int)
    pb = np.asarray(memoryview(b), dtype=np.uint8).astype(int)
    return int((np.abs(pa - pb) > tolerance).sum())


def test_debug_scene_overlay_lifecycle_on_shared_renderer():
    # The overlay clear path only matters when one renderer is reused; a
    # fresh renderer per call could never leak. Same-renderer replays drift
    # by a few levels on some driver stacks, so compare with a noise budget:
    # the overlay signal must dwarf it and the cleared frames must sit
    # inside it (mirrors the C++ lifecycle test).
    world = _settled_debug_world()
    camera = dart.gui.frame_body(world, "debug_box", margin=3.0)
    from dartpy import _world_render_bridge as bridge

    descriptors = bridge._renderables_from_world(world)
    scene = dart.gui.debug_scene_for_world(
        world, layers=("grid", "body_frames", "contacts")
    )
    renderer = dart.gui.OffscreenRenderer(width=160, height=120)

    plain = renderer.render(descriptors, camera)
    debugged = renderer.render(descriptors, camera, debug=scene)
    plain_again = renderer.render(descriptors, camera)
    emptied = renderer.render(descriptors, camera, debug=dart.gui.DebugScene())

    assert _bytes_over_tolerance(debugged, plain) >= 256
    # The overlay must not leak into a plain render or survive an empty
    # DebugScene on the same renderer.
    assert _bytes_over_tolerance(plain_again, plain) <= 64
    assert _bytes_over_tolerance(emptied, plain) <= 64


def test_debug_scene_render_is_pure_across_fresh_renderers():
    world = _settled_debug_world()
    camera = dart.gui.frame_body(world, "debug_box", margin=3.0)
    debugged = dart.gui.render(
        world, camera, size=(160, 120), debug=("grid", "body_frames", "contacts")
    )
    debugged_again = dart.gui.render(
        world, camera, size=(160, 120), debug=("grid", "body_frames", "contacts")
    )
    assert bytes(memoryview(debugged)) == bytes(memoryview(debugged_again))


def test_render_annotated_composites_labels():
    world = dart.World()
    box = world.add_rigid_body("labeled_box", position=(0.0, 0.0, 0.3))
    box.set_collision_shape(dart.CollisionShape.box((0.2, 0.2, 0.2)))
    camera = dart.gui.frame_body(world, "labeled_box", margin=3.0)

    without_labels = dart.gui.render_annotated(world, camera, (160, 120))
    with_labels = dart.gui.render_annotated(
        world, camera, (160, 120), debug=("labels",)
    )
    assert with_labels.shape == (120, 160, 4)
    assert not np.array_equal(without_labels, with_labels)


def test_world_render_default_camera_is_stable():
    world = dart.World()
    ground = world.add_rigid_body("render_ground", position=(0.0, 0.0, -0.05))
    ground.is_static = True
    ground.set_collision_shape(dart.CollisionShape.box((1.4, 1.4, 0.05)))

    box = world.add_rigid_body("render_box", position=(0.0, 0.0, 0.35))
    box.set_collision_shape(dart.CollisionShape.box((0.22, 0.22, 0.22)))

    first = dart.gui.render(world, size=(96, 72))
    second = dart.gui.render(world, size=(96, 72))

    assert (first.width, first.height, first.channels) == (96, 72, 4)
    assert _nonblank(first)
    assert bytes(memoryview(first)) == bytes(memoryview(second))
