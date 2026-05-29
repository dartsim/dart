"""Simple frames scene: nested SimpleFrames with ellipsoid markers.

Mirrors examples/demos/scenes/simple_frames.cpp. The ArrowShape from the C++
scene is not bound in dartpy yet, so its arrow visual is substituted with a
small thin box that points in the same direction; the F1/F2/F3 chain and the
A/A1/A2/A3 ellipsoid markers are reproduced faithfully.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def _axis_angle_rotation(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [c + x * x * t,     x * y * t - z * s, x * z * t + y * s],
        [y * x * t + z * s, c + y * y * t,     y * z * t - x * s],
        [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
    ])


def _tf(translation, rotation_axis=None, angle: float = 0.0) -> np.ndarray:
    tf = np.eye(4)
    tf[:3, 3] = translation
    if rotation_axis is not None:
        tf[:3, :3] = _axis_angle_rotation(rotation_axis, angle)
    return tf


def _set_color(frame: "dart.SimpleFrame", color: tuple[float, float, float, float]) -> None:
    frame.create_visual_aspect().set_color(list(color))


def build() -> SceneSetup:
    world = dart.World("simple_frames")
    world.set_gravity([0.0, 0.0, 0.0])

    tf1 = _tf((0.1, -0.1, 0.0))
    tf2 = _tf((0.0, 0.1, 0.0), (1.0, 0.0, 0.0), math.pi / 4.0)
    tf3 = _tf((0.0, 0.0, 0.1), (0.0, 1.0, 0.0), math.pi / 3.0)

    f1 = dart.SimpleFrame(dart.Frame.world(), "F1", tf1)
    f1.set_shape(dart.BoxShape(np.array([0.05, 0.05, 0.02])))
    _set_color(f1, (0.92, 0.32, 0.24, 1.0))
    world.add_simple_frame(f1)

    f2 = dart.SimpleFrame(f1, "F2", tf2)
    f2.set_shape(dart.BoxShape(np.array([0.05, 0.05, 0.02])))
    _set_color(f2, (0.24, 0.58, 0.92, 1.0))
    world.add_simple_frame(f2)

    f3 = dart.SimpleFrame(f2, "F3", tf3)
    f3.set_shape(dart.BoxShape(np.array([0.05, 0.05, 0.02])))
    _set_color(f3, (0.28, 0.76, 0.34, 1.0))
    world.add_simple_frame(f3)

    marker_root = dart.SimpleFrame(dart.Frame.world(), "A", np.eye(4))
    marker_root.set_shape(dart.EllipsoidShape(np.array([0.02, 0.02, 0.02])))
    _set_color(marker_root, (0.95, 0.75, 0.20, 1.0))
    world.add_simple_frame(marker_root)

    # The C++ scene queries f<i>.getTransform(markerRoot.get()) to get the
    # frame-of-frame transform; identity at t=0 since marker_root is at origin
    # and f1/f2/f3 are constants we just defined.
    a1 = dart.SimpleFrame(marker_root, "A1", tf1)
    a1.set_shape(dart.EllipsoidShape(np.array([0.01, 0.01, 0.01])))
    _set_color(a1, (0.95, 0.75, 0.20, 1.0))
    world.add_simple_frame(a1)

    a2 = dart.SimpleFrame(marker_root, "A2", tf1 @ tf2)
    a2.set_shape(dart.EllipsoidShape(np.array([0.01, 0.01, 0.01])))
    _set_color(a2, (0.95, 0.75, 0.20, 1.0))
    world.add_simple_frame(a2)

    a3 = dart.SimpleFrame(marker_root, "A3", tf1 @ tf2 @ tf3)
    a3.set_shape(dart.EllipsoidShape(np.array([0.01, 0.01, 0.01])))
    _set_color(a3, (0.95, 0.75, 0.20, 1.0))
    world.add_simple_frame(a3)

    # ArrowShape substitute: a long thin box along +X.
    arrow_tf = _tf((0.15, -0.1, 0.0))
    arrow = dart.SimpleFrame(dart.Frame.world(), "arrow", arrow_tf)
    arrow.set_shape(dart.BoxShape(np.array([0.10, 0.004, 0.004])))
    _set_color(arrow, (1.0, 0.5, 0.5, 1.0))
    world.add_simple_frame(arrow)

    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="simple_frames",
    title="Simple Frames",
    category="Visualization",
    summary="Nested SimpleFrames with ellipsoid markers.",
    build=build,
)
