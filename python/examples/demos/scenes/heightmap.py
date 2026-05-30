"""Heightmap rendering scene.

Mirrors examples/demos/scenes/heightmap.cpp at a high level. The C++
scene exposes an interactive panel for regenerating the heightmap; the
Python mirror builds a deterministic 25x25 ridge-and-sine terrain and
attaches it to a SimpleFrame so the HeightmapShape binding is
exercised end-to-end (the panel controls are GUI-only and live in C++).
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


_X_RES = 25
_Y_RES = 25
_X_SIZE = 2.0
_Y_SIZE = 2.0
_Z_MIN = -0.1
_Z_MAX = 0.4


def _generate_heights(
    x_res: int, y_res: int, z_min: float, z_max: float, generation: int = 0
) -> list[float]:
    if z_max < z_min:
        z_min, z_max = z_max, z_min
    heights: list[float] = []
    phase = generation * 0.17
    for y in range(y_res):
        for x in range(x_res):
            x_phase = x * 0.31 + phase
            y_phase = y * 0.27 - phase * 0.5
            ridge = 0.18 if x == x_res // 2 or y == y_res // 2 else 0.0
            normalized = max(
                0.0,
                min(1.0, 0.5 + ridge + 0.24 * math.sin(x_phase) * math.cos(y_phase)),
            )
            heights.append(z_min + (z_max - z_min) * normalized)
    return heights


def _make_terrain() -> "dart.SimpleFrame":
    shape = dart.HeightmapShape()
    shape.set_height_field(_X_RES, _Y_RES, _generate_heights(_X_RES, _Y_RES, _Z_MIN, _Z_MAX))
    x_stride = _X_RES - 1 if _X_RES > 1 else 1
    y_stride = _Y_RES - 1 if _Y_RES > 1 else 1
    shape.set_scale(np.array([_X_SIZE / x_stride, _Y_SIZE / y_stride, 1.0]))

    frame = dart.SimpleFrame(dart.Frame.World(), "heightmap_terrain")
    frame.set_shape(shape)
    visual = frame.create_visual_aspect()
    visual.set_rgba(np.array([0.24, 0.58, 0.88, 1.0]))
    return frame


def _make_static_box(name: str, size, position, color) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    _joint, body = skel.create_weld_joint_and_body_node_pair()
    shape = dart.BoxShape(np.asarray(size, dtype=float))
    node = body.create_shape_node(shape)
    node.set_relative_translation(np.asarray(position, dtype=float))
    node.create_visual_aspect().set_rgba(np.asarray(color, dtype=float))
    return skel


def build() -> SceneSetup:
    world = dart.simulation.World("heightmap")
    world.set_gravity([0.0, 0.0, 0.0])
    world.add_simple_frame(_make_terrain())

    world.add_skeleton(
        _make_static_box(
            "heightmap_reference_box",
            [0.48, 0.48, 0.28],
            [0.72, 0.0, 0.20],
            [0.20, 0.72, 0.28, 0.48],
        )
    )
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="heightmap",
    title="Heightmap",
    category="Visualization",
    summary="Ridge + sine terrain rendered through HeightmapShape.",
    build=build,
)
