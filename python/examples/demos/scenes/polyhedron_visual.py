"""Polyhedron visual scene: wireframe + ground grid via LineSegmentShape.

Mirrors examples/demos/scenes/polyhedron_visual.cpp. The convex-hull surface
mesh uses `ConvexMeshShape` which is not bound in dartpy today, so we render
only the wireframe + ground grid (both LineSegment-based) — visually
equivalent to the C++ scene's `--cycle-scenes` smoke frame minus the shaded
surface fill.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


_VERTICES = [
    (-0.5, -0.5, 0.0),
    (0.5, -0.5, 0.2),
    (0.6, 0.5, 0.1),
    (-0.4, 0.6, 0.15),
    (-0.2, -0.2, 0.9),
    (0.35, -0.3, 0.8),
    (0.4, 0.35, 0.75),
    (-0.35, 0.4, 0.7),
]

_WIREFRAME_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def _static_visual_skel(name: str, shape: "dart.Shape", color: tuple, alpha: float = 1.0) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    _, body = skel.create_weld_joint_and_body_node_pair()
    sn = body.create_shape_node(shape)
    rgba = list(color) + [alpha]
    sn.create_visual_aspect().set_color(rgba)
    return skel


def _wireframe() -> "dart.LineSegmentShape":
    shape = dart.LineSegmentShape(2.0)
    for v in _VERTICES:
        shape.add_vertex(np.array(v))
    for a, b in _WIREFRAME_EDGES:
        shape.add_connection(a, b)
    return shape


def _ground_grid() -> "dart.LineSegmentShape":
    grid = dart.LineSegmentShape(1.5)
    half = 1.25
    n = 10
    idx = 0
    for i in range(n + 1):
        coord = -half + 2.0 * half * i / n
        grid.add_vertex(np.array([-half, coord, -0.02]))
        grid.add_vertex(np.array([half, coord, -0.02]))
        grid.add_connection(idx, idx + 1)
        idx += 2
        grid.add_vertex(np.array([coord, -half, -0.02]))
        grid.add_vertex(np.array([coord, half, -0.02]))
        grid.add_connection(idx, idx + 1)
        idx += 2
    return grid


def build() -> SceneSetup:
    world = dart.World("dartsim_polyhedron")
    world.set_gravity([0.0, 0.0, 0.0])
    world.add_skeleton(_static_visual_skel(
        "visual_polyhedron_wireframe", _wireframe(), (0.05, 0.05, 0.05)))
    world.add_skeleton(_static_visual_skel(
        "visual_polyhedron_grid", _ground_grid(), (0.42, 0.48, 0.44), 0.48))
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="polyhedron_visual",
    title="Polyhedron",
    category="Visualization",
    summary="A convex polyhedron rendered as a wireframe over a ground grid.",
    build=build,
)
