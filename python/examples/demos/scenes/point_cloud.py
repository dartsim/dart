"""Point cloud rendering scene.

Mirrors examples/demos/scenes/point_cloud.cpp at a high level. The C++
scene drives a robot through the workspace and samples points off its
geometry. The Python mirror builds a static deterministic cloud
(spherical Fibonacci lattice) so the PointCloudShape binding is
exercised end-to-end; the C++-only interactive controls are out of
scope for the headless mirror.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


_POINT_COUNT = 600
_SPHERE_RADIUS = 0.4
_VISUAL_SIZE = 0.02


def _fibonacci_sphere(n: int, radius: float) -> list[np.ndarray]:
    points: list[np.ndarray] = []
    phi = math.pi * (math.sqrt(5.0) - 1.0)
    for i in range(n):
        y = 1.0 - (i / float(n - 1)) * 2.0
        r = math.sqrt(max(0.0, 1.0 - y * y))
        theta = phi * i
        points.append(
            np.array([radius * math.cos(theta) * r, radius * y, radius * math.sin(theta) * r])
        )
    return points


def _make_point_cloud_frame() -> "dart.SimpleFrame":
    shape = dart.PointCloudShape(_VISUAL_SIZE)
    shape.add_points(_fibonacci_sphere(_POINT_COUNT, _SPHERE_RADIUS))
    shape.set_color_mode(dart.PointCloudShape.ColorMode.BIND_OVERALL)
    shape.set_overall_color(np.array([0.20, 0.38, 0.94, 1.0]))
    shape.set_point_shape_type(dart.PointCloudShape.PointShapeType.BOX)

    frame = dart.SimpleFrame(dart.Frame.World(), "point_cloud")
    frame.set_shape(shape)
    frame.create_visual_aspect().set_rgba(np.array([0.20, 0.38, 0.94, 1.0]))
    return frame


def _ground() -> "dart.Skeleton":
    skel = dart.Skeleton("point_cloud_ground")
    joint, body = skel.create_weld_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[2, 3] = -0.5
    joint.set_transform_from_parent_body_node(tf)
    shape = dart.BoxShape(np.array([4.0, 4.0, 0.04]))
    node = body.create_shape_node(shape)
    node.create_visual_aspect().set_rgba(np.array([0.42, 0.44, 0.40, 0.78]))
    return skel


def build() -> SceneSetup:
    world = dart.simulation.World("point_cloud")
    world.set_gravity([0.0, 0.0, 0.0])
    world.add_skeleton(_ground())
    world.add_simple_frame(_make_point_cloud_frame())
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="point_cloud",
    title="Point Cloud",
    category="Visualization",
    summary="Spherical Fibonacci point cloud rendered through PointCloudShape.",
    build=build,
)
