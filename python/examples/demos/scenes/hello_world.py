"""Hello-world scene: a single box falls onto a ground plane.

Mirrors the C++ `examples/demos/scenes/hello_world.cpp` scene byte-for-byte
(shape sizes, initial transform, masses, gravity) so the golden-set parity smoke
(PLAN-103 Phase 2) can assert both languages produce the same simulated state.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def _axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / norm, y / norm, z / norm
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [c + x * x * t,     x * y * t - z * s, x * z * t + y * s],
        [y * x * t + z * s, c + y * y * t,     y * z * t - x * s],
        [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
    ])


def _make_falling_box() -> dart.Skeleton:
    skel = dart.Skeleton("falling_box")
    joint, body = skel.create_free_joint_and_body_node_pair()

    transform = np.eye(4)
    transform[:3, 3] = (0.0, 0.0, 1.0)
    rotation = (
        _axis_angle((1.0, 0.0, 0.0), 0.35)
        @ _axis_angle((0.0, 1.0, 0.0), -0.45)
        @ _axis_angle((0.0, 0.0, 1.0), 0.25)
    )
    transform[:3, :3] = rotation
    joint.set_transform_from_parent_body_node(transform)

    shape = dart.BoxShape(np.array([0.3, 0.3, 0.3]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect().set_color([0.1, 0.2, 0.9])
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()

    mass = 1.0
    body.set_inertia(dart.Inertia(
        mass,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(np.array([0.3, 0.3, 0.3]), mass),
    ))
    return skel


def _make_ground() -> dart.Skeleton:
    skel = dart.Skeleton("ground")
    joint, body = skel.create_weld_joint_and_body_node_pair()

    transform = np.eye(4)
    transform[:3, 3] = (0.0, 0.0, -0.05)
    joint.set_transform_from_parent_body_node(transform)

    shape = dart.BoxShape(np.array([10.0, 10.0, 0.1]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect().set_color([0.8, 0.8, 0.8])
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()
    return skel


def build() -> SceneSetup:
    world = dart.World("hello_world")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_make_falling_box())
    world.add_skeleton(_make_ground())
    return SceneSetup(world=world, info={"golden_skeletons": ["falling_box"]})


SCENE = PythonDemoScene(
    id="hello_world",
    title="Hello World",
    category="Getting Started",
    summary="A single box falls onto a ground plane (mirrors C++ hello_world).",
    build=build,
)
