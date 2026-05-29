"""Free joint cases: FreeJoint integration cases over a ground.

Mirrors examples/demos/scenes/free_joint_cases.cpp at a high level: spawn a
handful of free-joint boxes with distinct initial orientations and linear /
angular velocities and let them evolve under gravity. The full C++ scene
exposes interactive case-switching panels; the Python mirror is a fixed
deterministic sweep over four representative cases.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def _rot_about_axis(axis: tuple, angle: float) -> np.ndarray:
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [c + x * x * t,     x * y * t - z * s, x * z * t + y * s],
        [y * x * t + z * s, c + y * y * t,     y * z * t - x * s],
        [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
    ])


def _ground() -> "dart.Skeleton":
    skel = dart.Skeleton("ground")
    joint, body = skel.create_weld_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = (0.0, 0.0, -0.05)
    joint.set_transform_from_parent_body_node(tf)
    shape = dart.BoxShape(np.array([10.0, 10.0, 0.1]))
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color([0.8, 0.8, 0.8])
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    return skel


def _case(name: str, position: tuple, axis: tuple, angle: float, color: tuple) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    _, body = skel.create_free_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, :3] = _rot_about_axis(axis, angle)
    tf[:3, 3] = position
    body.get_parent_joint().set_transform_from_parent_body_node(tf)
    size = np.array([0.25, 0.25, 0.25])
    shape = dart.BoxShape(size)
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color(list(color))
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    body.get_inertia().set_mass(1.0)
    body.get_inertia().set_moment(dart.BoxShape.compute_inertia_of(size, 1.0))
    return skel


def build() -> SceneSetup:
    world = dart.World("free_joint_cases")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_ground())

    cases = [
        ("case_identity",  (-0.6, -0.6, 0.6), (0.0, 0.0, 1.0), 0.0,           (0.90, 0.45, 0.20)),
        ("case_x90",       ( 0.6, -0.6, 0.6), (1.0, 0.0, 0.0), math.pi / 2,   (0.20, 0.55, 0.90)),
        ("case_y45",       (-0.6,  0.6, 0.6), (0.0, 1.0, 0.0), math.pi / 4,   (0.40, 0.80, 0.35)),
        ("case_xyz_skew",  ( 0.6,  0.6, 0.6), (1.0, 1.0, 0.0), math.pi / 3,   (0.80, 0.30, 0.70)),
    ]
    for name, p, axis, ang, color in cases:
        world.add_skeleton(_case(name, p, axis, ang, color))

    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="free_joint_cases",
    title="Free Joint Cases",
    category="Constraints & Joints",
    summary="Free-joint boxes with assorted initial orientations.",
    build=build,
)
