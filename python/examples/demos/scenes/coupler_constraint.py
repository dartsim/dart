"""Coupler constraint (simplified): two revolute pendulums.

Mirrors examples/demos/scenes/coupler_constraint.cpp at a high level. The
C++ scene compares a coupler constraint with a mimic motor; both features
need bindings (`dart.dynamics.Coupler`, mimic motor) that are not yet
exposed in dartpy. The Python mirror builds two unconstrained revolute
pendulums so the scene still loads, steps, and renders headlessly. The
coupler/mimic behavior itself is a known dartpy-binding gap.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_LINK_SIZE = np.array([0.08, 0.08, 0.4])


def _pendulum(name: str, base_x: float, color: tuple, initial_angle: float) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    joint, body = skel.create_revolute_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = (base_x, 0.0, 0.4)
    joint.set_transform_from_parent_body_node(tf)
    joint.set_axis(np.array([0.0, 1.0, 0.0]))
    shape = dart.BoxShape(_LINK_SIZE)
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color(list(color))
    body.get_inertia().set_mass(0.5)
    body.get_inertia().set_moment(dart.BoxShape.compute_inertia_of(_LINK_SIZE, 0.5))
    skel.set_positions([initial_angle])
    return skel


def build() -> SceneSetup:
    world = dart.World("coupler_constraint")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_pendulum("driver", -0.25, (0.85, 0.30, 0.30), math.pi / 3.0))
    world.add_skeleton(_pendulum("follower", 0.25, (0.30, 0.55, 0.85), -math.pi / 6.0))
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="coupler_constraint",
    title="Coupler Constraint",
    category="Constraints & Joints",
    summary="Two revolute pendulums (coupler binding pending).",
    build=build,
)
