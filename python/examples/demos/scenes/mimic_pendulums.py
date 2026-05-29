"""Mimic pendulums (simplified): paired revolute pendulums under gravity.

Mirrors examples/demos/scenes/mimic_pendulums.cpp at a high level. The C++
scene wires a `MimicJoint` controller so one pendulum mirrors another's
motion; that feature isn't bound in dartpy today, so the Python mirror builds
two independent two-link revolute pendulums side-by-side. The visual layout
matches; the mimic behavior itself is a known dartpy-binding gap (see
docs/dev_tasks/examples_strategy/RESUME.md).
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_LINK_SIZE = np.array([0.08, 0.08, 0.4])


def _pendulum(name: str, base_x: float, color: tuple) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    parent = None
    for i in range(2):
        joint, body = skel.create_revolute_joint_and_body_node_pair(parent)
        tf = np.eye(4)
        tf[2, 3] = _LINK_SIZE[2] if parent is not None else 0.4
        if parent is None:
            tf[0, 3] = base_x
        joint.set_transform_from_parent_body_node(tf)
        joint.set_axis(np.array([0.0, 1.0, 0.0]))
        shape = dart.BoxShape(_LINK_SIZE)
        sn = body.create_shape_node(shape)
        sn.create_visual_aspect().set_color([color[0], color[1] + 0.1 * i, color[2]])
        body.get_inertia().set_mass(0.5)
        body.get_inertia().set_moment(dart.BoxShape.compute_inertia_of(_LINK_SIZE, 0.5))
        parent = body
    skel.set_positions([math.pi / 3.0, 0.0])
    return skel


def build() -> SceneSetup:
    world = dart.World("mimic_pendulums")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_pendulum("mimic_left", -0.3, (0.85, 0.30, 0.30)))
    world.add_skeleton(_pendulum("mimic_right", 0.3, (0.30, 0.55, 0.85)))
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="mimic_pendulums",
    title="Mimic Pendulums",
    category="Constraints & Joints",
    summary="Two paired revolute pendulums (mimic-joint binding pending).",
    build=build,
)
