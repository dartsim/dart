"""Hardcoded design scene: a hand-built revolute chain.

Mirrors examples/demos/scenes/hardcoded_design.cpp. Builds a 5-link revolute
chain with box links and a small deterministic initial pose (the C++ scene
exposes interactive keyboard step controls; the Python mirror picks a fixed
sine pose).
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_LINK_SIZE = np.array([0.3, 0.3, 1.0])
_NUM_LINKS = 5


def build() -> SceneSetup:
    world = dart.World("hardcoded_design")
    world.set_gravity([0.0, 0.0, -9.81])

    skel = dart.Skeleton("visual_hardcoded_design")
    parent_body = None
    for i in range(_NUM_LINKS):
        joint, body = skel.create_revolute_joint_and_body_node_pair(parent_body)
        tf = np.eye(4)
        tf[2, 3] = _LINK_SIZE[2] if parent_body is None else _LINK_SIZE[2]
        joint.set_transform_from_parent_body_node(tf)
        joint.set_axis(np.array([0.0, 1.0, 0.0]))
        shape = dart.BoxShape(_LINK_SIZE)
        sn = body.create_shape_node(shape)
        sn.create_visual_aspect().set_color([0.30 + 0.12 * i, 0.55, 0.85 - 0.12 * i])
        body.set_inertia(dart.Inertia(
            1.0,
            np.zeros(3),
            dart.BoxShape.compute_inertia_of(_LINK_SIZE, 1.0),
        ))
        parent_body = body

    skel.set_positions([0.25 * math.sin(0.8 * i) for i in range(skel.get_num_dofs())])
    world.add_skeleton(skel)

    return SceneSetup(world=world, info={"golden_skeletons": ["visual_hardcoded_design"]})


SCENE = PythonDemoScene(
    id="hardcoded_design",
    title="Hardcoded Design",
    category="Constraints & Joints",
    summary="A hand-built revolute chain with deterministic initial pose.",
    build=build,
)
