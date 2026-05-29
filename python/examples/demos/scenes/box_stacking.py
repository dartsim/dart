"""Box stacking scene: 5 boxes stacked over a floor.

Mirrors examples/demos/scenes/box_stacking.cpp. The C++ scene swaps the LCP
solver between Dantzig and PGS via panel — the Python mirror uses the
default solver. Box colors are deterministic (the C++ scene randomizes them).
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_BOX_SIZE = np.array([1.0, 1.0, 0.5])
_NUM_BOXES = 5


def _floor() -> "dart.Skeleton":
    skel = dart.Skeleton("floor")
    joint, body = skel.create_weld_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = (0.0, 0.0, -0.005)
    joint.set_transform_from_parent_body_node(tf)
    shape = dart.BoxShape(np.array([10.0, 10.0, 0.01]))
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color([0.84, 0.84, 0.84])
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    return skel


def _box(index: int, position: tuple) -> "dart.Skeleton":
    skel = dart.Skeleton(f"stack_box_{index}")
    _, body = skel.create_free_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = position
    body.get_parent_joint().set_transform_from_parent_body_node(tf)
    shape = dart.BoxShape(_BOX_SIZE)
    sn = body.create_shape_node(shape)
    hue = (index % 5) / 5.0
    sn.create_visual_aspect().set_color([0.30 + 0.6 * hue, 0.55, 0.85 - 0.5 * hue])
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    body.set_inertia(dart.Inertia(
        1.0,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(_BOX_SIZE, 1.0),
    ))
    return skel


def build() -> SceneSetup:
    world = dart.World("box_stacking")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_floor())
    for i in range(_NUM_BOXES):
        world.add_skeleton(_box(i, (0.0, 0.0, 0.5 + 0.25 + i * 0.5)))
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="box_stacking",
    title="Box Stacking",
    category="Constraints & Joints",
    summary="Five boxes stacked over a floor under gravity.",
    build=build,
)
