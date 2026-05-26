"""Boxes scene: a small grid of rigid boxes dropped onto a ground.

Mirrors the C++ `examples/demos/scenes/boxes.cpp` scene (3x3x3 grid, same
shape size and spacing, deterministic positions) for golden-set parity.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_GRID_SIZE = 3
_BOX_EXTENT = 0.18
_SPACING = 0.32


def _make_box(index: int, position: tuple[float, float, float]) -> dart.Skeleton:
    skel = dart.Skeleton(f"box_{index}")
    joint, body = skel.create_free_joint_and_body_node_pair()
    transform = np.eye(4)
    transform[:3, 3] = position
    joint.set_transform_from_parent_body_node(transform)

    shape = dart.BoxShape(np.array([_BOX_EXTENT, _BOX_EXTENT, _BOX_EXTENT]))
    shape_node = body.create_shape_node(shape)
    hue = (index % 6) / 6.0
    shape_node.create_visual_aspect().set_color(
        [0.35 + 0.5 * hue, 0.45, 0.9 - 0.4 * hue])
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()
    body.get_inertia().set_mass(1.0)
    body.get_inertia().set_moment(dart.BoxShape.compute_inertia_of(
        np.array([_BOX_EXTENT, _BOX_EXTENT, _BOX_EXTENT]), 1.0))
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
    world = dart.World("boxes")
    world.set_gravity([0.0, 0.0, -9.81])

    index = 0
    origin = -0.5 * (_GRID_SIZE - 1) * _SPACING
    box_names: list[str] = []
    for x in range(_GRID_SIZE):
        for y in range(_GRID_SIZE):
            for z in range(_GRID_SIZE):
                position = (
                    origin + x * _SPACING,
                    origin + y * _SPACING,
                    0.4 + z * _SPACING,
                )
                skel = _make_box(index, position)
                world.add_skeleton(skel)
                box_names.append(skel.get_name())
                index += 1
    world.add_skeleton(_make_ground())

    return SceneSetup(world=world, info={"golden_skeletons": box_names})


SCENE = PythonDemoScene(
    id="boxes",
    title="Boxes",
    category="Rigid Body",
    summary="A 3x3x3 grid of rigid boxes drops onto the ground.",
    build=build,
)
