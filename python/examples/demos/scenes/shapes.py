"""Shapes scene: a box, a sphere, and another box settling on the ground.

Mirrors examples/demos/scenes/shapes.cpp.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def _make_ground() -> "dart.Skeleton":
    skel = dart.Skeleton("ground")
    joint, body = skel.create_weld_joint_and_body_node_pair()
    transform = np.eye(4)
    transform[:3, 3] = (0.0, 0.0, -0.05)
    joint.set_transform_from_parent_body_node(transform)
    shape = dart.BoxShape(np.array([10.0, 10.0, 0.1]))
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color([0.8, 0.8, 0.8])
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    return skel


def _make_box(
    name: str, position: tuple[float, float, float], color: tuple[float, float, float]
) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    _, body = skel.create_free_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = position
    body.get_parent_joint().set_transform_from_parent_body_node(tf)

    size = np.array([0.24, 0.24, 0.24])
    shape = dart.BoxShape(size)
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color(list(color))
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    body.set_inertia(dart.Inertia(
        1.0,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(size, 1.0),
    ))
    return skel


def _make_sphere(
    name: str, position: tuple[float, float, float], color: tuple[float, float, float]
) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    _, body = skel.create_free_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = position
    body.get_parent_joint().set_transform_from_parent_body_node(tf)

    shape = dart.SphereShape(0.14)
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color(list(color))
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    body.set_inertia(dart.Inertia(
        1.0,
        np.zeros(3),
        dart.SphereShape.compute_inertia_of(0.14, 1.0),
    ))
    return skel


def build() -> SceneSetup:
    world = dart.World("shapes")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_make_ground())
    world.add_skeleton(_make_box("box_a", (-0.5, 0.0, 0.4), (0.90, 0.45, 0.20)))
    world.add_skeleton(_make_sphere("sphere_a", (0.0, 0.0, 0.5), (0.20, 0.55, 0.90)))
    world.add_skeleton(_make_box("box_b", (0.5, 0.0, 0.6), (0.40, 0.80, 0.35)))
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="shapes",
    title="Shapes",
    category="Visualization",
    summary="Assorted primitive shapes resting on the ground.",
    build=build,
)
