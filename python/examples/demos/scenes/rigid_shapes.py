"""Rigid shapes scene: drops one of each primitive shape over a ground.

Mirrors examples/demos/scenes/rigid_shapes.cpp at a high level. The C++ scene
also adds visualized contact-point markers + visual nodes per shape; the
Python mirror keeps the shape variety (box, sphere, capsule, cylinder,
ellipsoid, cone, pyramid) and lets gravity settle them on the ground.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


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


def _add_shape_skel(world: "dart.World", name: str, shape: "dart.Shape",
                    position: tuple, color: tuple, mass: float = 1.0) -> None:
    skel = dart.Skeleton(name)
    _, body = skel.create_free_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = position
    body.get_parent_joint().set_transform_from_parent_body_node(tf)
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color(list(color))
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    body.set_inertia(dart.Inertia(
        mass,
        np.zeros(3),
        shape.compute_inertia(mass),
    ))
    world.add_skeleton(skel)


def build() -> SceneSetup:
    world = dart.World("rigid_shapes")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_ground())

    _add_shape_skel(world, "box",       dart.BoxShape(np.array([0.25, 0.25, 0.25])),     (-1.2, 0.0, 0.6), (0.90, 0.45, 0.20))
    _add_shape_skel(world, "sphere",    dart.SphereShape(0.15),                          (-0.6, 0.0, 0.6), (0.20, 0.55, 0.90))
    _add_shape_skel(world, "capsule",   dart.CapsuleShape(0.12, 0.30),                   ( 0.0, 0.0, 0.6), (0.40, 0.80, 0.35))
    _add_shape_skel(world, "cylinder",  dart.CylinderShape(0.13, 0.30),                  ( 0.6, 0.0, 0.6), (0.80, 0.30, 0.70))
    _add_shape_skel(world, "ellipsoid", dart.EllipsoidShape(np.array([0.20, 0.30, 0.18])), ( 1.2, 0.0, 0.6), (0.70, 0.55, 0.20))
    _add_shape_skel(world, "cone",      dart.ConeShape(0.16, 0.36),                      (-0.6, 0.6, 0.6), (0.10, 0.30, 0.85))
    _add_shape_skel(world, "pyramid",   dart.PyramidShape(0.30, 0.20, 0.30),             ( 0.6, 0.6, 0.6), (0.90, 0.20, 0.30))

    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="rigid_shapes",
    title="Rigid Shapes",
    category="Rigid Body",
    summary="One of each primitive shape, dropped on the ground.",
    build=build,
)
