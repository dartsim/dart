"""Capsule ground contact: a capsule settles on a plane under gravity.

Mirrors examples/demos/scenes/capsule_ground_contact.cpp. Uses PlaneShape
(now bound) for the ground collision surface.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_CAPSULE_RADIUS = 0.2
_CAPSULE_HEIGHT = 0.6


def _horizontal_pose() -> np.ndarray:
    tf = np.eye(4)
    tf[:3, 3] = (0.0, 0.0, _CAPSULE_RADIUS + 0.12)
    c, s = math.cos(math.pi / 2.0), math.sin(math.pi / 2.0)
    R = np.eye(3)
    R[0, 0] = c
    R[0, 2] = s
    R[2, 0] = -s
    R[2, 2] = c
    tf[:3, :3] = R
    return tf


def _make_ground() -> "dart.Skeleton":
    skel = dart.Skeleton("ground")
    _, body = skel.create_weld_joint_and_body_node_pair()

    plane = dart.PlaneShape(np.array([0.0, 0.0, 1.0]), 0.0)
    sn_collision = body.create_shape_node(plane)
    sn_collision.create_collision_aspect()
    sn_collision.create_dynamics_aspect()

    visual = dart.BoxShape(np.array([4.0, 4.0, 0.08]))
    sn_visual = body.create_shape_node(visual)
    sn_visual.create_visual_aspect().set_color([0.70, 0.70, 0.70])

    # set_mobile(False) isn't bound; the WeldJoint already pins the ground.
    return skel


def _make_capsule() -> "dart.Skeleton":
    skel = dart.Skeleton("capsule")
    joint, body = skel.create_free_joint_and_body_node_pair()
    joint.set_transform_from_parent_body_node(_horizontal_pose())

    shape = dart.CapsuleShape(_CAPSULE_RADIUS, _CAPSULE_HEIGHT)
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color([0.2, 0.4, 0.8])
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()

    body.get_inertia().set_mass(1.0)
    body.get_inertia().set_moment(shape.compute_inertia(1.0))
    return skel


def build() -> SceneSetup:
    world = dart.World("capsule_ground_contact")
    world.set_time_step(0.001)
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_make_ground())
    world.add_skeleton(_make_capsule())
    return SceneSetup(world=world, info={"golden_skeletons": ["capsule"]})


SCENE = PythonDemoScene(
    id="capsule_ground_contact",
    title="Capsule Ground Contact",
    category="Collision",
    summary="A capsule settles on a plane under gravity.",
    build=build,
)
