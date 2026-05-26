"""Contact-rich manipulation (minimal): a 1-link arm swings into a free box.

A short rigid "arm" is swung by a constant torque around a fixed pivot; on the
way down it strikes a free-floating box resting on the ground. Demonstrates the
contact-rich scenario surface: a controlled actuator interacting with rigid
objects through contact. Replace the constant torque with a richer controller
(impedance, MPC, IK) without changing the scene shape.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_ARM_TORQUE = 4.0
_ARM_INITIAL_ANGLE = 1.2  # radians, arm raised


def _make_arm() -> dart.Skeleton:
    skel = dart.Skeleton("arm")
    joint, body = skel.create_revolute_joint_and_body_node_pair()
    joint.set_axis(np.array([0.0, 1.0, 0.0]))
    joint.set_position(0, _ARM_INITIAL_ANGLE)
    shape = dart.BoxShape(np.array([0.05, 0.05, 0.6]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect().set_color([0.40, 0.65, 0.30])
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()
    body.get_inertia().set_mass(1.0)
    body.get_inertia().set_moment(
        dart.BoxShape.compute_inertia_of(np.array([0.05, 0.05, 0.6]), 1.0))
    return skel


def _make_box() -> dart.Skeleton:
    skel = dart.Skeleton("target_box")
    joint, body = skel.create_free_joint_and_body_node_pair()
    transform = np.eye(4)
    transform[:3, 3] = (0.5, 0.0, 0.1)
    joint.set_transform_from_parent_body_node(transform)
    shape = dart.BoxShape(np.array([0.18, 0.18, 0.18]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect().set_color([0.90, 0.50, 0.20])
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()
    body.get_inertia().set_mass(0.6)
    body.get_inertia().set_moment(
        dart.BoxShape.compute_inertia_of(np.array([0.18, 0.18, 0.18]), 0.6))
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
    world = dart.World("arm_push_box")
    world.set_gravity([0.0, 0.0, -9.81])
    arm = _make_arm()
    world.add_skeleton(arm)
    world.add_skeleton(_make_box())
    world.add_skeleton(_make_ground())
    arm_joint = arm.get_joint(0)

    def step(n: int) -> None:
        for _ in range(max(0, n)):
            arm_joint.set_force(0, -_ARM_TORQUE)  # swing downward
            world.step()

    return SceneSetup(world=world, step=step,
                      info={"controller": "constant_torque"})


SCENE = PythonDemoScene(
    id="arm_push_box",
    title="Arm Pushes Box",
    category="Control & Modern",
    summary="A 1-link arm swings into a free box (contact-rich manipulation).",
    build=build,
)
