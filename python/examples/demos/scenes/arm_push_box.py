"""Contact-rich manipulation (minimal): a 1-link arm swings into a free box.

A short rigid "arm" is swung by a constant torque around a fixed pivot; on the
way down it strikes a free-floating box resting on the ground. Demonstrates the
contact-rich scenario surface: a controlled actuator interacting with rigid
objects through contact. Replace the constant torque with a richer controller
(impedance, MPC, IK) without changing the scene shape.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, ScenePanel, SceneSetup

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
    body.set_inertia(dart.Inertia(
        1.0,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(np.array([0.05, 0.05, 0.6]), 1.0),
    ))
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
    body.set_inertia(dart.Inertia(
        0.6,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(np.array([0.18, 0.18, 0.18]), 0.6),
    ))
    return skel


def _make_ground() -> dart.Skeleton:
    skel = dart.Skeleton("ground")
    joint, body = skel.create_weld_joint_and_body_node_pair()
    transform = np.eye(4)
    transform[:3, 3] = (0.0, 0.0, -0.05)
    joint.set_transform_from_parent_body_node(transform)
    visual = dart.BoxShape(np.array([10.0, 10.0, 0.1]))
    visual_node = body.create_shape_node(visual)
    visual_node.create_visual_aspect().set_color([0.8, 0.8, 0.8])
    collision_node = body.create_shape_node(dart.PlaneShape(np.array([0.0, 0.0, 1.0]), 0.0))
    collision_node.create_collision_aspect()
    collision_node.create_dynamics_aspect()
    return skel


def build() -> SceneSetup:
    world = dart.World("arm_push_box")
    world.set_gravity([0.0, 0.0, -9.81])
    arm = _make_arm()
    box = _make_box()
    world.add_skeleton(arm)
    world.add_skeleton(box)
    world.add_skeleton(_make_ground())
    arm_joint = arm.get_joint(0)
    last_force = {"value": 0.0}
    arm_angle_history = deque(maxlen=120)
    box_x_history = deque(maxlen=120)

    def pre_step() -> None:
        last_force["value"] = -_ARM_TORQUE
        arm_joint.set_force(0, last_force["value"])  # swing downward

    def build_panel(builder: object, context: object) -> None:
        arm_angle = float(arm_joint.get_position(0))
        arm_velocity = float(arm_joint.get_velocity(0))
        box_com = np.asarray(box.get_com(), dtype=float)
        arm_angle_history.append(arm_angle)
        box_x_history.append(float(box_com[0]))
        builder.text("controller: constant joint torque")
        builder.text(f"arm angle: {arm_angle:.3f} rad")
        builder.text(f"arm velocity: {arm_velocity:.3f} rad/s")
        builder.text(f"command torque: {last_force['value']:.3f} N m")
        builder.text(
            f"box COM: {box_com[0]:.3f}, {box_com[1]:.3f}, {box_com[2]:.3f} m"
        )
        builder.separator()
        builder.plot_lines("Arm angle", list(arm_angle_history))
        builder.plot_lines("Box x", list(box_x_history))

    return SceneSetup(
        world=world,
        pre_step=pre_step,
        panels=[ScenePanel("Arm Push Box", build_panel)],
        info={"controller": "constant_torque"},
    )


SCENE = PythonDemoScene(
    id="arm_push_box",
    title="Arm Pushes Box",
    category="Control & Modern",
    summary="A 1-link arm swings into a free box (contact-rich manipulation).",
    build=build,
)
