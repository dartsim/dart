"""Legged whole-body balance scene (minimal): inverted pendulum stabilization.

A 1-DoF inverted pendulum is held upright by a PD torque controller on its base
revolute joint. This is the minimal "legged whole-body control" exemplar — a
single articulated link balancing under closed-loop control — that proves the
scene surface scales to control-loop scenarios. Full multi-link WBC controllers
(quadruped/humanoid) plug into the same pattern.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_KP = 80.0
_KD = 10.0
_DESIRED_ANGLE = 0.0  # upright


def _make_pendulum() -> dart.Skeleton:
    skel = dart.Skeleton("pendulum")
    joint, body = skel.create_revolute_joint_and_body_node_pair()
    joint.set_axis(np.array([0.0, 1.0, 0.0]))
    joint.set_position(0, 0.05)  # small initial tilt to drive the controller
    shape = dart.BoxShape(np.array([0.05, 0.05, 1.0]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect().set_color([0.85, 0.45, 0.25])
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()
    body.set_inertia(dart.Inertia(
        1.0,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(np.array([0.05, 0.05, 1.0]), 1.0),
    ))
    return skel


def build() -> SceneSetup:
    world = dart.World("legged_balance")
    world.set_gravity([0.0, 0.0, -9.81])
    pendulum = _make_pendulum()
    world.add_skeleton(pendulum)
    joint = pendulum.get_joint(0)
    last_tau = {"value": 0.0}
    angle_history = deque(maxlen=120)
    torque_history = deque(maxlen=120)

    def pre_step() -> None:
        q = joint.get_position(0)
        qdot = joint.get_velocity(0)
        tau = -_KP * (q - _DESIRED_ANGLE) - _KD * qdot
        last_tau["value"] = tau
        joint.set_force(0, tau)

    def build_panel(builder: object, context: object) -> None:
        q = float(joint.get_position(0))
        qdot = float(joint.get_velocity(0))
        angle_history.append(q)
        torque_history.append(float(last_tau["value"]))
        builder.text("controller: PD balance")
        builder.text(f"target angle: {_DESIRED_ANGLE:.3f} rad")
        builder.text(f"angle: {q:.3f} rad")
        builder.text(f"angular velocity: {qdot:.3f} rad/s")
        builder.text(f"torque: {last_tau['value']:.3f} N m")
        builder.separator()
        builder.plot_lines("Angle", list(angle_history))
        builder.plot_lines("Torque", list(torque_history))

    return SceneSetup(
        world=world,
        pre_step=pre_step,
        panels=[ScenePanel("Legged Balance", build_panel)],
        info={"controller": "PD", "kp": _KP, "kd": _KD},
    )


SCENE = PythonDemoScene(
    id="legged_balance",
    title="Legged Balance (PD)",
    category="Control & Modern",
    summary="Minimal inverted-pendulum balance under PD torque control.",
    build=build,
)
