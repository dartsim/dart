"""Legged whole-body balance scene (minimal): inverted pendulum stabilization.

A 1-DoF inverted pendulum is held upright by a PD torque controller on its base
revolute joint. This is the minimal "legged whole-body control" exemplar — a
single articulated link balancing under closed-loop control — that proves the
scene surface scales to control-loop scenarios. Full multi-link WBC controllers
(quadruped/humanoid) plug into the same pattern.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

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
    body.get_inertia().set_mass(1.0)
    body.get_inertia().set_moment(
        dart.BoxShape.compute_inertia_of(np.array([0.05, 0.05, 1.0]), 1.0))
    return skel


def build() -> SceneSetup:
    world = dart.World("legged_balance")
    world.set_gravity([0.0, 0.0, -9.81])
    pendulum = _make_pendulum()
    world.add_skeleton(pendulum)
    joint = pendulum.get_joint(0)

    def step(n: int) -> None:
        for _ in range(max(0, n)):
            q = joint.get_position(0)
            qdot = joint.get_velocity(0)
            tau = -_KP * (q - _DESIRED_ANGLE) - _KD * qdot
            joint.set_force(0, tau)
            world.step()

    return SceneSetup(world=world, step=step,
                      info={"controller": "PD", "kp": _KP, "kd": _KD})


SCENE = PythonDemoScene(
    id="legged_balance",
    title="Legged Balance (PD)",
    category="Control & Modern",
    summary="Minimal inverted-pendulum balance under PD torque control.",
    build=build,
)
