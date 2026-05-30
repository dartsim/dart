"""Rigid loop scene: a chain closed into a loop with a ball-joint constraint.

Mirrors examples/demos/scenes/rigid_loop.cpp. Loads chain.skel, applies a
deterministic initial pose (4 specific DOFs bent to 0.4·pi), and connects
link 6 ↔ link 10 via a BallJointConstraint at link 6's local offset
(0, 0.025, 0). A custom step callable applies the same damping the C++ scene
applies in preStep.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup
from ._z_up import reorient_to_z_up

_URI = "dart://sample/skel/chain.skel"
_LOOP_NAME = "rigid_loop"


def _make_pre_step(chain: "dart.Skeleton") -> "callable":
    def pre_step() -> None:
        vel = np.asarray(chain.get_velocities())
        damping = -0.01 * vel
        for i in range(damping.shape[0]):
            if i % 3 == 1:
                damping[i] *= 0.1
        chain.set_forces(damping.tolist())
    return pre_step


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_URI)
    if world is None:
        raise RuntimeError(f"Failed to load {_URI}")
    world.set_time_step(1.0 / 2000.0)

    chain = world.get_skeleton(0)
    if chain is None:
        raise RuntimeError("Rigid loop world did not contain a skeleton")
    chain.set_name(_LOOP_NAME)

    num_dofs = chain.get_num_dofs()
    pose = [0.0] * num_dofs
    for index in (20, 23, 26, 29):
        if index < num_dofs:
            pose[index] = 0.4 * math.pi
    chain.set_positions(pose)

    # chain.skel is authored Y-up; reorient to Z-up before deriving the loop
    # constraint anchor from link 6's (now Z-up) world transform.
    reorient_to_z_up(world)

    link6 = chain.get_body_node("link 6")
    link10 = chain.get_body_node("link 10")
    if link6 is None or link10 is None:
        raise RuntimeError("Rigid loop world is missing link 6 or link 10")

    link6_tf = np.asarray(link6.get_world_transform().matrix())
    joint_position = link6_tf[:3, :3] @ np.array([0.0, 0.025, 0.0]) + link6_tf[:3, 3]
    constraint = dart.constraint.BallJointConstraint(link6, link10, joint_position)
    world.get_constraint_solver().add_constraint(constraint)

    setup = SceneSetup(world=world, info={"golden_skeletons": [_LOOP_NAME]})
    setup.pre_step = _make_pre_step(chain)
    return setup


SCENE = PythonDemoScene(
    id="rigid_loop",
    title="Rigid Loop",
    category="Constraints & Joints",
    summary="A chain closed into a loop with a ball-joint constraint.",
    build=build,
)
