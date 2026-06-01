"""Hybrid dynamics: VELOCITY-actuated fullbody biped with PASSIVE root.

Mirrors examples/demos/scenes/hybrid_dynamics.cpp. Loads the
fullbody1 skeleton from dart://sample/skel/fullbody1.skel, sets the root
joint to PASSIVE and the rest to VELOCITY actuation, then drives
shoulder/forearm/shin joints with sinusoidal velocity commands.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, ScenePanel, SceneSetup
from ._z_up import reorient_to_z_up

_WORLD_URI = "dart://sample/skel/fullbody1.skel"
_BIPED_NAME = "fullbody1"
_DRIVEN_JOINTS = (
    "j_scapula_left",
    "j_scapula_right",
    "j_forearm_left",
    "j_forearm_right",
    "j_shin_left",
    "j_shin_right",
)


def _required_joint(skel: "dart.Skeleton", joint_name: str) -> "dart.Joint":
    joint = skel.get_joint(joint_name)
    if joint is None:
        raise RuntimeError(f"hybrid_dynamics world is missing joint: {joint_name}")
    return joint


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_WORLD_URI)
    if world is None:
        raise RuntimeError(f"Failed to load hybrid_dynamics world from {_WORLD_URI}")
    reorient_to_z_up(world)

    biped = world.get_skeleton(_BIPED_NAME)
    if biped is None:
        raise RuntimeError("hybrid_dynamics world is missing fullbody1")

    positions = np.array(biped.get_positions(), dtype=float)
    gen_coord_ids = [1, 6, 9, 10, 13, 16, 17, 21]
    init_config = [-0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0]
    for index, value in zip(gen_coord_ids, init_config):
        positions[index] = value
    biped.set_positions(positions)

    root = biped.get_joint(0)
    if root is not None:
        root.set_actuator_type(dart.PASSIVE)
    for i in range(1, biped.get_num_joints()):
        joint = biped.get_joint(i)
        if joint is not None:
            joint.set_actuator_type(dart.VELOCITY)

    driven = {name: _required_joint(biped, name) for name in _DRIVEN_JOINTS}
    state = {"arm_cycle": 0.0, "leg_cycle": 0.0}
    arm_history = deque(maxlen=120)
    leg_history = deque(maxlen=120)
    com_z_history = deque(maxlen=120)

    def pre_step() -> None:
        t = world.get_time()
        arm_cycle = math.sin(t * 4.0)
        leg_cycle = math.sin(t * 2.0)
        state["arm_cycle"] = arm_cycle
        state["leg_cycle"] = leg_cycle
        driven["j_scapula_left"].set_command(0, arm_cycle)
        driven["j_scapula_right"].set_command(0, -arm_cycle)
        driven["j_forearm_left"].set_command(0, 0.8 * arm_cycle)
        driven["j_forearm_right"].set_command(0, 0.8 * arm_cycle)
        driven["j_shin_left"].set_command(0, 0.1 * leg_cycle)
        driven["j_shin_right"].set_command(0, 0.1 * leg_cycle)

    def build_panel(builder: object, context: object) -> None:
        com = np.asarray(biped.get_com(), dtype=float)
        arm_history.append(float(state["arm_cycle"]))
        leg_history.append(float(state["leg_cycle"]))
        com_z_history.append(float(com[2]))
        builder.text("actuation: passive root + velocity joints")
        builder.text(f"arm velocity command: {state['arm_cycle']:.3f} rad/s")
        builder.text(f"leg velocity command: {0.1 * state['leg_cycle']:.3f} rad/s")
        builder.text(f"COM height: {com[2]:.3f} m")
        builder.separator()
        builder.plot_lines("Arm command", list(arm_history))
        builder.plot_lines("Leg command", list(leg_history))
        builder.plot_lines("COM height", list(com_z_history))

    return SceneSetup(
        world=world,
        pre_step=pre_step,
        panels=[ScenePanel("Hybrid Dynamics", build_panel)],
        info={"driven_joints": list(_DRIVEN_JOINTS)},
    )


SCENE = PythonDemoScene(
    id="hybrid_dynamics",
    title="Hybrid Dynamics",
    category="Control & IK",
    summary="Scripted velocity commands on a fullbody biped with PASSIVE root.",
    build=build,
)
