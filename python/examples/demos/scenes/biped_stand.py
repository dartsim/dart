"""Biped standing scene: SPD controller holding a fullbody pose.

Mirrors examples/demos/scenes/biped_stand.cpp at a high level. The C++
scene supports keyboard-driven perturbation pushes via
``BodyNode.addExtForce`` and a constraint-force-aware Stable PD step;
those two C++ APIs are not bound in dartpy yet, so the Python mirror
runs a simplified Stable PD without the constraint-force term and
without perturbations. The resulting controller still keeps the biped
standing in the configured pose, which is the canonical demonstration
the scene exists to showcase.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup
from ._z_up import reorient_to_z_up

_WORLD_URI = "dart://sample/skel/fullbody1.skel"
_BIPED_NAME = "fullbody1"

_DOF_POSE = {
    "j_pelvis_rot_y": -0.20,
    "j_thigh_left_z": 0.15,
    "j_shin_left": -0.40,
    "j_heel_left_1": 0.25,
    "j_thigh_right_z": 0.15,
    "j_shin_right": -0.40,
    "j_heel_right_1": 0.25,
    "j_abdomen_2": 0.00,
}


def _find_dof(skel: "dart.Skeleton", name: str) -> "dart.DegreeOfFreedom":
    for i in range(skel.get_num_dofs()):
        dof = skel.get_dof(i)
        if dof.get_name() == name:
            return dof
    raise RuntimeError(f"biped_stand world is missing DOF: {name}")


def _set_required_dof_position(skel: "dart.Skeleton", name: str, value: float) -> None:
    _find_dof(skel, name).set_position(value)


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_WORLD_URI)
    if world is None:
        raise RuntimeError(f"Failed to load biped_stand world from {_WORLD_URI}")
    reorient_to_z_up(world)

    biped = world.get_skeleton(_BIPED_NAME)
    if biped is None:
        raise RuntimeError("biped_stand world is missing fullbody1")

    for name, value in _DOF_POSE.items():
        _set_required_dof_position(biped, name, value)

    dofs = biped.get_num_dofs()
    kp = np.zeros((dofs, dofs))
    kd = np.zeros((dofs, dofs))
    for i in range(6, dofs):
        kp[i, i] = 400.0
        kd[i, i] = 40.0
    desired = np.array(biped.get_positions(), dtype=float)

    def pre_step() -> None:
        dt = world.get_time_step()
        positions = np.asarray(biped.get_positions(), dtype=float)
        velocities = np.asarray(biped.get_velocities(), dtype=float)
        mass_matrix = np.asarray(biped.get_mass_matrix(), dtype=float)
        cg = np.asarray(biped.get_coriolis_and_gravity_forces(), dtype=float)

        proportional = -kp @ (positions + velocities * dt - desired)
        derivative = -kd @ velocities
        # Stable PD without the constraint-force term (binding gap).
        inv_mass = np.linalg.inv(mass_matrix + kd * dt)
        acceleration = inv_mass @ (-cg + proportional + derivative)
        torques = proportional + derivative - kd @ acceleration * dt
        torques[:6] = 0.0
        biped.set_forces(torques)

    return SceneSetup(world=world, pre_step=pre_step, info={})


SCENE = PythonDemoScene(
    id="biped_stand",
    title="Biped Stand",
    category="Control & IK",
    summary="Stable PD controller holding a fullbody biped upright.",
    build=build,
)
