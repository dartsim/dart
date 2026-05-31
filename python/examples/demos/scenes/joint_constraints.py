"""Joint-constraints scene: SPD-balanced biped with sagittal foot-strategy.

Mirrors examples/demos/scenes/joint_constraints.cpp at a high level. The
C++ scene exposes keyboard pushes (1-4) and a weld-joint harness toggle
(h); the headless Python mirror runs the same Stable PD controller +
ankle/back sagittal correction without the interactive perturbation
inputs. Uses the recently-added BodyNode.addExtForce / clearExternalForces
and Skeleton.getConstraintForces bindings.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, ScenePanel, SceneSetup
from ._z_up import reorient_to_z_up


_WORLD_URI = "dart://sample/skel/fullbody1.skel"
_BIPED_NAME = "fullbody1"


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_WORLD_URI)
    if world is None:
        raise RuntimeError(f"Failed to load joint_constraints world from {_WORLD_URI}")
    reorient_to_z_up(world)

    biped = world.get_skeleton(_BIPED_NAME)
    if biped is None:
        raise RuntimeError("joint_constraints world is missing fullbody1")

    positions = np.array(biped.get_positions(), dtype=float)
    indices = [1, 4, 6, 9, 10, 13, 16, 17, 21]
    initial = [-0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1]
    for i, v in zip(indices, initial):
        positions[i] = v
    biped.set_positions(positions)

    heel_left = biped.get_body_node("h_heel_left")
    if heel_left is None:
        raise RuntimeError("joint_constraints world is missing h_heel_left")

    dofs = biped.get_num_dofs()
    kp = np.zeros((dofs, dofs))
    kd = np.zeros((dofs, dofs))
    for i in range(6, min(22, dofs)):
        kp[i, i] = 200.0
        kd[i, i] = 100.0
    for i in range(22, dofs):
        kp[i, i] = 20.0
        kd[i, i] = 10.0
    desired = np.array(biped.get_positions(), dtype=float)

    prev_offset = [0.0]
    state = {"sagittal_offset": 0.0, "torque_norm": 0.0}
    pose_error_history = deque(maxlen=120)
    sagittal_offset_history = deque(maxlen=120)
    torque_history = deque(maxlen=120)

    def sagittal_offset() -> float:
        com = np.asarray(biped.get_com(), dtype=float)
        heel_transform = heel_left.get_transform()
        heel_tf = np.asarray(
            heel_transform.matrix()
            if hasattr(heel_transform, "matrix")
            else heel_transform,
            dtype=float,
        )
        cop = heel_tf[:3, :3] @ np.array([0.05, 0.0, 0.0]) + heel_tf[:3, 3]
        return float(com[0] - cop[0])

    def pre_step() -> None:
        dt = world.get_time_step()
        positions_now = np.asarray(biped.get_positions(), dtype=float)
        velocities = np.asarray(biped.get_velocities(), dtype=float)
        constraint_forces = np.asarray(biped.get_constraint_forces(), dtype=float)
        mass_matrix = np.asarray(biped.get_mass_matrix(), dtype=float)
        cg = np.asarray(biped.get_coriolis_and_gravity_forces(), dtype=float)

        proportional = -kp @ (positions_now + velocities * dt - desired)
        derivative = -kd @ velocities
        inv_mass = np.linalg.inv(mass_matrix + kd * dt)
        acceleration = inv_mass @ (-cg + proportional + derivative + constraint_forces)
        torques = proportional + derivative - kd @ acceleration * dt

        offset = sagittal_offset()
        if torques.size > 26 and offset < 0.1:
            ankle_hip_gain = 20.0
            back_gain = 10.0
            derivative_gain = 100.0
            correction = derivative_gain * (prev_offset[0] - offset)
            torques[17] += -ankle_hip_gain * offset + correction
            torques[25] += -back_gain * offset + correction
            torques[19] += -ankle_hip_gain * offset + correction
            torques[26] += -back_gain * offset + correction
            prev_offset[0] = offset

        torques[:6] = 0.0
        state["sagittal_offset"] = offset
        state["torque_norm"] = float(np.linalg.norm(torques))
        biped.set_forces(torques)

    def build_panel(builder: object, context: object) -> None:
        positions_now = np.asarray(biped.get_positions(), dtype=float)
        constraint_forces = np.asarray(biped.get_constraint_forces(), dtype=float)
        pose_error = float(np.linalg.norm(positions_now[6:] - desired[6:]))
        pose_error_history.append(pose_error)
        sagittal_offset_history.append(float(state["sagittal_offset"]))
        torque_history.append(float(state["torque_norm"]))
        builder.text("controller: SPD + sagittal correction")
        builder.text(f"pose error: {pose_error:.4f} rad")
        builder.text(f"sagittal COM-COP offset: {state['sagittal_offset']:.4f} m")
        builder.text(f"torque norm: {state['torque_norm']:.3f} N m")
        builder.text(f"constraint force norm: {np.linalg.norm(constraint_forces):.3f} N")
        builder.separator()
        builder.plot_lines("Pose error", list(pose_error_history))
        builder.plot_lines("Sagittal offset", list(sagittal_offset_history))
        builder.plot_lines("Torque norm", list(torque_history))

    return SceneSetup(
        world=world,
        pre_step=pre_step,
        panels=[ScenePanel("Joint Constraints", build_panel)],
        info={"controller": "stable_pd_sagittal_correction"},
    )


SCENE = PythonDemoScene(
    id="joint_constraints",
    title="Joint Constraints",
    category="Control & IK",
    summary="SPD-balanced fullbody biped with ankle/back sagittal correction.",
    build=build,
)
