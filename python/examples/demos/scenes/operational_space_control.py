"""Operational-space control scene: KR5 arm tracks a fixed target.

Mirrors `examples/demos/scenes/operational_space_control.cpp` so the
golden-set parity smoke (PLAN-103 Phase 2) can assert both languages
produce the same simulated state. The target frame is held at the
deterministic offset (0.05, 0, 0) from the end-effector's initial world
transform; the C++ scene uses the same fixed target during the parity
window before the user starts dragging it.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_ROBOT_URI = "dart://sample/urdf/KR5/KR5 sixx R650.urdf"
_GROUND_URI = "dart://sample/urdf/KR5/ground.urdf"
_ROBOT_NAME = "KR5"
_TARGET_FRAME_NAME = "operational_space_control_target"
_GROUND_NAME = "visual_operational_space_control_ground"


def _load_robot(urdf: "dart.io.UrdfParser") -> "dart.Skeleton":
    robot = urdf.parse_skeleton(_ROBOT_URI)
    if robot is None:
        raise RuntimeError(f"Failed to load robot {_ROBOT_URI}")
    if robot.get_num_joints() == 0:
        raise RuntimeError("Operational-space KR5 is missing its root joint")
    robot.get_joint(0).set_transform_from_parent_body_node(np.eye(4))
    robot.set_name(_ROBOT_NAME)

    for i in range(robot.get_num_joints()):
        joint = robot.get_joint(i)
        joint.set_limit_enforcement(False)
        if joint.get_num_dofs() > 0:
            joint.set_damping_coefficient(0, 0.5)
    return robot


def _load_ground(urdf: "dart.io.UrdfParser") -> "dart.Skeleton":
    ground = urdf.parse_skeleton(_GROUND_URI)
    if ground is None:
        raise RuntimeError(f"Failed to load ground {_GROUND_URI}")
    ground.set_name(_GROUND_NAME)

    joint = ground.get_joint(0)
    # Match the C++ scene: pretranslate(0, 0, 0.5) then rotate RotX(pi/2). In
    # Eigen, `Isometry3d::pretranslate(t)` multiplies the translation on the
    # LEFT (translate * base) and `rotate(R)` multiplies the rotation on the
    # RIGHT (transform * R), so reproduce that ordering on 4x4 numpy matrices.
    base = np.asarray(joint.get_transform_from_parent_body_node().matrix())
    translate = np.eye(4)
    translate[:3, 3] = (0.0, 0.0, 0.5)
    rot = np.eye(4)
    c, s = math.cos(math.pi / 2.0), math.sin(math.pi / 2.0)
    rot[1, 1] = c
    rot[1, 2] = -s
    rot[2, 1] = s
    rot[2, 2] = c
    transform = translate @ base @ rot
    joint.set_transform_from_parent_body_node(transform)
    return ground


def _make_step(setup: "SceneSetup") -> "callable":
    world = setup.world
    robot = world.get_skeleton(_ROBOT_NAME)
    end_effector = robot.get_body_node(robot.get_num_body_nodes() - 1)
    target = world.get_simple_frame(_TARGET_FRAME_NAME)

    kp = np.eye(3) * 50.0
    num_dependent = end_effector.get_num_dependent_gen_coords()
    kd = np.eye(num_dependent) * 5.0
    # mOffset = R(ee_world).T @ (0.05, 0, 0). At t=0, R is the rotation of the
    # end-effector's initial world transform.
    ee_world = np.asarray(end_effector.get_world_transform().matrix())
    raw_offset = np.array([0.05, 0.0, 0.0])
    offset = ee_world[:3, :3].T @ raw_offset
    reg = 0.0025 * np.eye(3)

    def step(frames: int) -> None:
        for _ in range(max(0, frames)):
            mass = robot.get_mass_matrix()
            # NOTE: jacobian / velocity calls use keyword args because
            # nanobind otherwise matches the `Frame*` overload of these names
            # and reinterprets the numpy 3-vector as a pointer (segfault).
            jac = end_effector.get_linear_jacobian(offset=offset)
            jac_inv = jac.T @ np.linalg.inv(jac @ jac.T + reg)
            jac_d = end_effector.get_linear_jacobian_deriv(offset=offset)
            jac_d_inv = jac_d.T @ np.linalg.inv(jac_d @ jac_d.T + reg)
            target_world = np.asarray(target.get_world_transform().matrix())
            ee_now = np.asarray(end_effector.get_world_transform().matrix())
            ee_offset_world = ee_now[:3, :3] @ offset + ee_now[:3, 3]
            pos_err = target_world[:3, 3] - ee_offset_world
            vel_err = -end_effector.get_linear_velocity(offset=offset)
            cg = robot.get_coriolis_and_gravity_forces()
            forces = (
                mass @ (jac_inv @ (kp @ vel_err) + jac_d_inv @ (kp @ pos_err))
                + cg
                + kd @ jac_inv @ (kp @ pos_err)
            )
            robot.set_forces(forces)
            world.step()

    return step


def build() -> SceneSetup:
    world = dart.World("dartsim_operational_space")
    world.set_gravity([0.0, 0.0, -9.81])

    urdf = dart.io.UrdfParser()
    ground = _load_ground(urdf)
    world.add_skeleton(ground)

    robot = _load_robot(urdf)
    end_effector = robot.get_body_node(robot.get_num_body_nodes() - 1)
    ee_world = np.asarray(end_effector.get_world_transform().matrix())
    target_transform = ee_world.copy()
    target_transform[:3, 3] = ee_world[:3, 3] + np.array([0.05, 0.0, 0.0])
    target = dart.SimpleFrame(
        dart.Frame.world(), _TARGET_FRAME_NAME, target_transform)
    world.add_simple_frame(target)

    world.add_skeleton(robot)

    setup = SceneSetup(
        world=world, info={"golden_skeletons": [_ROBOT_NAME]})
    setup.step = _make_step(setup)
    return setup


SCENE = PythonDemoScene(
    id="operational_space_control",
    title="Operational Space Control",
    category="Control & IK",
    summary="A KR5 arm tracks a fixed target via operational-space control.",
    build=build,
)
