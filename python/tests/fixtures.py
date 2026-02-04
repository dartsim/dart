"""
Test fixtures for creating skeletons and objects in Python tests.

Port of tests/helpers/dynamics_helpers.hpp patterns for Python tests.
Uses the snake_case dartpy API.
"""

from __future__ import annotations

import math
from enum import IntEnum

import numpy as np

import dartpy as dart


class TypeOfDOF(IntEnum):
    DOF_X = 0
    DOF_Y = 1
    DOF_Z = 2
    DOF_ROLL = 3
    DOF_PITCH = 4
    DOF_YAW = 5


def _get_axis(dof_type: TypeOfDOF) -> np.ndarray:
    """Get the axis vector for a given DOF type."""
    if dof_type == TypeOfDOF.DOF_X:
        return np.array([1.0, 0.0, 0.0])
    elif dof_type == TypeOfDOF.DOF_Y:
        return np.array([0.0, 1.0, 0.0])
    elif dof_type in (TypeOfDOF.DOF_Z, TypeOfDOF.DOF_YAW):
        return np.array([0.0, 0.0, 1.0])
    elif dof_type == TypeOfDOF.DOF_PITCH:
        return np.array([0.0, 1.0, 0.0])
    else:  # DOF_ROLL
        return np.array([1.0, 0.0, 0.0])


def add_end_effector(
    robot: dart.Skeleton,
    parent_node: dart.BodyNode,
    dim: np.ndarray,
) -> dart.BodyNode:
    """Add an end effector to the robot."""
    [joint, body] = robot.create_weld_joint_and_body_node_pair(parent_node)
    joint.set_name("eeJoint")

    # Set transform from parent
    T = np.eye(4)
    T[2, 3] = dim[2]
    joint.set_transform_from_parent_body_node(T)

    # Add shape
    shape = dart.BoxShape(np.array([0.2, 0.2, 0.2]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect()
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()

    return body


def add_1dof_joint(
    skel: dart.Skeleton,
    parent: dart.BodyNode | None,
    name: str,
    val: float,
    min_val: float,
    max_val: float,
    dof_type: TypeOfDOF,
) -> tuple:
    """Add a 1-DOF joint (prismatic or revolute) to the skeleton."""
    if dof_type in (TypeOfDOF.DOF_X, TypeOfDOF.DOF_Y, TypeOfDOF.DOF_Z):
        [joint, body] = skel.create_prismatic_joint_and_body_node_pair(parent)
        joint.set_axis(_get_axis(dof_type))
    else:
        [joint, body] = skel.create_revolute_joint_and_body_node_pair(parent)
        joint.set_axis(_get_axis(dof_type))

    joint.set_name(name)
    dof = joint.get_dof(0)
    dof.set_position_lower_limit(min_val)
    dof.set_position_upper_limit(max_val)
    joint.set_position(0, val)

    return (joint, body)


def create_three_link_robot(
    dim1: np.ndarray,
    type1: TypeOfDOF,
    dim2: np.ndarray,
    type2: TypeOfDOF,
    dim3: np.ndarray,
    type3: TypeOfDOF,
    finished: bool = False,
    collision_shape: bool = True,
    stop_after: int = 3,
) -> dart.Skeleton:
    """Create a 3-link robot with configurable DOF types."""
    robot = dart.Skeleton()
    dim_ee = dim1

    # Link 1
    joint1, body1 = add_1dof_joint(robot, None, "joint1", 0.0, -math.pi, math.pi, type1)
    body1.get_inertia().set_local_com(np.array([0.0, 0.0, dim1[2] / 2.0]))

    shape1 = dart.BoxShape(dim1)
    shape_node1 = body1.create_shape_node(shape1)
    shape_node1.create_visual_aspect()
    if collision_shape:
        shape_node1.create_collision_aspect()
        shape_node1.create_dynamics_aspect()

    parent_node = body1

    if stop_after > 1:
        # Link 2
        joint2, body2 = add_1dof_joint(
            robot, parent_node, "joint2", 0.0, -math.pi, math.pi, type2
        )
        body2.get_inertia().set_local_com(np.array([0.0, 0.0, dim2[2] / 2.0]))

        T = np.eye(4)
        T[2, 3] = dim1[2]
        joint2.set_transform_from_parent_body_node(T)

        shape2 = dart.BoxShape(dim2)
        shape_node2 = body2.create_shape_node(shape2)
        shape_node2.create_visual_aspect()
        if collision_shape:
            shape_node2.create_collision_aspect()
            shape_node2.create_dynamics_aspect()

        parent_node = body2
        dim_ee = dim2

    if stop_after > 2:
        # Link 3
        joint3, body3 = add_1dof_joint(
            robot, parent_node, "joint3", 0.0, -math.pi, math.pi, type3
        )
        body3.get_inertia().set_local_com(np.array([0.0, 0.0, dim3[2] / 2.0]))

        T = np.eye(4)
        T[2, 3] = dim2[2]
        joint3.set_transform_from_parent_body_node(T)

        shape3 = dart.BoxShape(dim3)
        shape_node3 = body3.create_shape_node(shape3)
        shape_node3.create_visual_aspect()
        if collision_shape:
            shape_node3.create_collision_aspect()
            shape_node3.create_dynamics_aspect()

        parent_node = body3
        dim_ee = dim3

    if finished:
        add_end_effector(robot, parent_node, dim_ee)

    return robot


def create_two_link_robot(
    dim1: np.ndarray,
    type1: TypeOfDOF,
    dim2: np.ndarray,
    type2: TypeOfDOF,
    finished: bool = True,
) -> dart.Skeleton:
    """Create a 2-link robot."""
    return create_three_link_robot(
        dim1, type1, dim2, type2, np.zeros(3), TypeOfDOF.DOF_X, finished, True, 2
    )


def create_n_link_robot(
    n: int,
    dim: np.ndarray,
    dof_type: TypeOfDOF,
    finished: bool = False,
) -> dart.Skeleton:
    """Create an n-link robot chain."""
    assert n > 0

    robot = dart.Skeleton()
    robot.disable_self_collision_check()

    # First link
    joint1, body1 = add_1dof_joint(
        robot, None, "joint1", 0.0, -math.pi, math.pi, dof_type
    )
    body1.get_inertia().set_local_com(np.array([0.0, 0.0, dim[2] / 2.0]))
    joint1.get_dof(0).set_damping_coefficient(0.01)

    shape1 = dart.BoxShape(dim)
    shape_node1 = body1.create_shape_node(shape1)
    shape_node1.create_visual_aspect()
    shape_node1.create_collision_aspect()
    shape_node1.create_dynamics_aspect()

    parent_node = body1

    for i in range(1, n):
        joint_name = f"joint{i + 1}"

        joint, body = add_1dof_joint(
            robot, parent_node, joint_name, 0.0, -math.pi, math.pi, dof_type
        )
        body.get_inertia().set_local_com(np.array([0.0, 0.0, dim[2] / 2.0]))

        T = np.eye(4)
        T[2, 3] = dim[2]
        joint.set_transform_from_parent_body_node(T)
        joint.get_dof(0).set_damping_coefficient(0.01)

        shape = dart.BoxShape(dim)
        shape_node = body.create_shape_node(shape)
        shape_node.create_visual_aspect()
        shape_node.create_collision_aspect()
        shape_node.create_dynamics_aspect()

        parent_node = body

    if finished:
        add_end_effector(robot, parent_node, dim)

    return robot


def create_ground(
    size: np.ndarray,
    position: np.ndarray | None = None,
    orientation: np.ndarray | None = None,
) -> dart.Skeleton:
    """Create a ground plane (welded to world)."""
    if position is None:
        position = np.zeros(3)
    if orientation is None:
        orientation = np.zeros(3)

    mass = 1.0

    skeleton = dart.Skeleton()
    [joint, body] = skeleton.create_weld_joint_and_body_node_pair(None)
    joint.set_name("joint1")
    body.get_inertia().set_mass(mass)

    # Set transform
    T = np.eye(4)
    T[:3, 3] = position
    T[:3, :3] = dart.euler_xyz_to_matrix(orientation)
    joint.set_transform_from_parent_body_node(T)

    # Add shape
    shape = dart.BoxShape(size)
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect()
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()

    return skeleton


def create_free_body(
    position: np.ndarray | None = None,
    orientation: np.ndarray | None = None,
) -> dart.Skeleton:
    """Create a free-floating body."""
    if position is None:
        position = np.zeros(3)
    if orientation is None:
        orientation = np.zeros(3)

    mass = 1.0

    skeleton = dart.Skeleton()
    [joint, body] = skeleton.create_free_joint_and_body_node_pair(None)
    joint.set_name("joint1")
    body.get_inertia().set_mass(mass)

    # Set position using Isometry3
    iso = dart.Isometry3()
    iso.set_translation(position)
    iso.set_rotation(dart.euler_xyz_to_matrix(orientation))
    joint.set_transform(iso)

    return skeleton


def create_sphere(
    radius: float,
    position: np.ndarray | None = None,
) -> dart.Skeleton:
    """Create a free-floating sphere."""
    sphere = create_free_body(position)

    bn = sphere.get_body_node(0)
    sphere_shape = dart.SphereShape(radius)
    shape_node = bn.create_shape_node(sphere_shape)
    shape_node.create_visual_aspect()
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()

    return sphere


def create_box(
    size: np.ndarray,
    position: np.ndarray | None = None,
    orientation: np.ndarray | None = None,
) -> dart.Skeleton:
    """Create a free-floating box."""
    box = create_free_body(position, orientation)

    bn = box.get_body_node(0)
    box_shape = dart.BoxShape(size)
    shape_node = bn.create_shape_node(box_shape)
    shape_node.create_visual_aspect()
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()

    return box
