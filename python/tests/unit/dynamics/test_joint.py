import math

import dartpy as dart
import numpy as np
import pytest


def kinematics_tester(joint):
    num_tests = 2

    joint.set_transform_from_child_body_node(dart.math.exp_map(np.random.rand(6)))
    joint.set_transform_from_parent_body_node(dart.math.exp_map(np.random.rand(6)))

    dof = joint.get_num_dofs()

    q = np.zeros(dof)
    dq = np.zeros(dof)

    for _ in range(num_tests):
        q_delta = 1e-5

        for i in range(dof):
            q[i] = dart.math.Random.uniform(-math.pi, math.pi)
            dq[i] = dart.math.Random.uniform(-math.pi, math.pi)

        joint.set_positions(q)
        joint.set_velocities(dq)

        if dof == 0:
            return

        T = joint.get_relative_transform()
        J = joint.get_relative_jacobian(q)
        dJ = joint.get_relative_jacobian_time_deriv()

        # Verify transform
        assert dart.math.verify_transform(T)

        # Test analytic Jacobian and numerical Jacobian
        numericJ = np.zeros((6, dof))
        for i in range(dof):
            q_a = q.copy()
            joint.set_positions(q_a)
            T_a = joint.get_relative_transform()

            q_b = q.copy()
            q_b[i] += q_delta
            joint.set_positions(q_b)
            T_b = joint.get_relative_transform()

            Tinv_a = T_a.inverse()

            dTdq = (T_b.matrix() - T_a.matrix()) / q_delta

            Ji_4x4matrix = np.matmul(Tinv_a.matrix(), dTdq)
            Ji = np.zeros(6)
            Ji[0] = Ji_4x4matrix[2, 1]
            Ji[1] = Ji_4x4matrix[0, 2]
            Ji[2] = Ji_4x4matrix[1, 0]
            Ji[3] = Ji_4x4matrix[0, 3]
            Ji[4] = Ji_4x4matrix[1, 3]
            Ji[5] = Ji_4x4matrix[2, 3]
            numericJ[:, i] = Ji

        assert np.allclose(J, numericJ, atol=1e-5)


def test_kinematics():
    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_weld_joint_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_revolute_joint_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_prismatic_joint_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_screw_joint_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_universal_joint_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_translational_joint2_d_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_euler_joint_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_translational_joint_and_body_node_pair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_planar_joint_and_body_node_pair()
    kinematics_tester(joint)


def test_access_to_parent_child_transforms():
    skel = dart.dynamics.Skeleton()
    joint, _ = skel.create_revolute_joint_and_body_node_pair()

    parentToJointTf = dart.math.Isometry3.Identity()
    parentToJointTf.set_translation(np.random.rand(3, 1))
    childToJointTf = dart.math.Isometry3.Identity()
    childToJointTf.set_translation(np.random.rand(3, 1))

    joint.set_transform_from_parent_body_node(parentToJointTf)
    joint.set_transform_from_child_body_node(childToJointTf)

    storedParentTf = joint.get_transform_from_parent_body_node()
    storedChildTf = joint.get_transform_from_child_body_node()

    assert np.allclose(parentToJointTf.matrix(), storedParentTf.matrix())
    assert np.allclose(childToJointTf.matrix(), storedChildTf.matrix())


def test_ball_joint_positions_conversion():
    assert np.allclose(
        dart.dynamics.BallJoint.convert_to_positions(np.eye(3)), np.zeros((1, 3))
    )
    assert np.allclose(
        dart.dynamics.BallJoint.convert_to_positions(
            np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        ),
        np.array([0, 0, -np.pi / 2]),
    )

    for i in range(30):
        ballJointPos = np.random.uniform(-np.pi / 2, np.pi / 2, 3)
        assert np.allclose(
            dart.dynamics.BallJoint.convert_to_rotation(
                dart.dynamics.BallJoint.convert_to_positions(
                    dart.dynamics.BallJoint.convert_to_rotation(ballJointPos)
                )
            ),
            dart.dynamics.BallJoint.convert_to_rotation(ballJointPos),
        )


if __name__ == "__main__":
    pytest.main()
