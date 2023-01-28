import math

import dartpy as dart
import numpy as np
import pytest


def kinematics_tester(joint):
    num_tests = 2

    joint.setTransformFromChildBodyNode(dart.math.expMap(np.random.rand(6)))
    joint.setTransformFromParentBodyNode(dart.math.expMap(np.random.rand(6)))

    dof = joint.getNumDofs()

    q = np.zeros(dof)
    dq = np.zeros(dof)

    for _ in range(num_tests):
        q_delta = 1e-5

        q = dart.math.UniformFloat(dof, -math.pi, math.pi)
        dq = dart.math.UniformFloat(dof, -math.pi, math.pi)

        joint.setPositions(q)
        joint.setVelocities(dq)

        if dof == 0:
            return

        T = joint.getRelativeTransform()
        J = joint.getRelativeJacobian(q)
        dJ = joint.getRelativeJacobianTimeDeriv()

        # Verify transform
        assert dart.math.verifyTransform(T)

        # Test analytic Jacobian and numerical Jacobian
        numericJ = np.zeros((6, dof))
        for i in range(dof):
            q_a = q.copy()
            joint.setPositions(q_a)
            T_a = joint.getRelativeTransform()

            q_b = q.copy()
            q_b[i] += q_delta
            joint.setPositions(q_b)
            T_b = joint.getRelativeTransform()

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
    joint, _ = skel.createWeldJointAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createRevoluteJointAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createPrismaticJointAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createScrewJointAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createUniversalJointAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createTranslationalJoint2DAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createEulerJointAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createTranslationalJointAndBodyNodePair()
    kinematics_tester(joint)

    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createPlanarJointAndBodyNodePair()
    kinematics_tester(joint)


def test_access_to_parent_child_transforms():
    skel = dart.dynamics.Skeleton()
    joint, _ = skel.createRevoluteJointAndBodyNodePair()

    parentToJointTf = dart.math.Isometry3.Identity()
    parentToJointTf.set_translation(np.random.rand(3, 1))
    childToJointTf = dart.math.Isometry3.Identity()
    childToJointTf.set_translation(np.random.rand(3, 1))

    joint.setTransformFromParentBodyNode(parentToJointTf)
    joint.setTransformFromChildBodyNode(childToJointTf)

    storedParentTf = joint.getTransformFromParentBodyNode()
    storedChildTf = joint.getTransformFromChildBodyNode()

    assert np.allclose(parentToJointTf.matrix(), storedParentTf.matrix())
    assert np.allclose(childToJointTf.matrix(), storedChildTf.matrix())


def test_BallJoint_positions_conversion():
    assert np.allclose(
        dart.dynamics.BallJoint.convertToPositions(np.eye(3)), np.zeros((1, 3))
    )
    assert np.allclose(
        dart.dynamics.BallJoint.convertToPositions(
            np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        ),
        np.array([0, 0, -np.pi / 2]),
    )

    for i in range(30):
        ballJointPos = np.random.uniform(-np.pi / 2, np.pi / 2, 3)
        assert np.allclose(
            dart.dynamics.BallJoint.convertToRotation(
                dart.dynamics.BallJoint.convertToPositions(
                    dart.dynamics.BallJoint.convertToRotation(ballJointPos)
                )
            ),
            dart.dynamics.BallJoint.convertToRotation(ballJointPos),
        )


if __name__ == "__main__":
    pytest.main()
