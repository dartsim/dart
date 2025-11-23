import platform

import dartpy as dart
import numpy as np
import pytest


def test_ball_joint_constraint():
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/chain.skel")
    world.setGravity([0, -9.81, 0])
    world.setTimeStep(1.0 / 2000)

    # Set joint damping
    chain = world.getSkeleton(0)
    for i in range(chain.getNumJoints()):
        joint = chain.getJoint(i)
        for j in range(joint.getNumDofs()):
            joint.setDampingCoefficient(j, 0.01)

    # Create a ball joint contraint
    bd1 = chain.getBodyNode("link 6")
    bd2 = chain.getBodyNode("link 10")
    offset1 = [0, 0.025, 0]
    joint_pos = bd1.getTransform().multiply(offset1)
    offset2 = bd2.getTransform().inverse().multiply(joint_pos)
    constraint = dart.constraint.BallJointConstraint(bd1, bd2, joint_pos)
    assert constraint.getType() == dart.constraint.BallJointConstraint.getStaticType()

    # Add ball joint constraint to the constraint solver
    constraint_solver = world.getConstraintSolver()
    constraint_solver.addConstraint(constraint)

    # Check if the ball joint constraint is being satisfied
    for _ in range(100):
        world.step()
        pos1 = bd1.getTransform().multiply(offset1)
        pos2 = bd2.getTransform().multiply(offset2)
        assert np.isclose(pos1, pos2).all()


def test_revolute_joint_constraint():
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/chain.skel")
    world.setGravity([0, -9.81, 0])
    world.setTimeStep(1.0 / 2000)

    chain = world.getSkeleton(0)
    init_pose = np.zeros(chain.getNumDofs())
    init_pose[[20, 23, 26, 29]] = np.pi * 0.4
    chain.setPositions(init_pose)

    for i in range(chain.getNumJoints()):
        joint = chain.getJoint(i)
        for j in range(joint.getNumDofs()):
            joint.setDampingCoefficient(j, 0.01)

    bd1 = chain.getBodyNode("link 6")
    bd2 = chain.getBodyNode("link 10")
    offset = [0, 0.025, 0]
    joint_pos = bd1.getTransform().multiply(offset)
    axis = [0, 1, 0]

    constraint = dart.constraint.RevoluteJointConstraint(
        bd1, bd2, joint_pos, axis, axis
    )
    assert (
        constraint.getType()
        == dart.constraint.RevoluteJointConstraint.getStaticType()
    )

    constraint_solver = world.getConstraintSolver()
    constraint_solver.addConstraint(constraint)

    for _ in range(200):
        world.step()
        pos1 = bd1.getTransform().multiply(offset)
        pos2 = bd2.getTransform().multiply(offset)
        assert np.linalg.norm(pos1 - pos2) < 1e-4

    axis1 = bd1.getTransform().rotation().dot(np.array([0.0, 1.0, 0.0]))
    axis2 = bd2.getTransform().rotation().dot(np.array([0.0, 1.0, 0.0]))
    axis1 /= np.linalg.norm(axis1)
    axis2 /= np.linalg.norm(axis2)
    assert np.isclose(np.dot(axis1, axis2), 1.0, atol=1e-4)


if __name__ == "__main__":
    pytest.main()
