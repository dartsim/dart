import platform

import dartpy as dart
import numpy as np
import pytest


def test_ball_joint_constraint():
    world = dart.io.SkelParser.readWorld("dart://sample/skel/chain.skel")
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
    constraint = dart.dynamics.BallJointConstraint(bd1, bd2, joint_pos)
    assert constraint.getType() == dart.dynamics.BallJointConstraint.getStaticType()

    # Add ball joint constraint to the constraint solver
    constraint_solver = world.getConstraintSolver()
    constraint_solver.addConstraint(constraint)

    # Check if the ball joint constraint is being satisfied
    for _ in range(100):
        world.step()
        pos1 = bd1.getTransform().multiply(offset1)
        pos2 = bd2.getTransform().multiply(offset2)
        assert np.isclose(pos1, pos2).all()


if __name__ == "__main__":
    pytest.main()
