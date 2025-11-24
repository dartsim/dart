import platform

import dartpy as dart
import numpy as np
import pytest


def test_ball_joint_constraint():
    world = dart.utils.SkelParser.read_world("dart://sample/skel/chain.skel")
    world.set_gravity([0, -9.81, 0])
    world.set_time_step(1.0 / 2000)

    # Set joint damping
    chain = world.get_skeleton(0)
    for i in range(chain.get_num_joints()):
        joint = chain.get_joint(i)
        for j in range(joint.get_num_dofs()):
            joint.set_damping_coefficient(j, 0.01)

    # Create a ball joint constraint
    bd1 = chain.get_body_node("link 6")
    bd2 = chain.get_body_node("link 10")
    offset1 = [0, 0.025, 0]
    joint_pos = bd1.get_transform().multiply(offset1)
    offset2 = bd2.get_transform().inverse().multiply(joint_pos)
    constraint = dart.constraint.BallJointConstraint(bd1, bd2, joint_pos)
    assert constraint.get_type() == dart.constraint.BallJointConstraint.get_static_type()

    # Add ball joint constraint to the constraint solver
    constraint_solver = world.get_constraint_solver()
    constraint_solver.add_constraint(constraint)

    # Check if the ball joint constraint is being satisfied
    for _ in range(100):
        world.step()
        pos1 = bd1.get_transform().multiply(offset1)
        pos2 = bd2.get_transform().multiply(offset2)
        assert np.isclose(pos1, pos2).all()


if __name__ == "__main__":
    pytest.main()
