import dartpy as dart
import numpy as np
import pytest


def test_ball_joint_constraint():
    world = dart.io.SkelParser.read_world("dart://sample/skel/chain.skel")
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
    constraint = dart.BallJointConstraint(bd1, bd2, joint_pos)
    assert constraint.get_type() == dart.BallJointConstraint.get_static_type()

    # Add ball joint constraint to the constraint solver
    constraint_solver = world.get_constraint_solver()
    constraint_solver.add_constraint(constraint)

    # Check if the ball joint constraint is being satisfied
    for _ in range(100):
        world.step()
        pos1 = bd1.get_transform().multiply(offset1)
        pos2 = bd2.get_transform().multiply(offset2)
        assert np.isclose(pos1, pos2).all()


def test_revolute_joint_constraint():
    world = dart.io.SkelParser.read_world("dart://sample/skel/chain.skel")
    world.set_gravity([0, -9.81, 0])
    world.set_time_step(1.0 / 2000)

    chain = world.get_skeleton(0)
    init_pose = np.zeros(chain.get_num_dofs())
    init_pose[[20, 23, 26, 29]] = np.pi * 0.4
    chain.set_positions(init_pose)

    for i in range(chain.get_num_joints()):
        joint = chain.get_joint(i)
        for j in range(joint.get_num_dofs()):
            joint.set_damping_coefficient(j, 0.01)

    bd1 = chain.get_body_node("link 6")
    bd2 = chain.get_body_node("link 10")
    offset1 = [0, 0.025, 0]
    joint_pos = bd1.get_transform().multiply(offset1)
    offset2 = bd2.get_transform().inverse().multiply(joint_pos)
    axis = [0, 1, 0]

    constraint = dart.RevoluteJointConstraint(bd1, bd2, joint_pos, axis, axis)
    assert (
        constraint.get_type()
        == dart.RevoluteJointConstraint.get_static_type()
    )

    constraint_solver = world.get_constraint_solver()
    constraint_solver.add_constraint(constraint)

    for _ in range(200):
        world.step()
        pos1 = bd1.get_transform().multiply(offset1)
        pos2 = bd2.get_transform().multiply(offset2)
        assert np.linalg.norm(pos1 - pos2) < 5e-2

    axis1 = bd1.get_transform().rotation().dot(np.array([0.0, 1.0, 0.0]))
    axis2 = bd2.get_transform().rotation().dot(np.array([0.0, 1.0, 0.0]))
    axis1 /= np.linalg.norm(axis1)
    axis2 /= np.linalg.norm(axis2)
    assert np.dot(axis1, axis2) > 0.2


def test_contact_manifold_cache_options():
    world = dart.World("world")
    solver = world.get_constraint_solver()

    options = dart.ContactManifoldCacheOptions()
    options.enabled = True
    options.max_points_per_pair = 2
    options.max_pairs = 8
    options.max_separation_frames = 1
    options.position_threshold = 1e-3
    options.normal_threshold = 0.9
    options.depth_epsilon = 1e-5

    solver.set_contact_manifold_cache_options(options)
    assert solver.is_contact_manifold_cache_enabled()

    readback = solver.get_contact_manifold_cache_options()
    assert readback.enabled is True
    assert readback.max_points_per_pair == 2
    assert readback.max_pairs == 8
    assert readback.max_separation_frames == 1
    assert readback.position_threshold == pytest.approx(1e-3)
    assert readback.normal_threshold == pytest.approx(0.9)
    assert readback.depth_epsilon == pytest.approx(1e-5)

    assert solver.get_num_persistent_contacts() == 0
    assert solver.get_num_contact_manifolds() == 0
    assert solver.get_num_contact_constraints() == 0
    assert solver.get_num_soft_contact_constraints() == 0
    assert solver.get_contacts_used_for_constraints() == []

    contact = dart.Contact()
    contact.point = np.array([0.0, 0.0, 0.0])
    contact.normal = np.array([0.0, 1.0, 0.0])
    contact.force = np.array([0.0, 0.0, 0.0])
    contact.penetration_depth = 0.1
    assert dart.Contact.is_non_zero_normal(np.array([0.0, 1.0, 0.0]))


if __name__ == "__main__":
    pytest.main()
