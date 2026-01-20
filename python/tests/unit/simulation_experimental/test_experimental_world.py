import numpy as np
import pytest

import dartpy as dart


def test_world_creation():
    exp = dart.simulation_experimental
    world = exp.World()
    assert world is not None


def test_multibody_creation():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("robot")
    assert robot.get_name() == "robot"
    assert robot.get_link_count() == 0
    assert robot.get_joint_count() == 0


def test_link_creation():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("robot")

    root = robot.add_link("base")
    assert root.get_name() == "base"
    assert robot.get_link_count() == 1

    child = robot.add_link_with_joint(
        "link1",
        root,
        "shoulder",
        exp.JointType.Revolute,
        np.array([0, 0, 1], dtype=float),
    )
    assert child.get_name() == "link1"
    assert robot.get_link_count() == 2
    assert robot.get_joint_count() == 1


def test_joint_properties():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("robot")

    root = robot.add_link("base")
    child = robot.add_link_with_joint(
        "link1", root, "j1", exp.JointType.Revolute, np.array([0, 0, 1], dtype=float)
    )

    joint = child.get_parent_joint()
    assert joint.get_name() == "j1"
    assert joint.get_type() == exp.JointType.Revolute
    assert joint.get_dof() == 1
    assert joint.is_valid()


def test_joint_state_accessors():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("robot")

    root = robot.add_link("base")
    child = robot.add_link_with_joint(
        "link1", root, "j1", exp.JointType.Revolute, np.array([0, 0, 1], dtype=float)
    )

    joint = child.get_parent_joint()

    pos = joint.get_position()
    assert len(pos) == 1
    assert pos[0] == 0.0

    joint.set_position(np.array([1.5]))
    pos = joint.get_position()
    assert np.isclose(pos[0], 1.5)


def test_joint_types_dof():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("robot")

    root = robot.add_link("base")

    for joint_type, expected_dof in [
        (exp.JointType.Fixed, 0),
        (exp.JointType.Revolute, 1),
        (exp.JointType.Prismatic, 1),
        (exp.JointType.Screw, 1),
        (exp.JointType.Universal, 2),
        (exp.JointType.Ball, 3),
        (exp.JointType.Planar, 3),
        (exp.JointType.Free, 6),
    ]:
        child = robot.add_link_with_joint(
            f"link_{joint_type.name}",
            root,
            f"j_{joint_type.name}",
            joint_type,
            np.array([0, 0, 1], dtype=float),
        )
        joint = child.get_parent_joint()
        assert joint.get_dof() == expected_dof, (
            f"{joint_type.name}: expected {expected_dof}, got {joint.get_dof()}"
        )


def test_rigid_body_creation():
    exp = dart.simulation_experimental
    world = exp.World()

    rb = world.add_rigid_body("box")
    assert rb.get_name() == "box"
    assert rb.get_mass() == 1.0
    assert world.get_rigid_body_count() == 1


def test_rigid_body_properties():
    exp = dart.simulation_experimental
    world = exp.World()

    opts = exp.RigidBodyOptions()
    opts.mass = 5.0
    opts.position = np.array([1.0, 2.0, 3.0])

    rb = world.add_rigid_body("box", opts)
    assert rb.get_mass() == 5.0

    pos = rb.get_position()
    assert np.allclose(pos, [1.0, 2.0, 3.0])


def test_frame_transforms():
    exp = dart.simulation_experimental
    world = exp.World()

    frame1 = world.add_free_frame("f1")
    tf = frame1.get_transform()
    assert tf is not None


def test_shape_node_creation():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("robot")
    base = robot.add_link("base")

    shape = dart.dynamics.BoxShape([1.0, 2.0, 3.0])
    options = exp.ShapeNodeOptions()
    options.collidable = False
    options.friction_coeff = 0.7
    options.restitution_coeff = 0.4

    node = base.create_shape_node(shape, "base_shape", options)
    assert node.get_name() == "base_shape"
    assert not node.is_collidable()
    assert node.get_friction_coeff() == pytest.approx(0.7)
    assert node.get_restitution_coeff() == pytest.approx(0.4)


def test_get_multibody():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("my_robot")

    result = world.get_multi_body("my_robot")
    assert result is not None
    assert result.get_name() == "my_robot"

    result_none = world.get_multi_body("nonexistent")
    assert result_none is None


def test_state_space_creation():
    exp = dart.simulation_experimental
    space = exp.StateSpace()
    assert not space.is_finalized()
    assert space.get_dimension() == 0
    assert space.get_num_variables() == 0


def test_state_space_add_variable():
    exp = dart.simulation_experimental
    space = exp.StateSpace()
    space.add_variable("joint_pos", 6, -3.14, 3.14)
    space.add_variable("joint_vel", 6, -10.0, 10.0)

    assert space.get_num_variables() == 2
    assert space.get_dimension() == 12
    assert space.has_variable("joint_pos")
    assert space.has_variable("joint_vel")
    assert not space.has_variable("nonexistent")


def test_state_space_finalize():
    exp = dart.simulation_experimental
    space = exp.StateSpace()
    space.add_variable("x", 3)
    space.finalize()

    assert space.is_finalized()


def test_state_space_bounds():
    exp = dart.simulation_experimental
    space = exp.StateSpace()
    space.add_variable("pos", 2, -1.0, 1.0)
    space.add_variable("vel", 2, -5.0, 5.0)
    space.finalize()

    lower = space.get_lower_bounds()
    upper = space.get_upper_bounds()

    assert len(lower) == 4
    assert len(upper) == 4
    assert np.allclose(lower, [-1.0, -1.0, -5.0, -5.0])


def test_world_step_advances_time():
    exp = dart.simulation_experimental
    world = exp.World()
    world.set_time_step(0.01)
    world.enter_simulation_mode()

    assert world.get_time() == 0.0
    world.step()
    assert world.get_time() == pytest.approx(0.01)
    assert np.allclose(upper, [1.0, 1.0, 5.0, 5.0])


def test_state_space_get_variable():
    exp = dart.simulation_experimental
    space = exp.StateSpace()
    space.add_variable("q", 3, -3.14, 3.14)
    space.finalize()

    var = space.get_variable("q")
    assert var is not None
    assert var.name == "q"
    assert var.dimension == 3
    assert var.start_index == 0
    assert np.isclose(var.lower_bound, -3.14)
    assert np.isclose(var.upper_bound, 3.14)

    none_var = space.get_variable("nonexistent")
    assert none_var is None


def test_state_space_variable_names():
    exp = dart.simulation_experimental
    space = exp.StateSpace()
    space.add_variable("a", 1)
    space.add_variable("b", 2)
    space.add_variable("c", 3)

    names = space.get_variable_names()
    assert names == ["a", "b", "c"]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
