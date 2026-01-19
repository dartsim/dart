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


def test_get_multibody():
    exp = dart.simulation_experimental
    world = exp.World()
    robot = world.add_multi_body("my_robot")

    result = world.get_multi_body("my_robot")
    assert result is not None
    assert result.get_name() == "my_robot"

    result_none = world.get_multi_body("nonexistent")
    assert result_none is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
