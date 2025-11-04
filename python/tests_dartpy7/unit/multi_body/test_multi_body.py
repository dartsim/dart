"""Tests for MultiBody public API"""

import dartpy7 as d
import numpy as np
import pytest


def test_creation_with_name():
    """Test creating MultiBody with explicit name"""
    world = d.World()
    mb = world.add_multi_body("multibody")

    assert mb.get_name() == "multibody"
    assert mb.get_link_count() == 0
    assert mb.get_joint_count() == 0
    assert mb.get_dof_count() == 0


def test_auto_generated_name():
    """Test creating MultiBody with auto-generated name"""
    world = d.World()

    mb1 = world.add_multi_body()
    mb2 = world.add_multi_body()

    assert mb1.get_name() == "multi_body_001"
    assert mb2.get_name() == "multi_body_002"


def test_mixed_names_and_duplicates():
    """Test mixed explicit and auto-generated names (duplicates OK)"""
    world = d.World()

    mb1 = world.add_multi_body("robot")  # Explicit (counter=1)
    mb2 = world.add_multi_body()  # Auto: "multibody_002" (counter=2)
    mb3 = world.add_multi_body("robot")  # Duplicate name OK! (counter=3)
    mb4 = world.add_multi_body()  # Auto: "multibody_004" (counter=4)

    assert mb1.get_name() == "robot"
    assert mb2.get_name() == "multi_body_002"
    assert mb3.get_name() == "robot"  # Duplicate allowed
    assert mb4.get_name() == "multi_body_004"


def test_custom_names():
    """Test MultiBody creation with different names"""
    world = d.World()

    robot = world.add_multi_body("ur5")
    assert robot.get_name() == "ur5"

    character = world.add_multi_body("humanoid")
    assert character.get_name() == "humanoid"

    mechanism = world.add_multi_body("cabinet")
    assert mechanism.get_name() == "cabinet"


def test_name_get_set():
    """Test MultiBody name get/set"""
    world = d.World()
    mb = world.add_multi_body("initial_name")
    assert mb.get_name() == "initial_name"

    mb.set_name("new_name")
    assert mb.get_name() == "new_name"

    mb.set_name("robot_arm")
    assert mb.get_name() == "robot_arm"


def test_name_property():
    """Test MultiBody.name property getter/setter"""
    world = d.World()
    mb = world.add_multi_body("original_name")

    # Test getter
    assert mb.name == "original_name"
    assert mb.get_name() == "original_name"

    # Test setter
    mb.name = "new_name"
    assert mb.name == "new_name"
    assert mb.get_name() == "new_name"


def test_repr():
    """Test MultiBody string representation"""
    world = d.World()
    mb = world.add_multi_body("test_robot")

    repr_str = repr(mb)
    assert "MultiBody" in repr_str
    assert "test_robot" in repr_str
    assert "0 link" in repr_str
    assert "0 joint" in repr_str
    assert "0 DOF" in repr_str


def test_empty_counts():
    """Test MultiBody counts (currently empty)"""
    world = d.World()
    mb = world.add_multi_body("test")

    # Empty multibody has no links, joints, or DOFs
    assert mb.get_link_count() == 0
    assert mb.get_joint_count() == 0
    assert mb.get_dof_count() == 0


def test_multiple_instances():
    """Test creating multiple MultiBody instances"""
    world = d.World()

    mb1 = world.add_multi_body("robot1")
    mb2 = world.add_multi_body("robot2")
    mb3 = world.add_multi_body("robot3")

    assert world.get_multi_body_count() == 3
    assert mb1.get_name() == "robot1"
    assert mb2.get_name() == "robot2"
    assert mb3.get_name() == "robot3"


def test_empty_name():
    """Test MultiBody with empty string name"""
    world = d.World()
    mb = world.add_multi_body("")  # Should auto-generate

    name = mb.get_name()
    assert name != ""
    assert name.startswith("multi_body_")


def test_multiple_instances_with_modification():
    """Test multiple MultiBody instances in same World"""
    world = d.World()

    mb1 = world.add_multi_body("multibody1")
    mb2 = world.add_multi_body("multibody2")

    assert mb1.get_name() == "multibody1"
    assert mb2.get_name() == "multibody2"

    # Handles are independent
    mb1.set_name("renamed1")
    assert mb1.get_name() == "renamed1"
    assert mb2.get_name() == "multibody2"  # Unchanged


# ==============================================================================
# Joint and Link API Tests
# ==============================================================================


def test_create_root_link():
    """Test creating a single link (root link)"""
    world = d.World()
    robot = world.add_multi_body("robot")

    # Create root link (no parent joint)
    robot.add_link("base")

    assert robot.get_link_count() == 1
    assert robot.get_joint_count() == 0


def test_simple_two_link_chain():
    """Test creating a simple 2-link chain with revolute joint"""
    world = d.World()
    robot = world.add_multi_body("robot")

    # Create kinematic chain: base -> joint1 -> link1
    base_link = robot.add_link("base")
    robot.add_link(
        "link1",
        parent_link=base_link,
        joint_name="joint1",
        joint_type=d.JointType.Revolute,
    )

    assert robot.get_link_count() == 2
    assert robot.get_joint_count() == 1


def test_serial_manipulator():
    """Test creating a serial manipulator (typical robot arm)"""
    world = d.World()
    robot = world.add_multi_body("ur5")

    # Create 6-DOF serial manipulator: base -> 6 revolute joints -> 6 links
    base = robot.add_link("base")
    link1 = robot.add_link("link1", parent_link=base, joint_name="shoulder_pan")
    link2 = robot.add_link("link2", parent_link=link1, joint_name="shoulder_lift")
    link3 = robot.add_link("link3", parent_link=link2, joint_name="elbow")
    link4 = robot.add_link("link4", parent_link=link3, joint_name="wrist1")
    link5 = robot.add_link("link5", parent_link=link4, joint_name="wrist2")
    robot.add_link("link6", parent_link=link5, joint_name="wrist3")

    assert robot.get_link_count() == 7  # base + 6 links
    assert robot.get_joint_count() == 6  # 6 revolute joints


def test_prismatic_joints():
    """Test creating prismatic joints"""
    world = d.World()
    robot = world.add_multi_body("linear_stage")

    # Create 3-DOF Cartesian robot (XYZ linear stages)
    base = robot.add_link("base")
    link_x = robot.add_link(
        "link_x",
        parent_link=base,
        joint_name="joint_x",
        joint_type=d.JointType.Prismatic,
        axis=np.array([1, 0, 0], dtype=np.float64),
    )
    link_y = robot.add_link(
        "link_y",
        parent_link=link_x,
        joint_name="joint_y",
        joint_type=d.JointType.Prismatic,
        axis=np.array([0, 1, 0], dtype=np.float64),
    )
    robot.add_link(
        "link_z",
        parent_link=link_y,
        joint_name="joint_z",
        joint_type=d.JointType.Prismatic,
        axis=np.array([0, 0, 1], dtype=np.float64),
    )

    assert robot.get_link_count() == 4
    assert robot.get_joint_count() == 3


def test_mixed_joint_types():
    """Test mixing revolute and prismatic joints"""
    world = d.World()
    robot = world.add_multi_body("scara")

    # Create SCARA robot: 2 revolute + 1 prismatic + 1 revolute
    base = robot.add_link("base")
    link1 = robot.add_link(
        "link1",
        parent_link=base,
        joint_name="joint1",
        axis=np.array([0, 0, 1], dtype=np.float64),
    )
    link2 = robot.add_link(
        "link2",
        parent_link=link1,
        joint_name="joint2",
        axis=np.array([0, 0, 1], dtype=np.float64),
    )
    link3 = robot.add_link(
        "link3",
        parent_link=link2,
        joint_name="joint3",
        joint_type=d.JointType.Prismatic,
        axis=np.array([0, 0, 1], dtype=np.float64),
    )
    robot.add_link(
        "link4",
        parent_link=link3,
        joint_name="joint4",
        axis=np.array([0, 0, 1], dtype=np.float64),
    )

    assert robot.get_link_count() == 5
    assert robot.get_joint_count() == 4


def test_custom_joint_axes():
    """Test custom joint axes"""
    world = d.World()
    robot = world.add_multi_body("custom_axes")

    base = robot.add_link("base")

    # Joint rotating around custom axis
    custom_axis = np.array([1.0, 1.0, 0.0], dtype=np.float64)
    link1 = robot.add_link(
        "link1", parent_link=base, joint_name="joint1", axis=custom_axis
    )

    # Joint translating along custom direction
    custom_dir = np.array([0.0, 1.0, 1.0], dtype=np.float64)
    robot.add_link(
        "link2",
        parent_link=link1,
        joint_name="joint2",
        joint_type=d.JointType.Prismatic,
        axis=custom_dir,
    )

    assert robot.get_link_count() == 3
    assert robot.get_joint_count() == 2


def test_design_time_restriction():
    """Test design-time restriction: cannot add joints after entering simulation mode"""
    world = d.World()
    robot = world.add_multi_body("robot")

    # Create links in design mode
    base = robot.add_link("base")
    robot.add_link("link1", parent_link=base, joint_name="joint1")

    assert robot.get_link_count() == 2
    assert robot.get_joint_count() == 1

    # Enter simulation mode
    world.enter_simulation_mode()
    assert world.is_simulation_mode()

    # Attempting to create link should throw
    with pytest.raises(RuntimeError):
        robot.add_link("link2")


def test_branching_kinematic_tree():
    """Test branching kinematic tree (not just serial chain)"""
    world = d.World()
    robot = world.add_multi_body("humanoid")

    # Create simplified humanoid structure with branching
    torso = robot.add_link("torso")

    # Left arm branch
    left_shoulder = robot.add_link(
        "left_shoulder", parent_link=torso, joint_name="left_shoulder_joint"
    )
    robot.add_link(
        "left_elbow", parent_link=left_shoulder, joint_name="left_elbow_joint"
    )

    # Right arm branch
    right_shoulder = robot.add_link(
        "right_shoulder", parent_link=torso, joint_name="right_shoulder_joint"
    )
    robot.add_link(
        "right_elbow", parent_link=right_shoulder, joint_name="right_elbow_joint"
    )

    # Head branch
    neck = robot.add_link("neck", parent_link=torso, joint_name="neck_joint")
    robot.add_link("head", parent_link=neck, joint_name="head_joint")

    assert robot.get_link_count() == 7  # torso + 2 shoulders + 2 elbows + neck + head
    assert robot.get_joint_count() == 6  # 6 revolute joints


def test_links_persist_across_handles():
    """Test that links persist across MultiBody handle copies"""
    world = d.World()
    robot1 = world.add_multi_body("robot")

    # Create links via robot1
    base = robot1.add_link("base")
    robot1.add_link("link1", parent_link=base, joint_name="joint1")

    # Get another handle to the same Multibody
    robot2 = world.get_multi_body("robot")
    assert robot2 is not None

    # Both handles see the same links/joints
    assert robot1.get_link_count() == 2
    assert robot2.get_link_count() == 2
    assert robot1.get_joint_count() == 1
    assert robot2.get_joint_count() == 1
