"""Tests for DART7 Python API

This module tests binary serialization/deserialization for complete world state.
All tests mirror the C++ serialization test suite to ensure parity.
"""

import io

import dartpy7 as dart7
import numpy as np
import pytest


class TestSerializationBasics:
    """Basic serialization functionality tests."""

    def test_empty_world(self):
        """Test save/load empty world."""
        world1 = dart7.World()

        # Save to bytes
        buffer = io.BytesIO()
        world1.save_binary(buffer)

        # Load into new world
        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        # Verify empty
        assert world2.get_multi_body_count() == 0
        assert world2.get_rigid_body_count() == 0
        assert not world2.is_simulation_mode()

    def test_single_multi_body_no_links(self):
        """Test save/load world with single multibody (no links)."""
        world1 = dart7.World()
        world1.add_multi_body("robot1")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        assert world2.get_multi_body_count() == 1
        mb_restored = world2.get_multi_body("robot1")
        assert mb_restored is not None
        assert mb_restored.get_name() == "robot1"
        assert mb_restored.get_link_count() == 0
        assert mb_restored.get_joint_count() == 0

    def test_single_root_link(self):
        """Test save/load world with single link (root, no parent joint)."""
        world1 = dart7.World()
        mb = world1.add_multi_body("robot")
        mb.add_link("base")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        mb_restored = world2.get_multi_body("robot")
        assert mb_restored is not None
        assert mb_restored.get_name() == "robot"
        assert mb_restored.get_link_count() == 1
        assert mb_restored.get_joint_count() == 0

    def test_two_link_chain(self):
        """Test save/load simple 2-link chain with full property verification."""
        world1 = dart7.World()
        mb = world1.add_multi_body("robot")
        base = mb.add_link("base")
        mb.add_link(
            "link1",
            base,
            "joint1",
            dart7.JointType.Revolute,
            np.array([0, 0, 1], dtype=np.float64),
        )

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        # Verify Multibody restored
        mb_restored = world2.get_multi_body("robot")
        assert mb_restored is not None
        assert mb_restored.get_name() == "robot"
        assert mb_restored.get_link_count() == 2
        assert mb_restored.get_joint_count() == 1

        # Verify links are accessible by name
        base_restored = mb_restored.get_link("base")
        assert base_restored is not None
        assert base_restored.get_name() == "base"

        link1_restored = mb_restored.get_link("link1")
        assert link1_restored is not None
        assert link1_restored.get_name() == "link1"

        # Verify joint properties
        joint1_restored = mb_restored.get_joint("joint1")
        assert joint1_restored is not None
        assert joint1_restored.get_name() == "joint1"
        assert joint1_restored.get_type() == dart7.JointType.Revolute

        # Verify joint axis
        axis = joint1_restored.get_axis()
        assert axis[0] == pytest.approx(0.0)
        assert axis[1] == pytest.approx(0.0)
        assert axis[2] == pytest.approx(1.0)

    def test_preserves_names(self):
        """Test save/load preserves names."""
        world1 = dart7.World()
        mb = world1.add_multi_body("test_robot")
        base = mb.add_link("base_link")
        mb.add_link("arm_link", base, "shoulder_joint")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        mb_restored = world2.get_multi_body("test_robot")
        assert mb_restored is not None
        assert mb_restored.get_name() == "test_robot"


class TestSerializationRigidBodies:
    """Tests for serialization with rigid bodies."""

    def test_with_rigid_bodies(self):
        """Test save/load with rigid bodies."""
        world1 = dart7.World()
        world1.add_rigid_body("box1")
        world1.add_rigid_body("box2")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        assert world2.get_rigid_body_count() == 2
        assert world2.has_rigid_body("box1")
        assert world2.has_rigid_body("box2")

    def test_mixed_content(self):
        """Test mixed multibodies and rigid bodies."""
        world1 = dart7.World()

        mb = world1.add_multi_body("robot")
        mb.add_link("base")

        world1.add_rigid_body("box1")
        world1.add_rigid_body("box2")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        assert world2.get_multi_body_count() == 1
        assert world2.get_rigid_body_count() == 2
        assert world2.has_multi_body("robot")
        assert world2.has_rigid_body("box1")
        assert world2.has_rigid_body("box2")


class TestSerializationModes:
    """Tests for simulation mode and state preservation."""

    def test_preserves_simulation_mode(self):
        """Test save/load preserves simulation mode."""
        world1 = dart7.World()
        world1.add_multi_body("robot")
        world1.enter_simulation_mode()
        assert world1.is_simulation_mode()

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        assert world2.is_simulation_mode()

    def test_preserves_counters(self):
        """Test save/load preserves auto-generation counters."""
        world1 = dart7.World()
        world1.add_multi_body()  # multi_body_001
        world1.add_multi_body()  # multi_body_002

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        # Next auto-generated name should continue sequence
        mb3 = world2.add_multi_body()
        assert mb3.get_name() == "multi_body_003"

    def test_load_clears_existing(self):
        """Test load clears existing state."""
        world1 = dart7.World()
        world1.add_multi_body("robot1")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        world2 = dart7.World()
        world2.add_multi_body("robot2")
        world2.add_multi_body("robot3")
        assert world2.get_multi_body_count() == 2

        buffer.seek(0)
        world2.load_binary(buffer)
        assert world2.get_multi_body_count() == 1
        assert world2.has_multi_body("robot1")
        assert not world2.has_multi_body("robot2")
        assert not world2.has_multi_body("robot3")


class TestSerializationJointTypes:
    """Tests for different joint types."""

    def test_joint_type_revolute(self):
        """Test saves and restores joint type (REVOLUTE)."""
        world1 = dart7.World()
        mb = world1.add_multi_body("robot")
        base = mb.add_link("base")
        mb.add_link(
            "link1",
            base,
            "j1",
            dart7.JointType.Revolute,
            np.array([0, 0, 1], dtype=np.float64),
        )

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        mb2 = world2.get_multi_body("robot")
        assert mb2 is not None
        assert mb2.get_joint_count() == 1

    def test_joint_type_prismatic(self):
        """Test saves and restores joint type (PRISMATIC)."""
        world1 = dart7.World()
        mb = world1.add_multi_body("robot")
        base = mb.add_link("base")
        mb.add_link(
            "link1",
            base,
            "j1",
            dart7.JointType.Prismatic,
            np.array([1, 0, 0], dtype=np.float64),
        )

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        mb2 = world2.get_multi_body("robot")
        assert mb2 is not None
        assert mb2.get_joint_count() == 1


class TestSerializationComplexHierarchies:
    """Tests for complex kinematic structures."""

    def test_complex_hierarchy(self):
        """Test complex hierarchy (6-DOF manipulator)."""
        world1 = dart7.World()
        robot = world1.add_multi_body("ur5")
        base = robot.add_link("base")
        link1 = robot.add_link(
            "shoulder_link",
            base,
            "shoulder_pan",
            dart7.JointType.Revolute,
            np.array([0, 0, 1], dtype=np.float64),
        )
        link2 = robot.add_link(
            "upper_arm_link",
            link1,
            "shoulder_lift",
            dart7.JointType.Revolute,
            np.array([0, 1, 0], dtype=np.float64),
        )
        link3 = robot.add_link(
            "forearm_link",
            link2,
            "elbow",
            dart7.JointType.Revolute,
            np.array([0, 1, 0], dtype=np.float64),
        )
        link4 = robot.add_link(
            "wrist_1_link",
            link3,
            "wrist_1",
            dart7.JointType.Revolute,
            np.array([0, 1, 0], dtype=np.float64),
        )
        link5 = robot.add_link(
            "wrist_2_link",
            link4,
            "wrist_2",
            dart7.JointType.Revolute,
            np.array([0, 0, 1], dtype=np.float64),
        )
        robot.add_link(
            "wrist_3_link",
            link5,
            "wrist_3",
            dart7.JointType.Revolute,
            np.array([0, 1, 0], dtype=np.float64),
        )

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        robot_restored = world2.get_multi_body("ur5")
        assert robot_restored is not None
        assert robot_restored.get_link_count() == 7
        assert robot_restored.get_joint_count() == 6

        # Verify all link names are preserved
        assert robot_restored.get_link("base") is not None
        assert robot_restored.get_link("shoulder_link") is not None
        assert robot_restored.get_link("upper_arm_link") is not None
        assert robot_restored.get_link("forearm_link") is not None
        assert robot_restored.get_link("wrist_1_link") is not None
        assert robot_restored.get_link("wrist_2_link") is not None
        assert robot_restored.get_link("wrist_3_link") is not None

        # Verify all joints and their properties
        shoulder_pan = robot_restored.get_joint("shoulder_pan")
        assert shoulder_pan is not None
        assert shoulder_pan.get_type() == dart7.JointType.Revolute
        axis1 = shoulder_pan.get_axis()
        assert axis1[0] == pytest.approx(0.0)
        assert axis1[1] == pytest.approx(0.0)
        assert axis1[2] == pytest.approx(1.0)

        shoulder_lift = robot_restored.get_joint("shoulder_lift")
        assert shoulder_lift is not None
        assert shoulder_lift.get_type() == dart7.JointType.Revolute
        axis2 = shoulder_lift.get_axis()
        assert axis2[0] == pytest.approx(0.0)
        assert axis2[1] == pytest.approx(1.0)
        assert axis2[2] == pytest.approx(0.0)

        elbow = robot_restored.get_joint("elbow")
        assert elbow is not None
        assert elbow.get_type() == dart7.JointType.Revolute

        wrist_1 = robot_restored.get_joint("wrist_1")
        assert wrist_1 is not None
        assert wrist_1.get_type() == dart7.JointType.Revolute

        wrist_2 = robot_restored.get_joint("wrist_2")
        assert wrist_2 is not None
        assert wrist_2.get_type() == dart7.JointType.Revolute
        axis5 = wrist_2.get_axis()
        assert axis5[0] == pytest.approx(0.0)
        assert axis5[1] == pytest.approx(0.0)
        assert axis5[2] == pytest.approx(1.0)

        wrist_3 = robot_restored.get_joint("wrist_3")
        assert wrist_3 is not None
        assert wrist_3.get_type() == dart7.JointType.Revolute

    def test_multiple_multibodies(self):
        """Test multiple multibodies."""
        world1 = dart7.World()
        mb1 = world1.add_multi_body("robot1")
        mb2 = world1.add_multi_body("robot2")
        mb3 = world1.add_multi_body("robot3")

        mb1.add_link("base1")
        mb2.add_link("base2")
        mb3.add_link("base3")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        assert world2.get_multi_body_count() == 3
        assert world2.has_multi_body("robot1")
        assert world2.has_multi_body("robot2")
        assert world2.has_multi_body("robot3")

    def test_large_hierarchy(self):
        """Test large hierarchy (stress test)."""
        world1 = dart7.World()
        robot = world1.add_multi_body("large_robot")

        # Create chain of 20 links
        prev = robot.add_link("base")
        for i in range(1, 20):
            prev = robot.add_link(f"link_{i}", prev, f"joint_{i}")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        robot2 = world2.get_multi_body("large_robot")
        assert robot2 is not None
        assert robot2.get_link_count() == 20
        assert robot2.get_joint_count() == 19


class TestSerializationEdgeCases:
    """Tests for edge cases and special scenarios."""

    def test_multiple_cycles(self):
        """Test multiple save/load cycles."""
        world1 = dart7.World()
        mb = world1.add_multi_body("robot")
        mb.add_link("base")

        for _ in range(3):
            buffer = io.BytesIO()
            world1.save_binary(buffer)

            buffer.seek(0)
            world2 = dart7.World()
            world2.load_binary(buffer)

            # Copy back
            buffer2 = io.BytesIO()
            world2.save_binary(buffer2)
            buffer2.seek(0)
            world1.load_binary(buffer2)

            assert world1.get_multi_body_count() == 1
            mb_restored = world1.get_multi_body("robot")
            assert mb_restored is not None
            assert mb_restored.get_link_count() == 1

    def test_empty_names(self):
        """Test auto-generated names are preserved."""
        world1 = dart7.World()
        mb = world1.add_multi_body()  # Auto-generated name: "multi_body_001"
        mb.add_link()  # Auto-generated name

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        assert world2.get_multi_body_count() == 1
        mb2 = world2.get_multi_body("multi_body_001")
        assert mb2 is not None
        assert mb2.get_link_count() == 1

    def test_binary_stability(self):
        """Test binary format is stable (no extra bytes)."""
        world1 = dart7.World()
        mb = world1.add_multi_body("robot")
        mb.add_link("base")

        # Save twice
        buffer1 = io.BytesIO()
        world1.save_binary(buffer1)

        buffer2 = io.BytesIO()
        world1.save_binary(buffer2)

        # Should produce identical output
        assert buffer1.getvalue() == buffer2.getvalue()

    def test_entity_count(self):
        """Test entity count is correct."""
        world1 = dart7.World()

        # Create specific number of entities
        mb1 = world1.add_multi_body("mb1")
        mb2 = world1.add_multi_body("mb2")
        mb1.add_link("link1")
        mb1.add_link("link2")
        mb2.add_link("link3")
        world1.add_rigid_body("rb1")

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        # Verify restoration
        assert world2.get_multi_body_count() == 2
        assert world2.get_rigid_body_count() == 1

    def test_metadata_fields(self):
        """Test all metadata fields are preserved."""
        world1 = dart7.World()

        # Advance counters by creating and manipulating entities
        world1.add_multi_body()  # counter = 1
        world1.add_multi_body()  # counter = 2
        world1.add_multi_body()  # counter = 3

        mb = world1.add_multi_body("test")
        mb.add_link()  # link counter = 1

        buffer = io.BytesIO()
        world1.save_binary(buffer)

        buffer.seek(0)
        world2 = dart7.World()
        world2.load_binary(buffer)

        # Verify counters by creating new entities
        newMb = world2.add_multi_body()
        assert newMb.get_name() == "multi_body_005"
