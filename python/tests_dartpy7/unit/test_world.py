#!/usr/bin/env python3
"""Tests for dartpy7 World class."""

import pytest


def test_world_creation():
    """Test World class creation."""
    import dartpy7 as d

    world = d.World()
    assert world is not None


def test_world_repr():
    """Test World repr string."""
    import dartpy7 as d

    world = d.World()
    repr_str = repr(world)
    assert "dart7.World" in repr_str
    assert "0 multibody" in repr_str or "0 multibodies" in repr_str
    assert "0 rigidbody" in repr_str or "0 rigidbodies" in repr_str


def test_add_multi_body_default():
    """Test creating Multibody with default options."""
    import dartpy7 as d

    world = d.World()
    mb = world.add_multi_body("multibody")
    assert mb is not None
    assert world.get_multi_body_count() == 1


def test_add_multi_body_with_name():
    """Test creating Multibody with a name."""
    import dartpy7 as d

    world = d.World()
    mb = world.add_multi_body("robot")
    assert mb is not None
    assert mb.get_name() == "robot"
    assert world.get_multi_body_count() == 1


def test_add_rigid_body_default():
    """Test creating RigidBody with default options."""
    import dartpy7 as d

    world = d.World()
    rb = world.add_rigid_body()
    assert rb is not None
    assert world.get_rigid_body_count() == 1


def test_add_rigid_body_with_name():
    """Test creating RigidBody with a name."""
    import dartpy7 as d

    world = d.World()
    rb = world.add_rigid_body("box")
    assert rb is not None
    assert rb.get_name() == "box"
    assert world.get_rigid_body_count() == 1


def test_multiple_objects():
    """Test creating multiple objects in a world."""
    import dartpy7 as d

    world = d.World()

    # Create multiple multibodies
    mb1 = world.add_multi_body("robot1")
    mb2 = world.add_multi_body("robot2")

    # Create multiple rigid bodies
    rb1 = world.add_rigid_body("box1")
    rb2 = world.add_rigid_body("box2")

    assert world.get_multi_body_count() == 2
    assert world.get_rigid_body_count() == 2

    # Verify names
    assert mb1.get_name() == "robot1"
    assert mb2.get_name() == "robot2"
    assert rb1.get_name() == "box1"
    assert rb2.get_name() == "box2"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
