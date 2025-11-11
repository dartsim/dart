#!/usr/bin/env python3
"""Tests for dartpy8 module import."""

import pytest


def test_import():
    """Test that dartpy8 can be imported."""
    import dartpy8

    assert dartpy8 is not None


def test_import_world():
    """Test that World class can be imported."""
    from dartpy8 import World

    assert World is not None


def test_import_multibody():
    """Test that MultiBody class can be imported."""
    from dartpy8 import MultiBody

    assert MultiBody is not None


def test_import_rigidbody():
    """Test that RigidBody class can be imported."""
    from dartpy8 import RigidBody

    assert RigidBody is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
