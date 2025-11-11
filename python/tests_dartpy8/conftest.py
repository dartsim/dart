"""Pytest configuration for dartpy8 tests."""

import pytest


@pytest.fixture
def dart():
    """Fixture that imports and returns the dartpy8 module."""
    import dartpy8

    return dartpy8


@pytest.fixture
def world(dart):
    """Fixture that creates and returns a fresh World instance."""
    return dart.World()
