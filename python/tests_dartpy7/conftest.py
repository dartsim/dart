"""Pytest configuration for dartpy7 tests."""

import pytest


@pytest.fixture
def dart():
    """Fixture that imports and returns the dartpy7 module."""
    import dartpy7

    return dartpy7


@pytest.fixture
def world(dart):
    """Fixture that creates and returns a fresh World instance."""
    return dart.World()
