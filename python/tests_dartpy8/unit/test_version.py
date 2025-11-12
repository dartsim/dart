#!/usr/bin/env python3
"""Tests for dartpy8 version information."""

import pytest


def test_version_string():
    """Test version string is available."""
    import dartpy8 as d

    # Check version string
    assert d.__version__ is not None
    assert isinstance(d.__version__, str)
    assert "8.0.0" in d.__version__


def test_version_major():
    """Test major version number."""
    import dartpy8 as d

    assert d.version_major() == 8


def test_version_minor():
    """Test minor version number."""
    import dartpy8 as d

    assert d.version_minor() == 0


def test_version_patch():
    """Test patch version number."""
    import dartpy8 as d

    assert d.version_patch() == 0


def test_version_components():
    """Test all version components together."""
    import dartpy8 as d

    assert d.version_major() == 8
    assert d.version_minor() == 0
    assert d.version_patch() == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
