#!/usr/bin/env python3
"""Tests for dartpy7 version information."""

import pytest


def test_version_string():
    """Test version string is available."""
    import dartpy7 as d

    # Check version string
    assert d.__version__ is not None
    assert isinstance(d.__version__, str)
    assert "7.0.0" in d.__version__


def test_version_major():
    """Test major version number."""
    import dartpy7 as d

    assert d.version_major() == 7


def test_version_minor():
    """Test minor version number."""
    import dartpy7 as d

    assert d.version_minor() == 0


def test_version_patch():
    """Test patch version number."""
    import dartpy7 as d

    assert d.version_patch() == 0


def test_version_components():
    """Test all version components together."""
    import dartpy7 as d

    assert d.version_major() == 7
    assert d.version_minor() == 0
    assert d.version_patch() == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
