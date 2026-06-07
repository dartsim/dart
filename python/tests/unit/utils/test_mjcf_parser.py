import dartpy as dart
import pytest


def test_read_world_removed():
    assert hasattr(dart.io.MjcfParser, "Options")
    assert not hasattr(dart.io.MjcfParser, "read_world")
    assert not hasattr(dart.io.MjcfParser, "readWorld")


if __name__ == "__main__":
    pytest.main()
