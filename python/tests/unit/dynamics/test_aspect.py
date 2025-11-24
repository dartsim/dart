import dartpy as dart
import pytest


def test_simple_frame():
    shape_frame = dart.dynamics.SimpleFrame()
    assert not shape_frame.has_visual_aspect()
    assert shape_frame.get_visual_aspect() is None
    assert shape_frame.get_visual_aspect(False) is None
    visual = shape_frame.create_visual_aspect()
    assert visual is not None


if __name__ == "__main__":
    pytest.main()
