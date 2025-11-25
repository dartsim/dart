import dartpy as dart
import pytest


def test_simple_frame():
    shape_frame = dart.SimpleFrame()
    visual = shape_frame.get_visual_aspect()
    assert visual is not None
    assert shape_frame.has_visual_aspect()
    assert shape_frame.get_visual_aspect(False) is not None


if __name__ == "__main__":
    pytest.main()
