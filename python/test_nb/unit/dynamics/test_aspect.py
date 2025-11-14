import dartpy as dart
import pytest


def test_simple_frame():
    shape_frame = dart.dynamics.SimpleFrame()
    assert not shape_frame.hasVisualAspect()
    assert shape_frame.getVisualAspect() is None
    assert shape_frame.getVisualAspect(False) is None
    visual = shape_frame.createVisualAspect()
    assert visual is not None


if __name__ == "__main__":
    pytest.main()
