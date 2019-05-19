import platform
import pytest
import numpy as np
import dartpy as dart


def test_basic():
    world_frame = dart.dynamics.Frame.World()
    assert world_frame.isWorld()

    frame = dart.dynamics.SimpleFrame()
    assert not frame.isWorld()
    assert not frame.isShapeNode()
    assert frame.isShapeFrame()

    assert frame.getTransform().translation()[0] == pytest.approx(0)
    assert frame.getTransform().translation()[1] == pytest.approx(0)
    assert frame.getTransform().translation()[2] == pytest.approx(0)

    frame.setTranslation([1, 2, 3])
    assert frame.getTransform().translation()[0] == pytest.approx(1)
    assert frame.getTransform().translation()[1] == pytest.approx(2)
    assert frame.getTransform().translation()[2] == pytest.approx(3)


if __name__ == "__main__":
    pytest.main()
