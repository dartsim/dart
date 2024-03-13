import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    world_frame = dart.dynamics.Frame.World()
    assert world_frame.isWorld()

    frame1 = dart.dynamics.SimpleFrame()
    assert not frame1.isWorld()
    assert not frame1.isShapeNode()
    assert frame1.isShapeFrame()

    assert frame1.getTransform().translation()[0] == pytest.approx(0)
    assert frame1.getTransform().translation()[1] == pytest.approx(0)
    assert frame1.getTransform().translation()[2] == pytest.approx(0)

    frame1.setTranslation([1, 2, 3])
    assert frame1.getTransform().translation()[0] == pytest.approx(1)
    assert frame1.getTransform().translation()[1] == pytest.approx(2)
    assert frame1.getTransform().translation()[2] == pytest.approx(3)

    assert frame1.getParentFrame().isWorld()
    assert frame1.descendsFrom(None)
    assert frame1.descendsFrom(world_frame)
    assert frame1.descendsFrom(frame1)

    frame2 = frame1.spawnChildSimpleFrame()
    assert not frame2.isWorld()
    assert frame2.descendsFrom(None)
    assert frame2.descendsFrom(world_frame)
    assert frame2.descendsFrom(frame1)
    assert frame2.descendsFrom(frame2)

    assert frame2.getTransform().translation()[0] == pytest.approx(1)
    assert frame2.getTransform().translation()[1] == pytest.approx(2)
    assert frame2.getTransform().translation()[2] == pytest.approx(3)

    frame2.setRelativeTranslation([1, 2, 3])
    assert frame2.getTransform().translation()[0] == pytest.approx(2)
    assert frame2.getTransform().translation()[1] == pytest.approx(4)
    assert frame2.getTransform().translation()[2] == pytest.approx(6)

    frame2.setTranslation([1, 2, 3])
    assert frame2.getTransform().translation()[0] == pytest.approx(1)
    assert frame2.getTransform().translation()[1] == pytest.approx(2)
    assert frame2.getTransform().translation()[2] == pytest.approx(3)


if __name__ == "__main__":
    pytest.main()
